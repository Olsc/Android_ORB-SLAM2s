/*
 * Copyright (C) 2026 Olsc <OlscStudio@outlook.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.orb.slam2s.util;

import android.opengl.Matrix;
import android.util.Log;

// 姿态传感器融合跟踪器：视觉良好时透传 SLAM 位姿，跟踪丢失时视图跟随传感器姿态。
// 结构与 Android_3dof 一致（视图 = 姿态矩阵 × 冻结偏移），姿态矩阵前的换基由视觉增量在线确定。
public class SensorFusionTracker {
    private static final String TAG = "SensorFusionTracker";

    // 换基候选：相机与屏幕的安装只可能是 48 种有符号轴置换之一（含镜像安装）
    private static final int CANDIDATE_COUNT = 48;
    // 参与标定的单帧旋转角窗口，过小则噪声占主导，过大则多为丢帧后跳变
    private static final float CALIB_MIN_ANGLE_DEG = 2.0f;
    private static final float CALIB_MAX_ANGLE_DEG = 40.0f;
    // 累计到该帧数后开始采用评分最高的候选
    private static final int CALIB_MIN_SAMPLES = 15;
    // 保持态不可用时的日志节流间隔（帧）
    private static final int HOLD_SKIP_LOG_INTERVAL = 60;

    private boolean m3DofAssistEnabled = false; // 初始状态由宿主根据传感器硬件探测结果决定 (默认启动)

    // 解析初值：rotationDegrees 为顺时针角度，相机帧手性相反故取负
    private final float[] mAnalyticRot = new float[16];
    private int mCameraFrameDegrees = -1;

    // 在线标定状态
    private final float[][] mCandidates = new float[CANDIDATE_COUNT][16];
    private final float[] mCandidateDet = new float[CANDIDATE_COUNT];
    private final double[] mCandidateCost = new double[CANDIDATE_COUNT];
    private int mCalibSamples = 0;
    private int mCalibBest = -1;
    private int mHoldSkipCount = 0;
    private float[] mConjugation = mAnalyticRot;

    private final float[] mPrevSensorRot = new float[16];
    private final float[] mPrevViewRot = new float[16];
    private boolean mHasPrevPair = false;

    // 冻结偏移：R_view = M(t) · R_offset，M(t) = 换基 · R_sensor(t)ᵀ
    private final float[] mAlignmentOffset = new float[16];
    private boolean mHasAlignment = false;

    // 缓存最后一次视觉有效跟踪时的状态 (用于 Lost-Hold 兜底)
    private final float[] mLastValidModel = new float[16];
    private final float[] mLastValidProj = new float[16];
    private final float[] mLastValidTranslation = new float[3]; // [tx, ty, tz]
    private int mLastValidMapId = -1;
    private boolean mHasValidState = false;

    // 矩阵计算复用缓存，避免高频渲染循环中GC开销
    private final float[] mTempSensorTrans = new float[16];
    private final float[] mTempSensorView = new float[16];
    private final float[] mTempViewDelta = new float[16];
    private final float[] mTempOffsetInst = new float[16];
    private final float[] mTempFusedRot = new float[16];
    private final float[] mTempConj = new float[16];
    private final float[] mTempPureRot = new float[16];

    public SensorFusionTracker() {
        buildCandidates();
        Matrix.setIdentityM(mAnalyticRot, 0);
        Matrix.setIdentityM(mAlignmentOffset, 0);
        Matrix.setIdentityM(mLastValidModel, 0);
        Matrix.setIdentityM(mLastValidProj, 0);
    }

    // 开启或关闭 3DOF 传感器辅助功能
    public synchronized void set3DofAssistEnabled(boolean enabled) {
        if (this.m3DofAssistEnabled != enabled) {
            this.m3DofAssistEnabled = enabled;
            Log.i(TAG, "3DOF 传感器辅助定位状态切换为: " + (enabled ? "开启" : "关闭"));
            if (!enabled) {
                mHasAlignment = false;
                mHasValidState = false;
                mHasPrevPair = false;
            }
        }
    }

    // 记录传感器可观测自由度，3 含航向，2 仅俯仰与滚转，仅用于诊断日志
    public synchronized void setRotationDof(int dof) {
        Log.i(TAG, "传感器姿态自由度: " + dof);
    }

    // 设置相机帧与设备帧的安装夹角，取值来自 CameraX 的 rotationDegrees；同时作为标定初值
    public synchronized void setCameraFrameRotation(int rotationDegrees) {
        final int degrees = ((rotationDegrees % 360) + 360) % 360;
        if (degrees == mCameraFrameDegrees) {
            return;
        }
        mCameraFrameDegrees = degrees;
        buildAnalyticRotation(degrees);
        resetCalibration();
        Log.i(TAG, "相机图像帧相对设备帧旋转: " + degrees);
    }

    // 每帧融合与解耦计算：透传视觉位姿或在丢失时视图跟随传感器，返回是否绘制 AR 物体。
    // 入参为绘制标志、跟踪状态、地图 ID、原始 MVP、传感器旋转矩阵与就绪标志，输出 Model/View/Proj 矩阵。
    public synchronized boolean update(boolean rawDrawFlag, int trackingState, int activeMapId,
                                      float[] rawMvp48, float[] sensorRot, boolean isSensorReady,
                                      float[] outModel, float[] outView, float[] outProj) {
        if (rawMvp48 == null || rawMvp48.length < 48 || outModel == null || outView == null || outProj == null) {
            return false;
        }

        // 提取视觉 SLAM 原始矩阵
        final int vViewOffset = 16;
        final int vProjOffset = 32;

        // 视觉当前是否处于良好跟踪并且可绘制状态
        final boolean isVisionTrackingOk = (trackingState == 2 && rawDrawFlag);

        // 如果未启用 3DOF 辅助：完全透传纯视觉 SLAM
        if (!m3DofAssistEnabled) {
            if (rawDrawFlag) {
                System.arraycopy(rawMvp48, 0, outModel, 0, 16);
                System.arraycopy(rawMvp48, vViewOffset, outView, 0, 16);
                System.arraycopy(rawMvp48, vProjOffset, outProj, 0, 16);
                return true;
            }
            return false;
        }

        // 1. 多地图识别与切换检测：地图 ID 变化时丢弃旧地图的保持状态（绝不跨地图保活）
        if (activeMapId != mLastValidMapId) {
            if (mHasValidState) {
                Log.i(TAG, "多地图切换确认: 退出旧地图 " + mLastValidMapId + " 的保持状态 (当前地图 " + activeMapId + ")");
                mHasAlignment = false;
                mHasValidState = false;
                mHasPrevPair = false;
            }
            mLastValidMapId = activeMapId;
        }

        // 2. 情况 A：视觉 SLAM 正在正常跟踪，透传视觉位姿并刷新冻结偏移
        if (isVisionTrackingOk) {
            System.arraycopy(rawMvp48, 0, mLastValidModel, 0, 16);
            System.arraycopy(rawMvp48, vProjOffset, mLastValidProj, 0, 16);

            // 提取视觉 View 矩阵的平移 [tx, ty, tz]
            mLastValidTranslation[0] = rawMvp48[vViewOffset + 12];
            mLastValidTranslation[1] = rawMvp48[vViewOffset + 13];
            mLastValidTranslation[2] = rawMvp48[vViewOffset + 14];

            // 提取视觉 View 的纯旋转分量 (将最后一列设为 [0,0,0,1]^T)
            System.arraycopy(rawMvp48, vViewOffset, mTempPureRot, 0, 16);
            mTempPureRot[12] = 0.0f;
            mTempPureRot[13] = 0.0f;
            mTempPureRot[14] = 0.0f;
            mTempPureRot[15] = 1.0f;

            final boolean hasSensor = (isSensorReady && sensorRot != null);
            if (hasSensor) {
                // 先让标定收敛，再用最新换基算偏移，避免标定切换后本帧偏移失效
                updateCalibration(sensorRot);
                buildSensorView(sensorRot);
                Matrix.transposeM(mTempSensorTrans, 0, mTempSensorView, 0);
                Matrix.multiplyMM(mTempOffsetInst, 0, mTempSensorTrans, 0, mTempPureRot, 0);
                System.arraycopy(mTempOffsetInst, 0, mAlignmentOffset, 0, 16);
                mHasAlignment = true;
            }
            mHasValidState = hasSensor;

            // 视觉稳定跟踪时严格使用视觉 SLAM 位姿，避免传感器时延与微抖
            System.arraycopy(rawMvp48, 0, outModel, 0, 16);
            System.arraycopy(rawMvp48, vViewOffset, outView, 0, 16);
            System.arraycopy(rawMvp48, vProjOffset, outProj, 0, 16);
            return true;
        }

        // 3. 情况 B：视觉 SLAM 丢失跟踪，视图旋转直接跟随传感器姿态 (Lost-Hold)
        if (mHasValidState && mHasAlignment && isSensorReady && sensorRot != null) {
            System.arraycopy(mLastValidModel, 0, outModel, 0, 16);
            System.arraycopy(mLastValidProj, 0, outProj, 0, 16);

            // View 矩阵：旋转由传感器姿态换算到相机帧后叠加冻结偏移，平移沿用丢失前的估计
            buildSensorView(sensorRot);
            Matrix.multiplyMM(mTempFusedRot, 0, mTempSensorView, 0, mAlignmentOffset, 0);
            System.arraycopy(mTempFusedRot, 0, outView, 0, 16);
            outView[12] = mLastValidTranslation[0];
            outView[13] = mLastValidTranslation[1];
            outView[14] = mLastValidTranslation[2];
            outView[15] = 1.0f;

            // 保活模式下强制绘制，确保 AR 物体不闪退
            return true;
        }

        // 既无当前视觉也无历史对齐基准时，无法绘制
        if ((++mHoldSkipCount % HOLD_SKIP_LOG_INTERVAL) == 0) {
            Log.w(TAG, "保持态不可用: 视觉基准=" + mHasValidState + " 对齐=" + mHasAlignment
                    + " 传感器就绪=" + isSensorReady + " 传感器矩阵=" + (sensorRot != null));
        }
        return false;
    }

    // 传感器姿态换算到相机图像帧：M = 换基 · R_sensorᵀ
    private void buildSensorView(float[] sensorRot) {
        Matrix.transposeM(mTempSensorTrans, 0, sensorRot, 0);
        Matrix.multiplyMM(mTempSensorView, 0, mConjugation, 0, mTempSensorTrans, 0);
    }

    // 用相邻两帧的视觉增量与传感器增量配对，为全部候选换基打分选出最优
    private void updateCalibration(float[] sensorRot) {
        if (!mHasPrevPair) {
            System.arraycopy(sensorRot, 0, mPrevSensorRot, 0, 16);
            System.arraycopy(mTempPureRot, 0, mPrevViewRot, 0, 16);
            mHasPrevPair = true;
            return;
        }

        // Δs = R_sensor(t)ᵀ·R_sensor(t-1)
        Matrix.transposeM(mTempSensorTrans, 0, sensorRot, 0);
        Matrix.multiplyMM(mTempSensorView, 0, mTempSensorTrans, 0, mPrevSensorRot, 0);

        final float angle = rotationAngleDeg(mTempSensorView);
        if (angle < CALIB_MIN_ANGLE_DEG || angle > CALIB_MAX_ANGLE_DEG) {
            System.arraycopy(sensorRot, 0, mPrevSensorRot, 0, 16);
            System.arraycopy(mTempPureRot, 0, mPrevViewRot, 0, 16);
            return;
        }

        // Δv = R_view(t)·R_view(t-1)ᵀ，候选应满足 Δv ≈ C·Δs·Cᵀ
        Matrix.transposeM(mTempSensorTrans, 0, mPrevViewRot, 0);
        Matrix.multiplyMM(mTempViewDelta, 0, mTempPureRot, 0, mTempSensorTrans, 0);

        for (int c = 0; c < CANDIDATE_COUNT; c++) {
            final float[] candidate = mCandidates[c];
            Matrix.multiplyMM(mTempFusedRot, 0, candidate, 0, mTempSensorView, 0);
            Matrix.transposeM(mTempOffsetInst, 0, candidate, 0);
            Matrix.multiplyMM(mTempConj, 0, mTempFusedRot, 0, mTempOffsetInst, 0);
            mCandidateCost[c] += 1.0 - rotationCosine(mTempConj, mTempViewDelta);
        }
        mCalibSamples++;

        System.arraycopy(sensorRot, 0, mPrevSensorRot, 0, 16);
        System.arraycopy(mTempPureRot, 0, mPrevViewRot, 0, 16);

        if (mCalibSamples < CALIB_MIN_SAMPLES) {
            return;
        }
        int best = 0;
        for (int c = 1; c < CANDIDATE_COUNT; c++) {
            if (mCandidateCost[c] < mCandidateCost[best]) {
                best = c;
            }
        }
        // 需明显优于当前候选才切换，避免误杀稳定状态
        final boolean better = (mCalibBest < 0)
                || (mCandidateCost[best] < mCandidateCost[mCalibBest] * 0.85);
        if (best != mCalibBest && better) {
            mCalibBest = best;
            mConjugation = mCandidates[best];
            Log.i(TAG, "换基标定: 候选 " + best + " 行列式=" + Math.round(mCandidateDet[best])
                    + " 样本=" + mCalibSamples + " 误差=" + String.format("%.3f", mCandidateCost[best]));
        }
    }

    // 换基变化后旧标定与偏移均失效，重新累计
    private void resetCalibration() {
        for (int c = 0; c < CANDIDATE_COUNT; c++) {
            mCandidateCost[c] = 0.0;
        }
        mCalibSamples = 0;
        mCalibBest = -1;
        mConjugation = mAnalyticRot;
        mHasPrevPair = false;
        mHasAlignment = false;
        mHasValidState = false;
    }

    // 绕光轴构造解析初值 Rz(-degrees)，列主序
    private void buildAnalyticRotation(int degrees) {
        final double rad = Math.toRadians(-degrees);
        final float c = (float) Math.cos(rad);
        final float s = (float) Math.sin(rad);
        Matrix.setIdentityM(mAnalyticRot, 0);
        mAnalyticRot[0] = c;
        mAnalyticRot[1] = s;
        mAnalyticRot[4] = -s;
        mAnalyticRot[5] = c;
    }

    // 生成全部 48 种有符号轴置换（含行列式为负的镜像安装）
    private void buildCandidates() {
        int index = 0;
        final int[][] perms = {{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
        for (int[] perm : perms) {
            for (int signBits = 0; signBits < 8; signBits++) {
                final int sx = ((signBits & 1) == 0) ? 1 : -1;
                final int sy = ((signBits & 2) == 0) ? 1 : -1;
                final int sz = ((signBits & 4) == 0) ? 1 : -1;
                final float[] m = mCandidates[index];
                Matrix.setIdentityM(m, 0);
                m[perm[0] * 4 + 0] = sx;
                m[perm[1] * 4 + 1] = sy;
                m[perm[2] * 4 + 2] = sz;
                mCandidateDet[index] = (float) (sx * sy * sz * permutationSign(perm));
                index++;
            }
        }
        if (index != CANDIDATE_COUNT) {
            Log.e(TAG, "换基候选构造异常: " + index);
        }
    }

    // 置换的符号：偶置换返回 1，奇置换返回 -1
    private static int permutationSign(int[] perm) {
        final int inversion = (perm[0] > perm[1] ? 1 : 0)
                + (perm[0] > perm[2] ? 1 : 0)
                + (perm[1] > perm[2] ? 1 : 0);
        return (inversion % 2 == 0) ? 1 : -1;
    }

    // 旋转矩阵的旋转角，单位度
    private static float rotationAngleDeg(float[] m) {
        final float trace = m[0] + m[5] + m[10];
        float cos = (trace - 1.0f) * 0.5f;
        cos = Math.max(-1.0f, Math.min(1.0f, cos));
        return (float) Math.toDegrees(Math.acos(cos));
    }

    // 两个旋转矩阵的相似度，等价于夹角的余弦
    private static double rotationCosine(float[] a, float[] b) {
        double sum = 0.0;
        for (int col = 0; col < 3; col++) {
            final int base = col * 4;
            sum += a[base] * b[base] + a[base + 1] * b[base + 1] + a[base + 2] * b[base + 2];
        }
        return sum / 3.0;
    }
}
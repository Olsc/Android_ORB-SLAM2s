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

// 姿态传感器融合跟踪器：视觉良好时透传 SLAM 位姿，跟踪丢失时视图直接跟随传感器姿态。
public class SensorFusionTracker {
    private static final String TAG = "SensorFusionTracker";

    private boolean m3DofAssistEnabled = false; // 初始状态由宿主根据传感器硬件探测结果决定 (默认启动)

    // 相机图像帧相对设备帧的安装夹角（绕光轴），由 rotationDegrees 决定
    private int mCameraFrameDegrees = -1;

    // 安装角候选：绕光轴 rotationDegrees + k·90°，每个角度再分“传感器矩阵取转置/不取转置”
    // 两种手性约定，共 8 个。真实夹角取决于屏幕旋转与相机安装（参见 Android_3dof 的
    // getRemappedMatrix），纯解析推导容易整体差 90°，表现为“上下移动变成左右移动”：
    // 因为丢失态施加的是共轭旋转 C·Δ·Cᵀ，C 差 90° 会把俯仰轴映射成横滚轴。
    // 因此运行期用视觉增量在候选间择优，不靠硬编码。
    private static final int MOUNT_ANGLE_COUNT = 4;
    private static final int MOUNT_CANDIDATE_COUNT = MOUNT_ANGLE_COUNT * 2;
    // 参与标定的单帧转角窗口：过小则噪声主导，过大则多为丢帧/重定位跳变
    private static final float CALIB_MIN_ANGLE_DEG = 0.8f;
    private static final float CALIB_MAX_ANGLE_DEG = 30.0f;
    // 累计到该样本数后开始择优
    private static final int CALIB_MIN_SAMPLES = 10;
    // 需明显更优才切换候选，避免来回抖动
    private static final double CALIB_SWITCH_RATIO = 0.6;

    private final float[][] mMountRot = new float[MOUNT_ANGLE_COUNT][16];
    private final double[] mMountCost = new double[MOUNT_CANDIDATE_COUNT];
    private int mMountIndex = 0;
    private int mMountSamples = 0;
    private boolean mHasPrevPair = false;
    private final float[] mPrevSensorRot = new float[16];
    private final float[] mPrevViewRot = new float[16];

    // 冻结偏移：R_view = M(t) · R_offset，M(t) = 安装夹角 · R_sensor(t)ᵀ
    private final float[] mAlignmentOffset = new float[16];
    private boolean mHasAlignment = false;

    // 缓存最后一次视觉有效跟踪时的状态 (用于 Lost-Hold 兜底)
    private final float[] mLastValidModel = new float[16];
    private final float[] mLastValidView = new float[16];
    private final float[] mLastValidProj = new float[16];
    private final float[] mLastValidTranslation = new float[3]; // [tx, ty, tz]
    private int mLastValidMapId = -1;
    private boolean mHasValidState = false;
    private boolean mHasLastPose = false;
    private boolean mHoldUnavailableLogged = false;

    // AR 锚点所属的地图 ID：3DOF 保持只允许发生在这张地图内，-1 表示当前还没有锚点
    private int mAnchorMapId = -1;
    // 最近一次视觉有效跟踪的时刻，用于限定冻结位姿的宽限时间
    private long mLastVisionOkNanos = 0L;

    // 视觉失效后仍允许沿用冻结位姿的宽限时间，仅用于平滑切换，不会长期悬挂 AR 物体
    private static final long LOST_GRACE_NANOS = 250_000_000L;

    // 矩阵计算复用缓存，避免高频渲染循环中GC开销
    private final float[] mTempSensorTrans = new float[16];
    private final float[] mTempSensorView = new float[16];
    private final float[] mTempOffsetInst = new float[16];
    private final float[] mTempFusedRot = new float[16];
    private final float[] mTempPureRot = new float[16];
    private final float[] mTempViewDelta = new float[16];
    private final float[] mTempConj = new float[16];

    public SensorFusionTracker() {
        buildMountRotations(0);
        Matrix.setIdentityM(mAlignmentOffset, 0);
        Matrix.setIdentityM(mLastValidModel, 0);
        Matrix.setIdentityM(mLastValidProj, 0);
    }

    // 开启或关闭 3DOF 传感器辅助功能
    public synchronized void set3DofAssistEnabled(boolean enabled) {
        if (this.m3DofAssistEnabled != enabled) {
            this.m3DofAssistEnabled = enabled;
            Log.i(TAG, "辅助定位状态切换为: " + (enabled ? "开启" : "关闭"));
            if (!enabled) {
                resetHoldState();
                mAnchorMapId = -1;
            }
        }
    }

    // 记录传感器可观测自由度，3 含航向，2 仅俯仰与滚转，仅用于诊断日志
    public synchronized void setRotationDof(int dof) {
        Log.i(TAG, "传感器姿态自由度: " + dof);
    }

    // 设置相机帧与设备帧的安装夹角，取值来自 CameraX 的 rotationDegrees
    public synchronized void setCameraFrameRotation(int rotationDegrees) {
        final int degrees = ((rotationDegrees % 360) + 360) % 360;
        if (degrees == mCameraFrameDegrees) {
            return;
        }
        mCameraFrameDegrees = degrees;
        buildMountRotations(degrees);
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

        // 视觉是否拿到了有效位姿：只看 SLAM 跟踪状态。
        // 这里不能再与 rawDrawFlag 相与：重定位到子地图后 rawDrawFlag 会因为"该地图没有 AR 锚点"
        // 而为 false，若据此认为视觉尚未恢复，AR 物体就会一直挂在 3DOF 保持态上而不跟随点云。
        final boolean isVisionTrackingOk = (trackingState == 2);

        // AR 锚点只属于放置它的那张地图，跨地图不得保活
        final boolean isAnchorOnActiveMap = (mAnchorMapId >= 0 && activeMapId == mAnchorMapId);

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

        // 1. 多地图识别：进入新地图（重定位失败新建的子地图等）后原地图的 AR 锚点已不存在，
        //    必须立刻丢弃保持态，否则会出现"扫到子地图、没有 AR 点位却仍显示 AR 物体并维持 3DOF"
        if (activeMapId != mLastValidMapId) {
            if (mLastValidMapId >= 0 && activeMapId != mAnchorMapId) {
                Log.i(TAG, "多地图切换: 退出旧地图 " + mLastValidMapId + " 的保持状态 (当前地图 " + activeMapId
                        + "，锚点地图 " + mAnchorMapId + ")");
                resetHoldState();
            }
            mLastValidMapId = activeMapId;
        }

        // 2. 情况 A：视觉 SLAM 正在正常跟踪，透传视觉位姿并刷新冻结偏移
        if (isVisionTrackingOk) {
            // Model 仅在视觉真正画出 AR 物体时才有意义：跟踪失败的帧里原生端会下发单位矩阵，
            // 若盲目缓存会在丢失保活时把物体画到世界原点
            if (rawDrawFlag) {
                System.arraycopy(rawMvp48, 0, mLastValidModel, 0, 16);
            }
            System.arraycopy(rawMvp48, vViewOffset, mLastValidView, 0, 16);
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
                updateMountCalibration(sensorRot, mTempPureRot);
                buildSensorView(sensorRot);
                Matrix.transposeM(mTempSensorTrans, 0, mTempSensorView, 0);
                Matrix.multiplyMM(mTempOffsetInst, 0, mTempSensorTrans, 0, mTempPureRot, 0);
                System.arraycopy(mTempOffsetInst, 0, mAlignmentOffset, 0, 16);
                mHasAlignment = true;
            }
            mHasValidState = hasSensor;
            mHasLastPose = true;
            mLastVisionOkNanos = System.nanoTime();
            mHoldUnavailableLogged = false;

            // 视觉真的画出了 AR 锚点时才认为"锚点属于当前地图"，供后续丢失时判断能否保活
            if (rawDrawFlag && mAnchorMapId != activeMapId) {
                Log.i(TAG, "AR 锚点绑定到地图 " + activeMapId);
                mAnchorMapId = activeMapId;
            }

            // 视觉稳定跟踪时严格使用视觉 SLAM 位姿，避免传感器时延与微抖；
            // 是否绘制完全由视觉侧决定（子地图里没有锚点时必须隐藏，不能靠 3DOF 硬撑）
            System.arraycopy(rawMvp48, 0, outModel, 0, 16);
            System.arraycopy(rawMvp48, vViewOffset, outView, 0, 16);
            System.arraycopy(rawMvp48, vProjOffset, outProj, 0, 16);
            return rawDrawFlag;
        }

        // 3. 情况 B：视觉 SLAM 丢失跟踪，视图旋转直接跟随传感器姿态 (Lost-Hold)
        //    仅在锚点所在的地图内允许保活，避免把 AR 物体带进没有该点位的新地图
        if (isAnchorOnActiveMap && mHasValidState && mHasAlignment && isSensorReady && sensorRot != null) {
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
            mHoldUnavailableLogged = false;
            return true;
        }

        // 兜底：锚点还在当前地图且刚刚丢失跟踪时，改用冻结姿态短时续画，避免物体闪退；
        // 超过宽限时间后必须隐藏，不能让 AR 物体长期停留在错误位置
        if (isAnchorOnActiveMap && mHasLastPose
                && (System.nanoTime() - mLastVisionOkNanos) <= LOST_GRACE_NANOS) {
            System.arraycopy(mLastValidModel, 0, outModel, 0, 16);
            System.arraycopy(mLastValidProj, 0, outProj, 0, 16);
            System.arraycopy(mLastValidView, 0, outView, 0, 16);
            if (!mHoldUnavailableLogged) {
                mHoldUnavailableLogged = true;
                Log.w(TAG, "保持态不可用，短时改用冻结姿态: 基准=" + mHasValidState + " 对齐=" + mHasAlignment
                        + " 传感器就绪=" + isSensorReady + " 地图=" + activeMapId);
            }
            return true;
        }

        if (mHasLastPose && !mHoldUnavailableLogged) {
            mHoldUnavailableLogged = true;
            Log.w(TAG, "保持态已失效，隐藏 AR 物体: 跟踪状态=" + trackingState + " 视觉可绘制=" + rawDrawFlag
                    + " 地图=" + activeMapId + " 锚点地图=" + mAnchorMapId
                    + " 传感器就绪=" + isSensorReady);
        }

        // 视觉位姿与 3DOF 保持都不可用：必须隐藏，绝不返回陈旧位姿
        return false;
    }

    // 丢弃保持态：地图切换（尤其是新建子地图）后原锚点的位姿基准已无意义
    private void resetHoldState() {
        mHasAlignment = false;
        mHasValidState = false;
        mHasLastPose = false;
        mHoldUnavailableLogged = false;
    }

    // 传感器姿态换算到相机图像帧：M(t) = 安装角 · R_sensor(t)（或 · R_sensor(t)ᵀ，取决于测得的手性约定）
    private void buildSensorView(float[] sensorRot) {
        if ((mMountIndex & 1) == 0) {
            Matrix.transposeM(mTempSensorTrans, 0, sensorRot, 0);
        } else {
            System.arraycopy(sensorRot, 0, mTempSensorTrans, 0, 16);
        }
        Matrix.multiplyMM(mTempSensorView, 0, mMountRot[mMountIndex >> 1], 0, mTempSensorTrans, 0);
    }

    // 用视觉增量与传感器增量的换基一致性为候选打分：Δv ≈ C·Δs·Cᵀ（Δs 为 R_s(t)ᵀ·R_s(t-1)）
    private void updateMountCalibration(float[] sensorRot, float[] pureViewRot) {
        if (!mHasPrevPair) {
            System.arraycopy(sensorRot, 0, mPrevSensorRot, 0, 16);
            System.arraycopy(pureViewRot, 0, mPrevViewRot, 0, 16);
            mHasPrevPair = true;
            return;
        }

        // Δs = R_sensor(t)ᵀ · R_sensor(t-1)，即设备系下的单帧姿态增量
        Matrix.transposeM(mTempSensorTrans, 0, sensorRot, 0);
        Matrix.multiplyMM(mTempSensorView, 0, mTempSensorTrans, 0, mPrevSensorRot, 0);

        // Δv = R_view(t) · R_view(t-1)ᵀ，即视觉实测的相机旋转增量
        Matrix.transposeM(mTempOffsetInst, 0, mPrevViewRot, 0);
        Matrix.multiplyMM(mTempViewDelta, 0, pureViewRot, 0, mTempOffsetInst, 0);

        System.arraycopy(sensorRot, 0, mPrevSensorRot, 0, 16);
        System.arraycopy(pureViewRot, 0, mPrevViewRot, 0, 16);

        final float sensorAngle = rotationAngleDeg(mTempSensorView);
        final float viewAngle = rotationAngleDeg(mTempViewDelta);
        if (sensorAngle < CALIB_MIN_ANGLE_DEG || viewAngle < CALIB_MIN_ANGLE_DEG
                || sensorAngle > CALIB_MAX_ANGLE_DEG || viewAngle > CALIB_MAX_ANGLE_DEG) {
            return;
        }

        for (int c = 0; c < MOUNT_CANDIDATE_COUNT; c++) {
            final float[] candidate = mMountRot[c >> 1];
            // 偶数候选预测 C·Δs·Cᵀ，奇数候选预测 C·Δsᵀ·Cᵀ（另一种传感器手性约定）
            if ((c & 1) == 0) {
                System.arraycopy(mTempSensorView, 0, mTempConj, 0, 16);
            } else {
                Matrix.transposeM(mTempConj, 0, mTempSensorView, 0);
            }
            Matrix.multiplyMM(mTempFusedRot, 0, candidate, 0, mTempConj, 0);
            Matrix.transposeM(mTempSensorTrans, 0, candidate, 0);
            Matrix.multiplyMM(mTempConj, 0, mTempFusedRot, 0, mTempSensorTrans, 0);
            mMountCost[c] += 1.0 - rotationCosine(mTempConj, mTempViewDelta);
        }
        mMountSamples++;

        if (mMountSamples < CALIB_MIN_SAMPLES) {
            return;
        }
        int best = 0;
        for (int c = 1; c < MOUNT_CANDIDATE_COUNT; c++) {
            if (mMountCost[c] < mMountCost[best]) {
                best = c;
            }
        }
        if (best != mMountIndex && mMountCost[best] < mMountCost[mMountIndex] * CALIB_SWITCH_RATIO) {
            mMountIndex = best;
            // 换基变更后冻结偏移需重建，下一帧视觉正常时会立刻重建
            mHasAlignment = false;
            Log.i(TAG, "安装角标定: 采用候选 " + best + "（绕光轴 +" + (90 * (best >> 1))
                    + "°，传感器矩阵" + (((best & 1) == 0) ? "取转置" : "不取转置")
                    + "），样本=" + mMountSamples + " 代价=" + String.format("%.3f", mMountCost[best]));
        }
    }

    // 生成绕光轴的 4 个安装角候选（+0/90/180/270），并清空标定
    private void buildMountRotations(int degrees) {
        for (int k = 0; k < MOUNT_ANGLE_COUNT; k++) {
            final double rad = Math.toRadians(degrees + 90.0 * k);
            final float c = (float) Math.cos(rad);
            final float s = (float) Math.sin(rad);
            final float[] m = mMountRot[k];
            Matrix.setIdentityM(m, 0);
            m[0] = c;
            m[1] = s;
            m[4] = -s;
            m[5] = c;
        }
        for (int c = 0; c < MOUNT_CANDIDATE_COUNT; c++) {
            mMountCost[c] = 0.0;
        }
        mMountIndex = 0;
        mMountSamples = 0;
        mHasPrevPair = false;
        mHasAlignment = false;
        mHasValidState = false;
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
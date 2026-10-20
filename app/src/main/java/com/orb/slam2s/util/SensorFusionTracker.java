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

// 姿态传感器融合跟踪器：3DOF 辅助关闭时透传视觉 SLAM 位姿；
// 开启时正常跟踪透传视觉，跟踪丢失则平滑切换到传感器保持，并按地图切换重建兜底状态。
public class SensorFusionTracker {
    private static final String TAG = "SensorFusionTracker";

    private boolean m3DofAssistEnabled = false; // 初始状态由宿主根据传感器硬件探测结果决定 (默认启动)

    // 传感器校准对齐偏置矩阵 R_offset: R_view = R_sensor * R_offset
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
    private final float[] mTempOffsetInst = new float[16];
    private final float[] mTempFusedRot = new float[16];
    private final float[] mTempPureRot = new float[16];

    // 在线自适应平滑系数 (alpha 越小越平滑，主要依靠传感器防抖，低频向视觉收敛)
    private static final float ALIGN_SMOOTH_FACTOR = 0.06f;

    public SensorFusionTracker() {
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
                // 关闭时重置对齐与保活状态
                mHasAlignment = false;
                mHasValidState = false;
            }
        }
    }

    // 每帧融合与解耦计算：透传视觉位姿或在丢失时切换 3DOF 保持，返回是否绘制 AR 物体。
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

        // 开启了 3DOF 传感器辅助时的逻辑：

        // 1. 多地图识别与切换检测：如果检测到地图 ID 发生变化，重置旧地图的兜底状态（绝不跨地图保活）
        if (activeMapId != mLastValidMapId) {
            if (mHasValidState) {
                Log.i(TAG, "多地图切换确认: 退出旧地图 " + mLastValidMapId + " 的保持状态 (当前地图 " + activeMapId + ")");
                mHasAlignment = false;
                mHasValidState = false;
            }
            mLastValidMapId = activeMapId;
        }

        // 2. 情况 A：视觉 SLAM 正在正常跟踪 (isVisionTrackingOk == true)
        if (isVisionTrackingOk) {
            // 缓存最后一次有效的 Model 和 Proj 矩阵
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

            if (isSensorReady && sensorRot != null) {
                // 计算当前瞬时对齐偏移: R_offset = R_sensor^T * R_view
                Matrix.transposeM(mTempSensorTrans, 0, sensorRot, 0);
                Matrix.multiplyMM(mTempOffsetInst, 0, mTempSensorTrans, 0, mTempPureRot, 0);

                if (!mHasAlignment) {
                    // 初次对齐建立基准
                    System.arraycopy(mTempOffsetInst, 0, mAlignmentOffset, 0, 16);
                    mHasAlignment = true;
                    Log.d(TAG, "视觉与传感器完成初次基准对齐 (MapId=" + activeMapId + ")");
                } else {
                    // 在线平滑滤波更新 R_offset (低通消除传感器慢漂移，为丢失时兜底做准备)
                    smoothMatrix(mAlignmentOffset, mTempOffsetInst);
                }
            }

            // 视觉稳定跟踪时严格使用视觉 SLAM 位姿，避免传感器时延与微抖
            System.arraycopy(rawMvp48, 0, outModel, 0, 16);
            System.arraycopy(rawMvp48, vViewOffset, outView, 0, 16);
            System.arraycopy(rawMvp48, vProjOffset, outProj, 0, 16);
            mHasValidState = true;
            return true;
        }

        // 3. 情况 B：视觉 SLAM 突然丢失跟踪 (isVisionTrackingOk == false)
        // 触发纯 3DOF 空间保活兜底机制 (Lost-Hold)
        if (mHasValidState && mHasAlignment && isSensorReady && sensorRot != null) {
            // 保持最后的模型矩阵与投影矩阵
            System.arraycopy(mLastValidModel, 0, outModel, 0, 16);
            System.arraycopy(mLastValidProj, 0, outProj, 0, 16);

            // View 矩阵：平移保持丢失前的估计，旋转随设备传感器实时转动
            Matrix.multiplyMM(mTempFusedRot, 0, sensorRot, 0, mAlignmentOffset, 0);
            System.arraycopy(mTempFusedRot, 0, outView, 0, 16);
            outView[12] = mLastValidTranslation[0];
            outView[13] = mLastValidTranslation[1];
            outView[14] = mLastValidTranslation[2];
            outView[15] = 1.0f;

            // 保活模式下强制绘制，确保 AR 物体不闪退
            return true;
        }

        // 既无当前视觉也无历史对齐基准时，无法绘制
        return false;
    }

    // 对旋转矩阵进行一阶低通平滑并做 Gram-Schmidt 正交规范化
    private void smoothMatrix(float[] target, float[] source) {
        for (int i = 0; i < 16; i++) {
            target[i] = (1.0f - ALIGN_SMOOTH_FACTOR) * target[i] + ALIGN_SMOOTH_FACTOR * source[i];
        }

        // 正交规范化前三列 (X, Y, Z 轴)
        normalizeVector(target, 0); // X 列单位化

        // Y 列减去其在 X 列上的投影
        float dotXY = target[0] * target[4] + target[1] * target[5] + target[2] * target[6];
        target[4] -= dotXY * target[0];
        target[5] -= dotXY * target[1];
        target[6] -= dotXY * target[2];
        normalizeVector(target, 4); // Y 列单位化

        // Z 列 = X 列 * Y 列 (叉乘保证右手正交系)
        target[8]  = target[1] * target[6] - target[2] * target[5];
        target[9]  = target[2] * target[4] - target[0] * target[6];
        target[10] = target[0] * target[5] - target[1] * target[4];
        normalizeVector(target, 8);

        // 齐次第 4 列与第 4 行清理
        target[3] = 0.0f;
        target[7] = 0.0f;
        target[11] = 0.0f;
        target[12] = 0.0f;
        target[13] = 0.0f;
        target[14] = 0.0f;
        target[15] = 1.0f;
    }

    private void normalizeVector(float[] m, int offset) {
        float lenSq = m[offset] * m[offset] + m[offset + 1] * m[offset + 1] + m[offset + 2] * m[offset + 2];
        if (lenSq > 1e-8f) {
            float invLen = 1.0f / (float) Math.sqrt(lenSq);
            m[offset] *= invLen;
            m[offset + 1] *= invLen;
            m[offset + 2] *= invLen;
        }
    }
}
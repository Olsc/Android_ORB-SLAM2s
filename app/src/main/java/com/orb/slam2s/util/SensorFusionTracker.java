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
    private final float[] mCameraFrameRot = new float[16];
    private int mCameraFrameDegrees = -1;

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

    // 矩阵计算复用缓存，避免高频渲染循环中GC开销
    private final float[] mTempSensorTrans = new float[16];
    private final float[] mTempSensorView = new float[16];
    private final float[] mTempOffsetInst = new float[16];
    private final float[] mTempFusedRot = new float[16];
    private final float[] mTempPureRot = new float[16];

    public SensorFusionTracker() {
        Matrix.setIdentityM(mCameraFrameRot, 0);
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
                mHasAlignment = false;
                mHasValidState = false;
                mHasLastPose = false;
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
        buildCameraFrameRotation(degrees);
        mHasAlignment = false;
        mHasValidState = false;
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

        // 1. 多地图识别：仅在正常跟踪时确认切换才丢弃保持状态；
        //    丢失期间地图 ID 变化多为自动重置或子地图建立，不应打断保活
        if (activeMapId != mLastValidMapId) {
            if (mLastValidMapId >= 0 && isVisionTrackingOk && mHasValidState) {
                Log.i(TAG, "多地图切换确认: 退出旧地图 " + mLastValidMapId + " 的保持状态 (当前地图 " + activeMapId + ")");
                mHasAlignment = false;
                mHasValidState = false;
            }
            mLastValidMapId = activeMapId;
        }

        // 2. 情况 A：视觉 SLAM 正在正常跟踪，透传视觉位姿并刷新冻结偏移
        if (isVisionTrackingOk) {
            System.arraycopy(rawMvp48, 0, mLastValidModel, 0, 16);
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
                buildSensorView(sensorRot);
                Matrix.transposeM(mTempSensorTrans, 0, mTempSensorView, 0);
                Matrix.multiplyMM(mTempOffsetInst, 0, mTempSensorTrans, 0, mTempPureRot, 0);
                System.arraycopy(mTempOffsetInst, 0, mAlignmentOffset, 0, 16);
                mHasAlignment = true;
            }
            mHasValidState = hasSensor;
            mHasLastPose = true;
            mHoldUnavailableLogged = false;

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
            mHoldUnavailableLogged = false;
            return true;
        }

        // 兜底：曾经有过有效位姿时改用冻结姿态继续绘制，避免物体闪退
        if (mHasLastPose) {
            System.arraycopy(mLastValidModel, 0, outModel, 0, 16);
            System.arraycopy(mLastValidProj, 0, outProj, 0, 16);
            System.arraycopy(mLastValidView, 0, outView, 0, 16);
            if (!mHoldUnavailableLogged) {
                mHoldUnavailableLogged = true;
                Log.w(TAG, "保持态不可用，改用冻结姿态: 基准=" + mHasValidState + " 对齐=" + mHasAlignment
                        + " 传感器就绪=" + isSensorReady + " 地图=" + activeMapId);
            }
            return true;
        }

        // 从未获得过有效位姿，无法绘制
        return false;
    }

    // 传感器姿态换算到相机图像帧：M = 安装夹角 · R_sensorᵀ
    private void buildSensorView(float[] sensorRot) {
        Matrix.transposeM(mTempSensorTrans, 0, sensorRot, 0);
        Matrix.multiplyMM(mTempSensorView, 0, mCameraFrameRot, 0, mTempSensorTrans, 0);
    }

    // 绕光轴构造安装夹角，列主序；rotationDegrees 为顺时针角度，相机帧手性相反故取负
    private void buildCameraFrameRotation(int degrees) {
        final double rad = Math.toRadians(-degrees);
        final float c = (float) Math.cos(rad);
        final float s = (float) Math.sin(rad);
        Matrix.setIdentityM(mCameraFrameRot, 0);
        mCameraFrameRot[0] = c;
        mCameraFrameRot[1] = s;
        mCameraFrameRot[4] = -s;
        mCameraFrameRot[5] = c;
    }
}
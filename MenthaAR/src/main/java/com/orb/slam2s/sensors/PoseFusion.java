/**
 * Copyright (C) 2025-2026 Olsc <OlscStudio@outlook.com>
 *
 * This file is part of the Android ORB-SLAM2s project (a fork of ORB-SLAM2).
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

package com.orb.slam2s.sensors;

import android.opengl.Matrix;
import android.util.Log;

/**
 * 位姿融合器：视觉正常时透传 SLAM 位姿，跟踪丢失时由传感器姿态延续视图旋转。
 * 对外只暴露开关、相机安装角与方向反向三项配置，结果统一由 fuse 输出。
 */
public final class PoseFusion {
    private static final String TAG = "PoseFusion";

    private static final int VIEW_OFFSET = 16;
    private static final int PROJ_OFFSET = 32;

    // 视觉失效后沿用冻结位姿的宽限时间，仅用于平滑切换
    private static final long LOST_GRACE_NANOS = 250_000_000L;

    private boolean mEnabled = false;
    private boolean mSensorReverse = true; // 默认反向，与宿主默认开关保持一致
    private int mCameraImageDegrees = -1;

    private final float[] mMountRot = new float[16];

    // 冻结偏移：视图 = M(t) * offset，M(t) = 安装角 * R_sensor(t)ᵀ
    private final float[] mAlignmentOffset = new float[16];
    private boolean mHasAlignment = false;

    private final float[] mLastValidModel = new float[16];
    private final float[] mLastValidView = new float[16];
    private final float[] mLastValidProj = new float[16];
    private final float[] mLastValidTranslation = new float[3];
    private int mLastMapId = -1;
    private int mAnchorMapId = -1;
    private boolean mHasValidState = false;
    private boolean mHasLastPose = false;
    private boolean mHoldUnavailableLogged = false;
    private long mLastVisionOkNanos = 0L;
    private int mHoldLogCounter = 0;

    // 矩阵运算复用缓存，避免每帧产生 GC
    private final float[] mTempSensorTrans = new float[16];
    private final float[] mTempSensorView = new float[16];
    private final float[] mTempOffsetInst = new float[16];
    private final float[] mTempFusedRot = new float[16];
    private final float[] mTempPureRot = new float[16];

    public PoseFusion() {
        buildMountRotation(0);
        Matrix.setIdentityM(mAlignmentOffset, 0);
        Matrix.setIdentityM(mLastValidModel, 0);
        Matrix.setIdentityM(mLastValidProj, 0);
    }

    public synchronized void setEnabled(boolean enabled) {
        if (mEnabled != enabled) {
            mEnabled = enabled;
            Log.i(TAG, "姿态辅助: " + (enabled ? "开启" : "关闭"));
            // 开关切换会重启传感器，旧冻结偏移与新姿态不再匹配，必须丢弃
            resetHoldState();
            mAnchorMapId = -1;
        }
    }

    // 传感器重启后姿态基准已重置，冻结偏移随之失效，需由下一帧视觉重建
    public synchronized void resetAlignment() {
        mHasAlignment = false;
        mHasValidState = false;
    }

    public synchronized boolean isEnabled() {
        return mEnabled;
    }

    // 相机图像相对设备坐标系的旋转角，来自 CameraX 的 rotationDegrees
    public synchronized void setCameraImageRotation(int rotationDegrees) {
        final int degrees = ((rotationDegrees % 360) + 360) % 360;
        if (degrees == mCameraImageDegrees) {
            return;
        }
        mCameraImageDegrees = degrees;
        buildMountRotation(degrees);
        Log.i(TAG, "相机图像旋转角: " + degrees);
    }

    // 反向传感器方向，用于前后摄像头光学朝向相反的场景
    public synchronized void setSensorReverse(boolean reversed) {
        if (mSensorReverse != reversed) {
            mSensorReverse = reversed;
            Log.i(TAG, "传感器方向反向: " + (reversed ? "开启" : "关闭"));
        }
    }

    // 融合一帧位姿：rawPose 为 48 个 float 的 Model/View/Proj，返回是否绘制 AR 物体
    public synchronized boolean fuse(boolean rawDraw, int trackingState, int activeMapId,
                                     float[] rawPose, float[] sensorRotation, boolean sensorReady,
                                     float[] outModel, float[] outView, float[] outProjection) {
        if (rawPose == null || rawPose.length < 48
                || outModel == null || outView == null || outProjection == null) {
            return false;
        }

        if (!mEnabled) {
            if (!rawDraw) {
                return false;
            }
            System.arraycopy(rawPose, 0, outModel, 0, 16);
            System.arraycopy(rawPose, VIEW_OFFSET, outView, 0, 16);
            System.arraycopy(rawPose, PROJ_OFFSET, outProjection, 0, 16);
            return true;
        }

        // 视觉是否有效只看跟踪状态，锚点缺失不应被当作仍在丢失
        final boolean visionTrackingOk = (trackingState == 2);
        final boolean anchorOnActiveMap = (mAnchorMapId >= 0 && activeMapId == mAnchorMapId);

        // 地图切换后原锚点失效，丢弃保持态
        if (activeMapId != mLastMapId) {
            if (mLastMapId >= 0 && activeMapId != mAnchorMapId) {
                Log.i(TAG, "退出旧地图 " + mLastMapId + " 的保持状态 (当前 " + activeMapId
                        + "，锚点 " + mAnchorMapId + ")");
                resetHoldState();
            }
            mLastMapId = activeMapId;
        }

        if (visionTrackingOk) {
            // Model 仅在视觉实际绘制时缓存，未绘制帧为占位矩阵
            if (rawDraw) {
                System.arraycopy(rawPose, 0, mLastValidModel, 0, 16);
            }
            System.arraycopy(rawPose, VIEW_OFFSET, mLastValidView, 0, 16);
            System.arraycopy(rawPose, PROJ_OFFSET, mLastValidProj, 0, 16);
            mLastValidTranslation[0] = rawPose[VIEW_OFFSET + 12];
            mLastValidTranslation[1] = rawPose[VIEW_OFFSET + 13];
            mLastValidTranslation[2] = rawPose[VIEW_OFFSET + 14];

            System.arraycopy(rawPose, VIEW_OFFSET, mTempPureRot, 0, 16);
            mTempPureRot[12] = 0.0f;
            mTempPureRot[13] = 0.0f;
            mTempPureRot[14] = 0.0f;
            mTempPureRot[15] = 1.0f;

            final boolean hasSensor = sensorReady && sensorRotation != null;
            if (hasSensor) {
                buildSensorView(sensorRotation);
                Matrix.transposeM(mTempSensorTrans, 0, mTempSensorView, 0);
                Matrix.multiplyMM(mTempOffsetInst, 0, mTempSensorTrans, 0, mTempPureRot, 0);
                System.arraycopy(mTempOffsetInst, 0, mAlignmentOffset, 0, 16);
                mHasAlignment = true;
            }
            mHasValidState = hasSensor;
            mHasLastPose = true;
            mLastVisionOkNanos = System.nanoTime();
            mHoldUnavailableLogged = false;

            if (rawDraw && mAnchorMapId != activeMapId) {
                Log.i(TAG, "锚点绑定到地图 " + activeMapId);
                mAnchorMapId = activeMapId;
            }

            System.arraycopy(rawPose, 0, outModel, 0, 16);
            System.arraycopy(rawPose, VIEW_OFFSET, outView, 0, 16);
            System.arraycopy(rawPose, PROJ_OFFSET, outProjection, 0, 16);
            return rawDraw;
        }

        // 视觉丢失：视图旋转跟随传感器，且仅在锚点所属地图内保持
        if (anchorOnActiveMap && mHasValidState && mHasAlignment && sensorReady && sensorRotation != null) {
            System.arraycopy(mLastValidModel, 0, outModel, 0, 16);
            System.arraycopy(mLastValidProj, 0, outProjection, 0, 16);

            buildSensorView(sensorRotation);
            Matrix.multiplyMM(mTempFusedRot, 0, mTempSensorView, 0, mAlignmentOffset, 0);
            if (mSensorReverse) {
                transposeRot3(mTempFusedRot);
            }
            System.arraycopy(mTempFusedRot, 0, outView, 0, 16);
            // 平移随融合旋转一同旋转，保证相机原地转动而非绕地图原点公转
            final float tx = mLastValidTranslation[0];
            final float ty = mLastValidTranslation[1];
            final float tz = mLastValidTranslation[2];
            outView[12] = mTempFusedRot[0] * tx + mTempFusedRot[4] * ty + mTempFusedRot[8] * tz;
            outView[13] = mTempFusedRot[1] * tx + mTempFusedRot[5] * ty + mTempFusedRot[9] * tz;
            outView[14] = mTempFusedRot[2] * tx + mTempFusedRot[6] * ty + mTempFusedRot[10] * tz;
            outView[15] = 1.0f;

            // 保持态诊断：每 30 帧输出安装角、冻结视图与合成视图的上方向，便于定位方向异常
            if ((++mHoldLogCounter % 30) == 0) {
                Log.d(TAG, "保持态: 安装角=" + mCameraImageDegrees
                        + " 反向=" + mSensorReverse
                        + " 冻结up=(" + String.format("%.2f", mLastValidView[4]) + ","
                        + String.format("%.2f", mLastValidView[5]) + ")"
                        + " 合成up=(" + String.format("%.2f", outView[4]) + ","
                        + String.format("%.2f", outView[5]) + ")"
                        + " 平移=(" + String.format("%.2f", tx) + ","
                        + String.format("%.2f", ty) + "," + String.format("%.2f", tz) + ")"
                        + " 地图=" + activeMapId);
            }
            mHoldUnavailableLogged = false;
            return true;
        }

        // 冻结位姿兜底：仅在锚点所属地图内、且处于宽限时间内续画
        if (anchorOnActiveMap && mHasLastPose
                && (System.nanoTime() - mLastVisionOkNanos) <= LOST_GRACE_NANOS) {
            System.arraycopy(mLastValidModel, 0, outModel, 0, 16);
            System.arraycopy(mLastValidProj, 0, outProjection, 0, 16);
            System.arraycopy(mLastValidView, 0, outView, 0, 16);
            if (!mHoldUnavailableLogged) {
                mHoldUnavailableLogged = true;
                Log.w(TAG, "保持态不可用，短时使用冻结姿态: 基准=" + mHasValidState
                        + " 对齐=" + mHasAlignment + " 传感器=" + sensorReady);
            }
            return true;
        }

        if (mHasLastPose && !mHoldUnavailableLogged) {
            mHoldUnavailableLogged = true;
            Log.w(TAG, "保持态失效，隐藏 AR 物体: 跟踪=" + trackingState + " 绘制=" + rawDraw
                    + " 地图=" + activeMapId + " 锚点地图=" + mAnchorMapId);
        }
        return false;
    }

    private void resetHoldState() {
        mHasAlignment = false;
        mHasValidState = false;
        mHasLastPose = false;
        mHoldUnavailableLogged = false;
    }

    // 传感器姿态换算到相机坐标系：安装角乘 R_sensor 或其转置
    private void buildSensorView(float[] sensorRotation) {
        Matrix.transposeM(mTempSensorTrans, 0, sensorRotation, 0);
        Matrix.multiplyMM(mTempSensorView, 0, mMountRot, 0, mTempSensorTrans, 0);
    }

    // 转置 4x4 矩阵的 3x3 旋转块，等价于取旋转逆
    private static void transposeRot3(float[] m) {
        float t = m[1];
        m[1] = m[4];
        m[4] = t;
        t = m[2];
        m[2] = m[8];
        m[8] = t;
        t = m[6];
        m[6] = m[9];
        m[9] = t;
    }

    // 绕光轴构造安装角，列主序：设备 y/z 轴与相机 y/z 轴反向，合成后为 Rz(degrees)
    private void buildMountRotation(int degrees) {
        final double rad = Math.toRadians(degrees);
        final float c = (float) Math.cos(rad);
        final float s = (float) Math.sin(rad);
        Matrix.setIdentityM(mMountRot, 0);
        mMountRot[0] = c;
        mMountRot[1] = s;
        mMountRot[4] = -s;
        mMountRot[5] = c;
        mHasAlignment = false;
        mHasValidState = false;
    }
}
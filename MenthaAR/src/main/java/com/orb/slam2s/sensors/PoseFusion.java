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

// 位姿融合器：视觉正常时透传 SLAM 位姿，跟踪丢失时由传感器姿态延续视图旋转。
// 对外只暴露开关、相机安装角与方向反向三项配置，结果统一由 fuse 输出。
public final class PoseFusion {
    private static final String TAG = "PoseFusion";

    private static final int VIEW_OFFSET = 16;
    private static final int PROJ_OFFSET = 32;

    // 视觉失效后沿用冻结位姿的宽限时间，仅用于平滑切换
    private static final long LOST_GRACE_NANOS = 250_000_000L;

    // 切入保持态与视觉恢复时的过渡时长：抑制位姿瞬时跳变
    private static final long ENTER_BLEND_NANOS = 200_000_000L;
    private static final long RECOVER_BLEND_NANOS = 300_000_000L;

    // 只在设备近似静止时刷新对齐基准：传感器与图像存在数十毫秒时间差，
    // 快速转动时刷新会把 ω·Δt 的偏差写进基准；基准本身是常量(仅随陀螺漂移缓变)，沿用旧值没有问题
    private static final float STABLE_RATE_RAD_PER_SEC = 0.8f; // 约 45°/s
    private static final long MAX_ALIGN_STALE_NANOS = 1_500_000_000L;

    private boolean mEnabled = false;
    private boolean mSensorReverse = true; // 默认反向，与 App 默认开关保持一致
    private int mCameraImageDegrees = -1;

    private final float[] mMountRot = new float[16];

    // 对齐基准：视图 = M(t) * offset，M(t) = 安装角 * R_sensor(t)^T；
    // 同时记录取基准时刻的 M0 与视觉旋转 R0，反向时只反转增量 Δ = M(t)*M0^T
    private final float[] mAlignmentOffset = new float[16];
    private final float[] mAnchorSensorView = new float[16];
    private final float[] mAnchorVisionRot = new float[16];
    private boolean mHasAlignment = false;
    private long mAlignNanos = 0L;

    // 相邻视觉帧间的传感器角速度估计
    private final float[] mPrevSensorRot = new float[16];
    private long mPrevSensorNanos = 0L;
    private boolean mHasPrevSensor = false;
    private float mSensorRate = 0f;

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

    // 过渡状态：源位姿(四元数 + 相机中心)与起始时刻
    private boolean mBlendActive = false;
    private long mBlendStartNanos = 0L;
    private long mBlendDurationNanos = 0L;
    private final float[] mBlendFromQ = new float[4];
    private final float[] mBlendFromC = new float[3];
    private boolean mHolding = false;       // 上一帧由传感器保持态输出
    private boolean mPrevNonVision = false; // 上一帧由非视觉分支输出且已绘制
    private final float[] mLastOutView = new float[16];
    private boolean mHasLastOut = false;

    // 矩阵运算复用缓存，每帧不产生临时对象
    private final float[] mTempSensorTrans = new float[16];
    private final float[] mTempSensorView = new float[16];
    private final float[] mTempOffsetInst = new float[16];
    private final float[] mTempFusedRot = new float[16];
    private final float[] mTempPureRot = new float[16];
    private final float[] mTempA = new float[16];
    private final float[] mTempB = new float[16];
    private final float[] mTq = new float[4];
    private final float[] mTc = new float[3];
    private final float[] mBq = new float[4];
    private final float[] mBc = new float[3];

    public PoseFusion() {
        buildMountRotation(0);
        Matrix.setIdentityM(mAlignmentOffset, 0);
        Matrix.setIdentityM(mAnchorSensorView, 0);
        Matrix.setIdentityM(mAnchorVisionRot, 0);
        Matrix.setIdentityM(mLastValidModel, 0);
        Matrix.setIdentityM(mLastValidProj, 0);
        Matrix.setIdentityM(mLastOutView, 0);
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
        mHasPrevSensor = false;
        mBlendActive = false;
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

    // 把设备坐标系下的向量换算到 OpenGL 相机坐标系(RUB)：与视图融合使用同一安装角
    public synchronized void deviceToCamera(float[] dev, float[] outCam) {
        outCam[0] = mMountRot[0] * dev[0] + mMountRot[4] * dev[1] + mMountRot[8] * dev[2];
        outCam[1] = mMountRot[1] * dev[0] + mMountRot[5] * dev[1] + mMountRot[9] * dev[2];
        outCam[2] = mMountRot[2] * dev[0] + mMountRot[6] * dev[1] + mMountRot[10] * dev[2];
    }

    // 融合一帧位姿：rawPose 为 48 个 float 的 Model/View/Proj，返回是否绘制 AR 物体
    public synchronized boolean fuse(boolean rawDraw, int trackingState, int activeMapId,
                                     float[] rawPose, float[] sensorRotation, boolean sensorReady,
                                     float[] outModel, float[] outView, float[] outProjection) {
        final boolean draw = fuseInternal(rawDraw, trackingState, activeMapId, rawPose,
                sensorRotation, sensorReady, outModel, outView, outProjection);
        if (draw && mEnabled) {
            System.arraycopy(outView, 0, mLastOutView, 0, 16);
            mHasLastOut = true;
        }
        return draw;
    }

    private boolean fuseInternal(boolean rawDraw, int trackingState, int activeMapId,
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
        final long nowNanos = System.nanoTime();

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

            // 从保持态恢复：以上一帧输出为起点平滑过渡到视觉位姿
            if (mPrevNonVision && mHasLastOut && rawDraw) {
                viewToQC(mLastOutView, mBlendFromQ, mBlendFromC);
                mBlendActive = true;
                mBlendStartNanos = nowNanos;
                mBlendDurationNanos = RECOVER_BLEND_NANOS;
            }
            mPrevNonVision = false;
            mHolding = false;

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
                updateSensorRate(sensorRotation, nowNanos);
                final boolean stable = mHasPrevSensor && mSensorRate <= STABLE_RATE_RAD_PER_SEC;
                final boolean stale = (nowNanos - mAlignNanos) > MAX_ALIGN_STALE_NANOS;
                if (!mHasAlignment || stable || stale) {
                    Matrix.transposeM(mTempSensorTrans, 0, mTempSensorView, 0);
                    Matrix.multiplyMM(mTempOffsetInst, 0, mTempSensorTrans, 0, mTempPureRot, 0);
                    System.arraycopy(mTempOffsetInst, 0, mAlignmentOffset, 0, 16);
                    System.arraycopy(mTempSensorView, 0, mAnchorSensorView, 0, 16);
                    System.arraycopy(mTempPureRot, 0, mAnchorVisionRot, 0, 16);
                    mHasAlignment = true;
                    mAlignNanos = nowNanos;
                }
            } else {
                mHasPrevSensor = false;
            }
            mHasValidState = hasSensor;
            mHasLastPose = true;
            mLastVisionOkNanos = nowNanos;
            mHoldUnavailableLogged = false;

            if (rawDraw && mAnchorMapId != activeMapId) {
                Log.i(TAG, "锚点绑定到地图 " + activeMapId);
                mAnchorMapId = activeMapId;
            }

            System.arraycopy(rawPose, 0, outModel, 0, 16);
            System.arraycopy(rawPose, PROJ_OFFSET, outProjection, 0, 16);
            System.arraycopy(rawPose, VIEW_OFFSET, outView, 0, 16);
            if (mBlendActive) {
                viewToQC(outView, mTq, mTc);
                applyBlend(mTq, mTc, nowNanos, outView);
            }
            return rawDraw;
        }

        mHasPrevSensor = false;

        // 视觉丢失：视图旋转跟随传感器，且仅在锚点所属地图内保持
        if (anchorOnActiveMap && mHasValidState && mHasAlignment && sensorReady && sensorRotation != null) {
            System.arraycopy(mLastValidModel, 0, outModel, 0, 16);
            System.arraycopy(mLastValidProj, 0, outProjection, 0, 16);

            buildSensorView(sensorRotation);
            if (mSensorReverse) {
                // 只反转增量 Δ = M(t)*M0^T：fused = Δ^T * R0 = M0 * M(t)^T * R0，
                // 切换瞬间 Δ=I，输出严格等于取基准时刻的视觉旋转 R0
                Matrix.transposeM(mTempA, 0, mTempSensorView, 0);
                Matrix.multiplyMM(mTempB, 0, mAnchorSensorView, 0, mTempA, 0);
                Matrix.multiplyMM(mTempFusedRot, 0, mTempB, 0, mAnchorVisionRot, 0);
            } else {
                Matrix.multiplyMM(mTempFusedRot, 0, mTempSensorView, 0, mAlignmentOffset, 0);
            }

            // 相机中心固定为最后一帧视觉的中心 C = -R0^T * t，随后 t' = -R' * C
            final float tx = mLastValidTranslation[0];
            final float ty = mLastValidTranslation[1];
            final float tz = mLastValidTranslation[2];
            final float[] v0 = mLastValidView;
            mTc[0] = -(v0[0] * tx + v0[1] * ty + v0[2] * tz);
            mTc[1] = -(v0[4] * tx + v0[5] * ty + v0[6] * tz);
            mTc[2] = -(v0[8] * tx + v0[9] * ty + v0[10] * tz);

            if (!mHolding) {
                // 刚切入保持态：从最后一帧视觉位姿平滑过渡到传感器位姿
                viewToQC(mLastValidView, mBlendFromQ, mBlendFromC);
                mBlendActive = true;
                mBlendStartNanos = nowNanos;
                mBlendDurationNanos = ENTER_BLEND_NANOS;
                mHolding = true;
            }
            mPrevNonVision = true;

            matToQuat(mTempFusedRot, mTq);
            applyBlend(mTq, mTc, nowNanos, outView);
            mHoldUnavailableLogged = false;
            return true;
        }

        // 冻结位姿兜底：仅在锚点所属地图内、且处于宽限时间内续画
        if (anchorOnActiveMap && mHasLastPose
                && (nowNanos - mLastVisionOkNanos) <= LOST_GRACE_NANOS) {
            System.arraycopy(mLastValidModel, 0, outModel, 0, 16);
            System.arraycopy(mLastValidProj, 0, outProjection, 0, 16);
            System.arraycopy(mLastValidView, 0, outView, 0, 16);
            mPrevNonVision = true;
            if (!mHoldUnavailableLogged) {
                mHoldUnavailableLogged = true;
                Log.w(TAG, "保持态不可用，短时使用冻结姿态: 基准=" + mHasValidState
                        + " 对齐=" + mHasAlignment + " 传感器=" + sensorReady);
            }
            return true;
        }

        mPrevNonVision = false;
        mHolding = false;
        mBlendActive = false;
        if (mHasLastPose && !mHoldUnavailableLogged) {
            mHoldUnavailableLogged = true;
            Log.w(TAG, "保持态失效，隐藏 AR 物体: 跟踪=" + trackingState + " 绘制=" + rawDraw
                    + " 地图=" + activeMapId + " 锚点地图=" + mAnchorMapId);
        }
        return false;
    }

    // 用相邻两帧传感器旋转估计角速度(rad/s)
    private void updateSensorRate(float[] sensorRotation, long nowNanos) {
        if (mHasPrevSensor) {
            final float[] a = mPrevSensorRot;
            final float[] b = sensorRotation;
            // trace(Rprev^T * Rcur) = 对应列点积之和
            final float tr = a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
                    + a[4] * b[4] + a[5] * b[5] + a[6] * b[6]
                    + a[8] * b[8] + a[9] * b[9] + a[10] * b[10];
            final float cos = Math.max(-1f, Math.min(1f, (tr - 1f) * 0.5f));
            final float angle = (float) Math.acos(cos);
            final float dt = (nowNanos - mPrevSensorNanos) * 1.0e-9f;
            mSensorRate = dt > 1e-4f ? angle / dt : 0f;
        }
        System.arraycopy(sensorRotation, 0, mPrevSensorRot, 0, 16);
        mPrevSensorNanos = nowNanos;
        mHasPrevSensor = true;
    }

    // 过渡：旋转球面插值、相机中心线性插值，平移由混合后的旋转与中心统一算出，保证刚体一致
    private void applyBlend(float[] targetQ, float[] targetC, long nowNanos, float[] outView) {
        if (mBlendActive) {
            float t = (float) (nowNanos - mBlendStartNanos) / (float) mBlendDurationNanos;
            if (t >= 1f) {
                mBlendActive = false;
            } else {
                t = t * t * (3f - 2f * t);
                slerp(mBlendFromQ, targetQ, t, mBq);
                for (int i = 0; i < 3; i++) {
                    mBc[i] = mBlendFromC[i] + (targetC[i] - mBlendFromC[i]) * t;
                }
                qcToView(mBq, mBc, outView);
                return;
            }
        }
        qcToView(targetQ, targetC, outView);
    }

    // 视图矩阵(列主序) -> 单位四元数(w,x,y,z) 与相机中心 C = -R^T * t
    private static void viewToQC(float[] v, float[] q, float[] c) {
        matToQuat(v, q);
        c[0] = -(v[0] * v[12] + v[1] * v[13] + v[2] * v[14]);
        c[1] = -(v[4] * v[12] + v[5] * v[13] + v[6] * v[14]);
        c[2] = -(v[8] * v[12] + v[9] * v[13] + v[10] * v[14]);
    }

    // 四元数 + 相机中心 -> 视图矩阵：t = -R * C
    private static void qcToView(float[] q, float[] c, float[] out) {
        quatToMat(q, out);
        out[12] = -(out[0] * c[0] + out[4] * c[1] + out[8] * c[2]);
        out[13] = -(out[1] * c[0] + out[5] * c[1] + out[9] * c[2]);
        out[14] = -(out[2] * c[0] + out[6] * c[1] + out[10] * c[2]);
        out[3] = 0f;
        out[7] = 0f;
        out[11] = 0f;
        out[15] = 1f;
    }

    // 列主序 3x3 旋转 -> 四元数(w,x,y,z)
    private static void matToQuat(float[] m, float[] q) {
        final float m00 = m[0], m10 = m[1], m20 = m[2];
        final float m01 = m[4], m11 = m[5], m21 = m[6];
        final float m02 = m[8], m12 = m[9], m22 = m[10];
        final float tr = m00 + m11 + m22;
        float w, x, y, z;
        if (tr > 0f) {
            final float s = (float) Math.sqrt(tr + 1f) * 2f;
            w = 0.25f * s;
            x = (m21 - m12) / s;
            y = (m02 - m20) / s;
            z = (m10 - m01) / s;
        } else if (m00 > m11 && m00 > m22) {
            final float s = (float) Math.sqrt(1f + m00 - m11 - m22) * 2f;
            w = (m21 - m12) / s;
            x = 0.25f * s;
            y = (m01 + m10) / s;
            z = (m02 + m20) / s;
        } else if (m11 > m22) {
            final float s = (float) Math.sqrt(1f + m11 - m00 - m22) * 2f;
            w = (m02 - m20) / s;
            x = (m01 + m10) / s;
            y = 0.25f * s;
            z = (m12 + m21) / s;
        } else {
            final float s = (float) Math.sqrt(1f + m22 - m00 - m11) * 2f;
            w = (m10 - m01) / s;
            x = (m02 + m20) / s;
            y = (m12 + m21) / s;
            z = 0.25f * s;
        }
        final float n = (float) Math.sqrt(w * w + x * x + y * y + z * z);
        final float inv = n > 0f ? 1f / n : 1f;
        q[0] = w * inv;
        q[1] = x * inv;
        q[2] = y * inv;
        q[3] = z * inv;
    }

    // 四元数 -> 列主序 3x3 旋转(其余元素不改动)
    private static void quatToMat(float[] q, float[] m) {
        final float w = q[0], x = q[1], y = q[2], z = q[3];
        m[0] = 1f - 2f * (y * y + z * z);
        m[1] = 2f * (x * y + z * w);
        m[2] = 2f * (x * z - y * w);
        m[4] = 2f * (x * y - z * w);
        m[5] = 1f - 2f * (x * x + z * z);
        m[6] = 2f * (y * z + x * w);
        m[8] = 2f * (x * z + y * w);
        m[9] = 2f * (y * z - x * w);
        m[10] = 1f - 2f * (x * x + y * y);
    }

    private static void slerp(float[] a, float[] b, float t, float[] out) {
        float dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
        float sign = 1f;
        if (dot < 0f) {
            dot = -dot;
            sign = -1f;
        }
        final float k0, k1;
        if (dot > 0.9995f) {
            k0 = 1f - t;
            k1 = t * sign;
        } else {
            final float theta = (float) Math.acos(dot);
            final float sinT = (float) Math.sin(theta);
            k0 = (float) Math.sin((1f - t) * theta) / sinT;
            k1 = (float) Math.sin(t * theta) / sinT * sign;
        }
        final float w = k0 * a[0] + k1 * b[0];
        final float x = k0 * a[1] + k1 * b[1];
        final float y = k0 * a[2] + k1 * b[2];
        final float z = k0 * a[3] + k1 * b[3];
        final float n = (float) Math.sqrt(w * w + x * x + y * y + z * z);
        final float inv = n > 0f ? 1f / n : 1f;
        out[0] = w * inv;
        out[1] = x * inv;
        out[2] = y * inv;
        out[3] = z * inv;
    }

    private void resetHoldState() {
        mHasAlignment = false;
        mHasValidState = false;
        mHasLastPose = false;
        mHoldUnavailableLogged = false;
        mHasPrevSensor = false;
        mBlendActive = false;
        mHolding = false;
        mPrevNonVision = false;
        mHasLastOut = false;
    }

    // 传感器姿态换算到相机坐标系：安装角乘 R_sensor 或其转置
    private void buildSensorView(float[] sensorRotation) {
        Matrix.transposeM(mTempSensorTrans, 0, sensorRotation, 0);
        Matrix.multiplyMM(mTempSensorView, 0, mMountRot, 0, mTempSensorTrans, 0);
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
        mHasPrevSensor = false;
    }
}
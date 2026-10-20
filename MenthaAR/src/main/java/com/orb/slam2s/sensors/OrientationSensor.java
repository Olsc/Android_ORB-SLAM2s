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

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.opengl.Matrix;
import android.util.Log;
import android.view.Surface;

import com.orb.slam2s.constant.GlobalConstant;

// 3DOF 方向传感器管理：按优先级多级回退选择姿态来源（旋转矢量 / 游戏旋转矢量 / 加速度计+磁力计 / 陀螺仪积分）
public class OrientationSensor implements SensorEventListener {
    private static final String TAG = "OrientationSensor";

    // 姿态来源模式
    public enum Mode {
        NONE,                 // 无任何可用传感器，场景保持静止
        ROTATION_VECTOR,      // 旋转矢量（绝对姿态）
        GAME_ROTATION_VECTOR, // 游戏旋转矢量（相对姿态）
        ACCEL_MAG,            // 加速度计 + 磁力计
        GYROSCOPE             // 仅陀螺仪积分
    }

    private SensorManager sensorManager;
    private Sensor rotationSensor;
    private Sensor gameRotationSensor;
    private Sensor accelerometer;
    private Sensor magnetometer;
    private Sensor gyroscope;

    private Mode mode = Mode.NONE;

    // 预分配矩阵缓存
    private final float[] rawRotationMatrix = new float[16]; // 原始旋转矩阵
    private final float[] rotationMatrix = new float[16];    // 重映射后的旋转矩阵

    private final float[] gravity = new float[3];
    private final float[] geomagnetic = new float[3];
    private boolean hasGravity = false;
    private boolean hasGeomagnetic = false;

    // 陀螺仪积分复用缓存
    private long lastGyroTimestamp = 0L;
    private final float[] gyroDeltaVector = new float[3];
    private final float[] gyroDeltaMatrix = new float[16];
    private final float[] gyroTempMatrix = new float[16];

    // 加速度计 + 磁力计姿态解算复用缓存
    private final float[] accelMagR = new float[16];
    private final float[] accelMagI = new float[16];

    // 传感器是否已产生第一个有效读数（跨线程读取保持可见性）
    private volatile boolean ready = false;

    public OrientationSensor() {
        Matrix.setIdentityM(rawRotationMatrix, 0);
        Matrix.setIdentityM(rotationMatrix, 0);
    }

    // 检测设备可用的姿态来源并按优先级选择模式（不注册监听器）
    public Mode detectMode(Context context) {
        if (context == null) {
            mode = Mode.NONE;
            return mode;
        }

        SensorManager sm = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);
        if (sm == null) {
            mode = Mode.NONE;
            return mode;
        }

        rotationSensor = sm.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
        gameRotationSensor = sm.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR);
        accelerometer = sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        magnetometer = sm.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        gyroscope = sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE);

        if (rotationSensor != null) {
            mode = Mode.ROTATION_VECTOR;
        } else if (gameRotationSensor != null) {
            mode = Mode.GAME_ROTATION_VECTOR;
        } else if (accelerometer != null && magnetometer != null) {
            mode = Mode.ACCEL_MAG;
        } else if (gyroscope != null) {
            mode = Mode.GYROSCOPE;
        } else {
            mode = Mode.NONE;
        }
        return mode;
    }

    // 检测设备是否存在可用于 3DOF 的姿态传感器硬件
    public boolean hasSensorHardware(Context context) {
        return detectMode(context) != Mode.NONE;
    }

    // 返回当前（或最近一次检测到的）姿态来源模式
    public Mode getMode() {
        return mode;
    }

    // 注册传感器监听器并开始获取数据
    public void start(Context context) {
        if (context == null) return;
        sensorManager = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);
        if (sensorManager == null) {
            mode = Mode.NONE;
            return;
        }

        // 每次启动重新探测，保证传感器引用有效
        detectMode(context);

        ready = false;
        hasGravity = false;
        hasGeomagnetic = false;
        lastGyroTimestamp = 0L;
        Matrix.setIdentityM(rawRotationMatrix, 0);
        remapForLandscape();

        switch (mode) {
            case ROTATION_VECTOR:
                sensorManager.registerListener(this, rotationSensor, SensorManager.SENSOR_DELAY_GAME);
                Log.d(TAG, "姿态来源：旋转矢量（绝对姿态）");
                break;
            case GAME_ROTATION_VECTOR:
                sensorManager.registerListener(this, gameRotationSensor, SensorManager.SENSOR_DELAY_GAME);
                Log.d(TAG, "姿态来源：游戏旋转矢量（相对姿态，航向会缓慢漂移）");
                break;
            case ACCEL_MAG:
                if (accelerometer != null) {
                    sensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_GAME);
                }
                if (magnetometer != null) {
                    sensorManager.registerListener(this, magnetometer, SensorManager.SENSOR_DELAY_GAME);
                }
                Log.d(TAG, "姿态来源：加速度计 + 磁力计");
                break;
            case GYROSCOPE:
                sensorManager.registerListener(this, gyroscope, SensorManager.SENSOR_DELAY_GAME);
                Log.d(TAG, "姿态来源：陀螺仪积分（姿态会随时间漂移）");
                break;
            default:
                Log.w(TAG, "无可用姿态传感器，3DOF 将保持静止");
                break;
        }
    }

    // 注销监听器并释放资源
    public void stop() {
        if (sensorManager != null) {
            sensorManager.unregisterListener(this);
        }
        lastGyroTimestamp = 0L;
        ready = false;
    }

    // 检查传感器是否已产生第一个有效读数
    public boolean isReady() {
        return ready;
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event == null || event.values == null) {
            return;
        }

        switch (event.sensor.getType()) {
            case Sensor.TYPE_ROTATION_VECTOR:
            case Sensor.TYPE_GAME_ROTATION_VECTOR:
                SensorManager.getRotationMatrixFromVector(rawRotationMatrix, event.values);
                remapForLandscape();
                markReady();
                break;

            case Sensor.TYPE_ACCELEROMETER:
                System.arraycopy(event.values, 0, gravity, 0, 3);
                hasGravity = true;
                updateRotationMatrixFromAccelMag();
                break;

            case Sensor.TYPE_MAGNETIC_FIELD:
                System.arraycopy(event.values, 0, geomagnetic, 0, 3);
                hasGeomagnetic = true;
                updateRotationMatrixFromAccelMag();
                break;

            case Sensor.TYPE_GYROSCOPE:
                integrateGyroscope(event);
                break;

            default:
                break;
        }
    }

    private void markReady() {
        if (!ready) {
            ready = true;
            Log.d(TAG, "传感器就绪（模式=" + mode + "）");
        }
    }

    // 当同时拥有重力和地磁数据时，解算旋转矩阵
    private void updateRotationMatrixFromAccelMag() {
        if (hasGravity && hasGeomagnetic) {
            if (SensorManager.getRotationMatrix(accelMagR, accelMagI, gravity, geomagnetic)) {
                System.arraycopy(accelMagR, 0, rawRotationMatrix, 0, 16);
                remapForLandscape();
                markReady();
            }
        }
    }

    // 陀螺仪角速度增量积分
    private void integrateGyroscope(SensorEvent event) {
        long now = event.timestamp;
        if (lastGyroTimestamp == 0L) {
            lastGyroTimestamp = now;
            return;
        }
        float dt = (now - lastGyroTimestamp) * 1.0e-9f;
        lastGyroTimestamp = now;
        // 过滤异常时间间隔
        if (dt <= 0f || dt > 0.5f) {
            return;
        }

        // 小角度近似：角增量矢量 = 角速度 * dt
        gyroDeltaVector[0] = event.values[0] * dt;
        gyroDeltaVector[1] = event.values[1] * dt;
        gyroDeltaVector[2] = event.values[2] * dt;
        SensorManager.getRotationMatrixFromVector(gyroDeltaMatrix, gyroDeltaVector);

        // R = R * deltaR (机体系右乘)
        Matrix.multiplyMM(gyroTempMatrix, 0, rawRotationMatrix, 0, gyroDeltaMatrix, 0);
        System.arraycopy(gyroTempMatrix, 0, rawRotationMatrix, 0, 16);

        // 定期 Gram-Schmidt 正交规范化，抑制数值漂移
        orthonormalize(rawRotationMatrix);
        remapForLandscape();

        markReady();
    }

    // 对 4x4 列主序旋转矩阵的 3x3 部分做 Gram-Schmidt 正交规范化
    private void orthonormalize(float[] m) {
        float c0x = m[0], c0y = m[1], c0z = m[2];
        float c1x = m[4], c1y = m[5], c1z = m[6];

        // 归一化第 0 列 (X 轴)
        float len0 = (float) Math.sqrt(c0x * c0x + c0y * c0y + c0z * c0z);
        if (len0 < 1.0e-6f) return;
        float invLen0 = 1.0f / len0;
        c0x *= invLen0; c0y *= invLen0; c0z *= invLen0;

        // 第 1 列 (Y 轴) 减去在第 0 列上的投影并归一化
        float d = c1x * c0x + c1y * c0y + c1z * c0z;
        c1x -= d * c0x;
        c1y -= d * c0y;
        c1z -= d * c0z;
        float len1 = (float) Math.sqrt(c1x * c1x + c1y * c1y + c1z * c1z);
        if (len1 < 1.0e-6f) return;
        float invLen1 = 1.0f / len1;
        c1x *= invLen1; c1y *= invLen1; c1z *= invLen1;

        // 第 2 列 (Z 轴) = 第 0 列 x 第 1 列 (叉乘保证右手正交系)
        float c2x = c0y * c1z - c0z * c1y;
        float c2y = c0z * c1x - c0x * c1z;
        float c2z = c0x * c1y - c0y * c1x;

        m[0] = c0x; m[1] = c0y; m[2] = c0z;
        m[4] = c1x; m[5] = c1y; m[6] = c1z;
        m[8] = c2x; m[9] = c2y; m[10] = c2z;
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // 无需处理精度变化
    }

    // 获取横屏重映射后的旋转矩阵
    public float[] getRotationMatrix() {
        return rotationMatrix;
    }

    // 将传感器坐标系重映射为当前屏幕坐标系
    private void remapForLandscape() {
        int displayRotation = GlobalConstant.DISPLAY_ROTATION;
        switch (displayRotation) {
            case Surface.ROTATION_90:
            case 90:
                // 左横屏：新 X 轴 = 旧 Y 轴，新 Y 轴 = 旧 -X 轴
                SensorManager.remapCoordinateSystem(
                    rawRotationMatrix,
                    SensorManager.AXIS_Y,
                    SensorManager.AXIS_MINUS_X,
                    rotationMatrix
                );
                break;
            case Surface.ROTATION_180:
            case 180:
                // 反向竖屏：新 X 轴 = 旧 -X 轴，新 Y 轴 = 旧 -Y 轴
                SensorManager.remapCoordinateSystem(
                    rawRotationMatrix,
                    SensorManager.AXIS_MINUS_X,
                    SensorManager.AXIS_MINUS_Y,
                    rotationMatrix
                );
                break;
            case Surface.ROTATION_270:
            case 270:
                // 右横屏：新 X 轴 = 旧 -Y 轴，新 Y 轴 = 旧 X 轴
                SensorManager.remapCoordinateSystem(
                    rawRotationMatrix,
                    SensorManager.AXIS_MINUS_Y,
                    SensorManager.AXIS_X,
                    rotationMatrix
                );
                break;
            default:
                // ROTATION_0：保持传感器原始坐标系
                System.arraycopy(rawRotationMatrix, 0, rotationMatrix, 0, 16);
                break;
        }
    }
}
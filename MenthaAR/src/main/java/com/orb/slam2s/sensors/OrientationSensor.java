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

import java.util.List;

// 3DOF 方向传感器管理：按优先级多级回退选择姿态来源（旋转矢量 / 游戏旋转矢量 / 加速度计+磁力计 / 陀螺仪积分 / 仅重力矢量）
public class OrientationSensor implements SensorEventListener {
    private static final String TAG = "OrientationSensor";

    // 姿态来源模式
    public enum Mode {
        NONE,                 // 无任何可用传感器，场景保持静止
        ROTATION_VECTOR,      // 旋转矢量（绝对姿态）
        GAME_ROTATION_VECTOR, // 游戏旋转矢量（相对姿态）
        ACCEL_MAG,            // 加速度计 + 磁力计
        GYROSCOPE,            // 仅陀螺仪积分
        ACCEL_ONLY            // 仅重力矢量（俯仰与滚转，无航向）
    }

    private SensorManager sensorManager;
    private Sensor rotationSensor;
    private Sensor gameRotationSensor;
    private Sensor accelerometer;
    private Sensor magnetometer;
    private Sensor gyroscope;
    private Sensor gravitySensor;

    private Mode mode = Mode.NONE;

    // 预分配矩阵缓存
    private final float[] rawRotationMatrix = new float[16]; // 原始旋转矩阵

    private final float[] gravity = new float[3];
    private final float[] geomagnetic = new float[3];
    private boolean hasGravity = false;
    private boolean hasGeomagnetic = false;

    // 仅重力模式：低通滤波后的重力矢量
    private final float[] filteredGravity = new float[3];
    private boolean hasFilteredGravity = false;

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
        gravitySensor = sm.getDefaultSensor(Sensor.TYPE_GRAVITY);

        if (rotationSensor != null) {
            mode = Mode.ROTATION_VECTOR;
        } else if (gameRotationSensor != null) {
            mode = Mode.GAME_ROTATION_VECTOR;
        } else if (accelerometer != null && magnetometer != null) {
            mode = Mode.ACCEL_MAG;
        } else if (gyroscope != null) {
            mode = Mode.GYROSCOPE;
        } else if (accelerometer != null || gravitySensor != null) {
            // 仅有重力方向，可解算俯仰与滚转，无法解算航向
            mode = Mode.ACCEL_ONLY;
        } else {
            mode = Mode.NONE;
        }
        return mode;
    }

    // 当前姿态来源可观测的自由度，3 含航向，2 仅俯仰与滚转，0 不可用
    public int getRotationDof() {
        switch (mode) {
            case ACCEL_ONLY:
                return 2;
            case NONE:
                return 0;
            default:
                return 3;
        }
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
        hasFilteredGravity = false;
        lastGyroTimestamp = 0L;
        Matrix.setIdentityM(rawRotationMatrix, 0);

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
            case ACCEL_ONLY:
                // 优先使用系统已滤好的重力传感器，缺失时对原始加速度做低通滤波
                if (gravitySensor != null) {
                    sensorManager.registerListener(this, gravitySensor, SensorManager.SENSOR_DELAY_GAME);
                } else if (accelerometer != null) {
                    sensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_GAME);
                }
                Log.d(TAG, "姿态来源：仅重力矢量（只有俯仰与滚转）");
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
                markReady();
                break;

            case Sensor.TYPE_ACCELEROMETER:
                if (mode == Mode.ACCEL_ONLY) {
                    // 单加速度计模式，先滤出重力方向再解算倾斜
                    lowPassGravity(event.values);
                    if (updateRotationMatrixFromGravity(filteredGravity)) {
                        markReady();
                    }
                    break;
                }
                System.arraycopy(event.values, 0, gravity, 0, 3);
                hasGravity = true;
                updateRotationMatrixFromAccelMag();
                break;

            case Sensor.TYPE_GRAVITY:
                // 系统已融合的重力矢量，直接解算倾斜
                if (updateRotationMatrixFromGravity(event.values)) {
                    markReady();
                }
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
                markReady();
            }
        }
    }

    // 一阶低通提取重力方向，抑制手部运动引入的线性加速度
    private void lowPassGravity(float[] a) {
        final float alpha = 0.85f;
        if (!hasFilteredGravity) {
            System.arraycopy(a, 0, filteredGravity, 0, 3);
            hasFilteredGravity = true;
            return;
        }
        for (int i = 0; i < 3; i++) {
            filteredGravity[i] = alpha * filteredGravity[i] + (1f - alpha) * a[i];
        }
    }

    // 仅用重力矢量解算姿态：求把设备坐标系天顶向量 n 旋到世界 Z 轴的最小旋转，
    // 该旋转不引入绕世界 Z 轴的扭转，等价于保留俯仰与滚转、航向恒为 0
    private boolean updateRotationMatrixFromGravity(float[] g) {
        float gx = g[0], gy = g[1], gz = g[2];
        float len = (float) Math.sqrt(gx * gx + gy * gy + gz * gz);
        if (len < 1.0f) {
            // 接近自由落体，方向不可信
            return false;
        }
        float nx = gx / len, ny = gy / len, nz = gz / len;

        Matrix.setIdentityM(rawRotationMatrix, 0);

        if (nz > 0.99999f) {
            // 近似平放，即单位阵
            return true;
        }
        if (nz < -0.99999f) {
            // 屏幕朝下，绕 X 轴翻转 180 度以避开退化
            rawRotationMatrix[5] = -1f;
            rawRotationMatrix[10] = -1f;
            return true;
        }

        // 分母趋零的情况已在上面排除
        float inv = 1f / (1f + nz);
        rawRotationMatrix[0] = nz + ny * ny * inv;
        rawRotationMatrix[1] = -nx * ny * inv;
        rawRotationMatrix[2] = -nx;
        rawRotationMatrix[4] = -nx * ny * inv;
        rawRotationMatrix[5] = nz + nx * nx * inv;
        rawRotationMatrix[6] = -ny;
        rawRotationMatrix[8] = nx;
        rawRotationMatrix[9] = ny;
        rawRotationMatrix[10] = nz;
        return true;
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

    // 获取世界坐标系下的旋转矩阵（竖屏下设备系即屏幕系，无需重映射）
    public float[] getRotationMatrix() {
        return rawRotationMatrix;
    }

}
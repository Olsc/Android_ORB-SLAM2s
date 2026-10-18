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
import android.util.Log;

import com.orb.slam2s.constant.GlobalConstant;

// 3DOF 方向传感器管理：按优先级选择姿态来源（旋转矢量 / 游戏旋转矢量 / 加速度计+磁力计 / 陀螺仪积分），
// 仅维护 Android 传感器生命周期与事件监听，姿态解算与横屏重映射在原生层完成。
public class OrientationSensor implements SensorEventListener {
    private static final String TAG = "OrientationSensor";

    static {
        try {
            System.loadLibrary("MenthaAR_Engine");
        } catch (UnsatisfiedLinkError e) {
            Log.e(TAG, "加载 MenthaAR_Engine 失败: " + e.getMessage());
        }
    }

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

    // C++ 原生处理器指针
    private long nativeHandle = 0L;

    // 横屏重映射后的旋转矩阵缓存
    private final float[] rotationMatrix = new float[16];

    public OrientationSensor() {
        nativeHandle = nativeCreate();
        if (nativeHandle != 0L) {
            nativeGetRotationMatrix(nativeHandle, rotationMatrix);
        }
    }

    // 检测设备可用的姿态来源并按优先级选择模式（不注册监听器）
    public Mode detectMode(Context context) {
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

    // 返回当前（或最近一次检测到的）姿态来源模式
    public Mode getMode() {
        return mode;
    }

    // 注册传感器监听器并开始获取数据
    public void start(Context context) {
        sensorManager = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);
        if (sensorManager == null) {
            mode = Mode.NONE;
            return;
        }

        // 每次启动重新探测，保证传感器引用有效
        detectMode(context);

        if (nativeHandle != 0L) {
            nativeReset(nativeHandle);
        }

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
        if (nativeHandle != 0L) {
            nativeReset(nativeHandle);
        }
    }

    // 检查传感器是否已产生第一个有效读数
    public boolean isReady() {
        return nativeHandle != 0L && nativeIsReady(nativeHandle);
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (nativeHandle == 0L || event == null || event.values == null) {
            return;
        }

        final int displayRotation = GlobalConstant.DISPLAY_ROTATION;

        switch (event.sensor.getType()) {
            case Sensor.TYPE_ROTATION_VECTOR:
            case Sensor.TYPE_GAME_ROTATION_VECTOR:
                nativeUpdateRotationVector(nativeHandle, event.values, displayRotation);
                break;

            case Sensor.TYPE_ACCELEROMETER:
                nativeUpdateAccelerometer(nativeHandle, event.values, displayRotation);
                break;

            case Sensor.TYPE_MAGNETIC_FIELD:
                nativeUpdateMagnetometer(nativeHandle, event.values, displayRotation);
                break;

            case Sensor.TYPE_GYROSCOPE:
                nativeIntegrateGyroscope(nativeHandle, event.values, event.timestamp, displayRotation);
                break;

            default:
                break;
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // 无需处理精度变化
    }

    // 获取横屏重映射后的旋转矩阵
    public float[] getRotationMatrix() {
        if (nativeHandle != 0L) {
            nativeGetRotationMatrix(nativeHandle, rotationMatrix);
        }
        return rotationMatrix;
    }

    // 检查设备是否具备运行 3DOF 所需的传感器
    public boolean hasRequiredSensors(Context context) {
        return detectMode(context) != Mode.NONE;
    }

    // 销毁并释放原生资源
    public synchronized void destroy() {
        if (nativeHandle != 0L) {
            nativeDestroy(nativeHandle);
            nativeHandle = 0L;
        }
    }

    @Override
    protected void finalize() throws Throwable {
        try {
            destroy();
        } finally {
            super.finalize();
        }
    }

    // 本地 JNI 声明
    private native long nativeCreate();
    private native void nativeDestroy(long handle);
    private native void nativeReset(long handle);
    private native void nativeUpdateRotationVector(long handle, float[] values, int displayRotation);
    private native void nativeUpdateAccelerometer(long handle, float[] values, int displayRotation);
    private native void nativeUpdateMagnetometer(long handle, float[] values, int displayRotation);
    private native void nativeIntegrateGyroscope(long handle, float[] values, long timestampNs, int displayRotation);
    private native void nativeRemapForDisplayRotation(long handle, int displayRotation);
    private native boolean nativeIsReady(long handle);
    private native void nativeGetRotationMatrix(long handle, float[] outMatrix);
}
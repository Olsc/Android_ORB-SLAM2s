package com.orb.slam2s.sensors;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.opengl.Matrix;
import android.util.Log;

import com.orb.slam2s.constant.GlobalConstant;

// 3DOF 方向传感器管理：按优先级多级回退选择姿态来源（旋转矢量 / 游戏旋转矢量 / 加速度计+磁力计 / 陀螺仪积分），
// 并将姿态重映射到固定横屏坐标系，兼容缺少磁力计或旋转矢量的设备。
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

    private final float[] rawRotationMatrix = new float[16]; // 原始旋转矩阵
    private final float[] rotationMatrix = new float[16];    // 重映射后的旋转矩阵

    private final float[] gravity = new float[3];
    private final float[] geomagnetic = new float[3];
    private boolean hasGravity = false;
    private boolean hasGeomagnetic = false;

    // 陀螺仪积分相关（成员复用，避免高频回调反复分配）
    private long lastGyroTimestamp = 0L;
    private final float[] gyroDeltaVector = new float[3];
    private final float[] gyroDeltaMatrix = new float[16];
    private final float[] gyroTempMatrix = new float[16];

    // 加速度计 + 磁力计姿态解算的复用矩阵，避免传感器事件高频路径上的重复内存分配
    private final float[] accelMagR = new float[16];
    private final float[] accelMagI = new float[16];

    // 传感器是否已产生第一个有效读数（跨线程读取，保持可见性）
    private volatile boolean ready = false;

    public OrientationSensor() {
        Matrix.setIdentityM(rawRotationMatrix, 0);
        Matrix.setIdentityM(rotationMatrix, 0);
    }

    // 检测设备可用的姿态来源并按优先级选择模式（不注册监听器），供 UI 提前判断
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
        switch (event.sensor.getType()) {
            case Sensor.TYPE_ROTATION_VECTOR:
            case Sensor.TYPE_GAME_ROTATION_VECTOR:
                // 两者都直接映射为旋转矩阵
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

    // 当同时拥有重力和地磁数据时，手动计算旋转矩阵
    private void updateRotationMatrixFromAccelMag() {
        if (hasGravity && hasGeomagnetic) {
            if (SensorManager.getRotationMatrix(accelMagR, accelMagI, gravity, geomagnetic)) {
                System.arraycopy(accelMagR, 0, rawRotationMatrix, 0, 16);
                remapForLandscape();
                markReady();
            }
        }
    }

    // 陀螺仪积分：以机体坐标系下的角速度增量为单位旋转矩阵，右乘到当前姿态上
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

        // 小角度近似：旋转矢量约等于角速度乘以 dt
        gyroDeltaVector[0] = event.values[0] * dt;
        gyroDeltaVector[1] = event.values[1] * dt;
        gyroDeltaVector[2] = event.values[2] * dt;
        SensorManager.getRotationMatrixFromVector(gyroDeltaMatrix, gyroDeltaVector);

        // R = R * deltaR（机体坐标系增量）
        Matrix.multiplyMM(gyroTempMatrix, 0, rawRotationMatrix, 0, gyroDeltaMatrix, 0);
        System.arraycopy(gyroTempMatrix, 0, rawRotationMatrix, 0, 16);

        // 定期正交化，抑制数值漂移导致的形变
        orthonormalize(rawRotationMatrix);
        remapForLandscape();

        markReady();
    }

    // 对旋转矩阵的 3x3 部分做 Gram-Schmidt 正交化，避免积分误差累积导致形变
    private void orthonormalize(float[] m) {
        float c0x = m[0], c0y = m[1], c0z = m[2];
        float c1x = m[4], c1y = m[5], c1z = m[6];

        // 归一化第一列
        float len0 = (float) Math.sqrt(c0x * c0x + c0y * c0y + c0z * c0z);
        if (len0 < 1.0e-6f) {
            return;
        }
        c0x /= len0; c0y /= len0; c0z /= len0;

        // 第二列减去在第一列上的投影并归一化
        float d = c1x * c0x + c1y * c0y + c1z * c0z;
        c1x -= d * c0x;
        c1y -= d * c0y;
        c1z -= d * c0z;
        float len1 = (float) Math.sqrt(c1x * c1x + c1y * c1y + c1z * c1z);
        if (len1 < 1.0e-6f) {
            return;
        }
        c1x /= len1; c1y /= len1; c1z /= len1;

        // 第三列 = 第一列 * 第二列
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

    // 获取已针对横屏重映射的旋转矩阵
    public float[] getRotationMatrix() {
        return rotationMatrix;
    }

    // 检查设备是否具备运行 3DOF 所需的传感器
    // 只要满足以下任意一种即可：旋转矢量 / 游戏旋转矢量 / (加速度计 + 磁力计) / 陀螺仪
    public boolean hasRequiredSensors(Context context) {
        return detectMode(context) != Mode.NONE;
    }

    // 将传感器坐标系重映射为横屏坐标系，左横屏(ROTATION_90)与右横屏(ROTATION_270)使用不同轴映射
    private void remapForLandscape() {
        // 获取当前显示旋转
        int displayRotation = android.view.Surface.ROTATION_90; // 默认左横屏
        try {
            // 通过静态方式获取当前旋转
            displayRotation = GlobalConstant.DISPLAY_ROTATION;
        } catch (Exception e) {
            // 默认左横屏
        }

        if (displayRotation == android.view.Surface.ROTATION_270) {
            // 右横屏 (reverse landscape)
            SensorManager.remapCoordinateSystem(
                rawRotationMatrix,
                SensorManager.AXIS_MINUS_Y,  // 新X轴 = 旧-Y轴
                SensorManager.AXIS_X,        // 新Y轴 = 旧X轴
                rotationMatrix
            );
        } else {
            // 左横屏 (landscape, 默认)
            SensorManager.remapCoordinateSystem(
                rawRotationMatrix,
                SensorManager.AXIS_Y,        // 新X轴 = 旧Y轴
                SensorManager.AXIS_MINUS_X,  // 新Y轴 = 旧-X轴
                rotationMatrix
            );
        }
    }
}
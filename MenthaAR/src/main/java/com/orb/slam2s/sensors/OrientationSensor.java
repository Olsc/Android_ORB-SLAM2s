package com.orb.slam2s.sensors;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.opengl.Matrix;
import android.util.Log;
import android.view.Surface;

/**
 * 3DOF方向传感器管理类
 * 负责处理设备传感器数据以计算设备的3D姿态
 * 优先使用旋转矢量传感器，不可用时回退到加速度计+磁力计方案
 * 
 * 针对固定横屏应用进行了坐标系重映射
 */
public class OrientationSensor implements SensorEventListener {
    private static final String TAG = "OrientationSensor";
    
    private SensorManager sensorManager;
    private Sensor rotationSensor;
    private Sensor accelerometer;
    private Sensor magnetometer;

    private final float[] rawRotationMatrix = new float[16];      // 原始旋转矩阵
    private final float[] rotationMatrix = new float[16];         // 重映射后的旋转矩阵

    private final float[] gravity = new float[3];
    private final float[] geomagnetic = new float[3];
    private boolean hasGravity = false;
    private boolean hasGeomagnetic = false;

    private boolean ready = false;

    public OrientationSensor() {
        Matrix.setIdentityM(rawRotationMatrix, 0);
        Matrix.setIdentityM(rotationMatrix, 0);
    }

    /**
     * 注册传感器监听器并开始获取数据
     */
    public void start(Context context) {
        sensorManager = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);

        rotationSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);

        if (rotationSensor != null) {
            sensorManager.registerListener(this, rotationSensor, SensorManager.SENSOR_DELAY_GAME);
            Log.d(TAG, "使用旋转矢量传感器");
        } else {
            accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
            magnetometer = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);

            if (accelerometer != null) {
                sensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_GAME);
            }
            if (magnetometer != null) {
                sensorManager.registerListener(this, magnetometer, SensorManager.SENSOR_DELAY_GAME);
            }
            Log.d(TAG, "使用加速度计+磁力计");
        }
    }

    /**
     * 注销监听器并释放资源
     */
    public void stop() {
        if (sensorManager != null) {
            sensorManager.unregisterListener(this);
        }
        ready = false;
    }

    /**
     * 检查传感器是否已经产生了第一个有效读数
     */
    public boolean isReady() {
        return ready;
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
            SensorManager.getRotationMatrixFromVector(rawRotationMatrix, event.values);
            remapForLandscape();
            if (!ready) {
                ready = true;
                Log.d(TAG, "传感器就绪（旋转矢量模式）");
            }
        } else if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            System.arraycopy(event.values, 0, gravity, 0, 3);
            hasGravity = true;
            updateRotationMatrix();
        } else if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            System.arraycopy(event.values, 0, geomagnetic, 0, 3);
            hasGeomagnetic = true;
            updateRotationMatrix();
        }
    }

    private void updateRotationMatrix() {
        if (hasGravity && hasGeomagnetic) {
            float[] R = new float[16];
            float[] I = new float[16];
            if (SensorManager.getRotationMatrix(R, I, gravity, geomagnetic)) {
                System.arraycopy(R, 0, rawRotationMatrix, 0, 16);
                remapForLandscape();
                if (!ready) {
                    ready = true;
                    Log.d(TAG, "传感器就绪（加速度+磁力计模式）");
                }
            }
        }
    }
    
    /**
     * 将传感器坐标系重映射为横屏坐标系
     *
     * 竖屏时传感器坐标系：X向右，Y向上，Z向外（屏幕朝向用户）
     * 根据当前显示旋转，选择正确的重映射方式：
     * - 左横屏 (ROTATION_90) : AXIS_Y → 新X, AXIS_MINUS_X → 新Y
     * - 右横屏 (ROTATION_270): AXIS_MINUS_Y → 新X, AXIS_X → 新Y
     */
    private void remapForLandscape() {
        // 获取当前显示旋转
        int displayRotation = android.view.Surface.ROTATION_90; // 默认左横屏
        try {
            // 通过静态方式获取当前旋转
            displayRotation = com.orb.slam2s.constant.GlobalConstant.DISPLAY_ROTATION;
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

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // 暂时无需处理精度变化
    }

    /**
     * 获取当前的旋转矩阵（已针对横屏重映射）
     */
    public float[] getRotationMatrix() {
        return rotationMatrix;
    }

    /**
     * 检查设备是否具备运行所需的传感器
     */
    public boolean hasRequiredSensors(Context context) {
        SensorManager sm = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);
        Sensor rotationSensor = sm.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
        Sensor accelerometer = sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        Sensor magnetometer = sm.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);

        return rotationSensor != null || (accelerometer != null && magnetometer != null);
    }
}

package com.orb.slam2s.constant;

/**
 * Created by ads on 17-2-20.
 * 由Olsc于2026/5/15修改：增加动态分辨率支持
 */

public class GlobalConstant {
    public static final int PLANE_DETECTED=233;

    // 参考分辨率 (基准值，用于比例计算)
    public static final int REFERENCE_WIDTH = 1280;
    public static final int REFERENCE_HEIGHT = 720;

    // 实际使用的分辨率 (运行时动态计算)
    public static int RESOLUTION_WIDTH = 1280;
    public static int RESOLUTION_HEIGHT = 720;

    // 显示旋转角度 (0=正常左横屏, 180=右横屏)
    public static int DISPLAY_ROTATION = 0;

    // 根据屏幕尺寸计算最佳相机分辨率，保持16:9比例且不超过参考分辨率
    public static void computeOptimalResolution(int screenWidth, int screenHeight) {
        // 确保宽>高 (横屏)
        if (screenWidth < screenHeight) {
            int tmp = screenWidth;
            screenWidth = screenHeight;
            screenHeight = tmp;
        }

        // 以参考分辨率为基准，按屏幕比例缩放
        // 保持16:9宽高比
        float screenRatio = (float) screenWidth / screenHeight;
        float targetRatio = (float) REFERENCE_WIDTH / REFERENCE_HEIGHT; // 16:9

        int targetW, targetH;

        if (screenRatio > targetRatio) {
            // 屏幕更宽：高度适配
            targetH = Math.min(screenHeight, REFERENCE_HEIGHT);
            targetW = (int) (targetH * targetRatio);
        } else {
            // 屏幕更高或相等：宽度适配
            targetW = Math.min(screenWidth, REFERENCE_WIDTH);
            targetH = (int) (targetW / targetRatio);
        }

        // 确保最低分辨率不低于640x360 (保证SLAM特征点数量)
        if (targetW < 640) {
            targetW = 640;
            targetH = (int) (targetW / targetRatio);
        }

        // 确保奇数尺寸对齐到偶数 (OpenCV处理需要)
        if (targetW % 2 != 0) targetW++;
        if (targetH % 2 != 0) targetH++;

        RESOLUTION_WIDTH = targetW;
        RESOLUTION_HEIGHT = targetH;
    }

    // 更新显示旋转角度，rotation 为 Android Surface 旋转角度
    public static void setDisplayRotation(int rotation) {
        // 传感器坐标系重映射在OrientationSensor中已处理
        // 这里记录屏幕旋转，用于相机帧旋转补偿
        // 左横屏(Surface.ROTATION_90) = 0度旋转
        // 右横屏(Surface.ROTATION_270) = 180度旋转
        DISPLAY_ROTATION = rotation;
    }
}
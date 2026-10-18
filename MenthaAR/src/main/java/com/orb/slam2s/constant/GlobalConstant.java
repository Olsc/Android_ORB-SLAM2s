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

package com.orb.slam2s.constant;

/**
 * Created by ads on 17-2-20.
 * 由Olsc于2026/5/15修改：增加动态分辨率支持
 */

public class GlobalConstant {
    // 参考分辨率 (基准值，用于比例计算)
    public static final int REFERENCE_WIDTH = 1280;
    public static final int REFERENCE_HEIGHT = 720;

    // 实际使用的分辨率 (运行时动态计算)
    public static int RESOLUTION_WIDTH = 1280;
    public static int RESOLUTION_HEIGHT = 720;

    // 显示旋转角度，取值为 Surface.ROTATION_0/90/180/270
    public static int DISPLAY_ROTATION = 0;

    // 根据屏幕尺寸计算最佳相机分辨率，保持 16:9 比例且不超过参考分辨率
    public static void computeOptimalResolution(int screenWidth, int screenHeight) {
        if (screenWidth < screenHeight) {
            int tmp = screenWidth;
            screenWidth = screenHeight;
            screenHeight = tmp;
        }
        float screenRatio = (float) screenWidth / screenHeight;
        float targetRatio = (float) REFERENCE_WIDTH / REFERENCE_HEIGHT;
        int targetW, targetH;
        if (screenRatio > targetRatio) {
            targetH = Math.min(screenHeight, REFERENCE_HEIGHT);
            targetW = (int) (targetH * targetRatio);
        } else {
            targetW = Math.min(screenWidth, REFERENCE_WIDTH);
            targetH = (int) (targetW / targetRatio);
        }
        if (targetW < 640) {
            targetW = 640;
            targetH = (int) (targetW / targetRatio);
        }
        if (targetW % 2 != 0) targetW++;
        if (targetH % 2 != 0) targetH++;
        RESOLUTION_WIDTH = targetW;
        RESOLUTION_HEIGHT = targetH;
    }

    // 更新显示旋转角度，rotation 取值为 Surface.ROTATION_0/90/180/270
    public static void setDisplayRotation(int rotation) {
        // 传感器坐标系重映射由 OrientationSensor 处理
        DISPLAY_ROTATION = rotation;
    }
}
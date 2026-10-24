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
    // 竖屏分辨率阶梯，均保持 9:16，CameraX 会从中选最接近的设备支持尺寸
    private static final int[][] PORTRAIT_TARGETS = {
            {720, 1280}, {540, 960}, {480, 854}, {360, 640}
    };

    // 实际使用的分辨率 (运行时动态计算，竖屏方向)
    public static int RESOLUTION_WIDTH = 720;
    public static int RESOLUTION_HEIGHT = 1280;

    // 根据屏幕尺寸选择最接近且不超过屏幕的竖屏分辨率
    public static void computeOptimalResolution(int screenWidth, int screenHeight) {
        // 统一换算到竖屏方向，宽为短边
        if (screenWidth > screenHeight) {
            int tmp = screenWidth;
            screenWidth = screenHeight;
            screenHeight = tmp;
        }
        int targetW = PORTRAIT_TARGETS[PORTRAIT_TARGETS.length - 1][0];
        int targetH = PORTRAIT_TARGETS[PORTRAIT_TARGETS.length - 1][1];
        for (int[] candidate : PORTRAIT_TARGETS) {
            if (candidate[0] <= screenWidth && candidate[1] <= screenHeight) {
                targetW = candidate[0];
                targetH = candidate[1];
                break;
            }
        }
        RESOLUTION_WIDTH = targetW;
        RESOLUTION_HEIGHT = targetH;
    }
}
/*
 * Copyright (C) 2026 Olsc <OlscStudio@outlook.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.orb.slam2s.camera;

// 相机采集分辨率选择：按屏幕尺寸挑选最接近的竖屏档位
public final class CameraResolution {
    // 竖屏分辨率阶梯，均保持 9:16，CameraX 会从中选最接近的设备支持尺寸
    private static final int[][] PORTRAIT_TARGETS = {
            {720, 1280}, {540, 960}, {480, 854}, {360, 640}
    };

    // 运行时计算出的竖屏分辨率
    public static int width = 720;
    public static int height = 1280;

    private CameraResolution() {
    }

    // 根据屏幕尺寸选择最接近且不超过屏幕的竖屏分辨率
    public static void computeOptimal(int screenWidth, int screenHeight) {
        // 统一换算到竖屏方向，宽为短边
        if (screenWidth > screenHeight) {
            final int tmp = screenWidth;
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
        width = targetW;
        height = targetH;
    }
}
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
package com.orb.slam2s.device;

import android.os.Build;

// 设备兼容性处理类：针对特定设备（如 Rokid RG-glasses）进行特殊画面处理或硬件参数调整
public final class DeviceCompat {
    private static Boolean sIsRokidGlasses = null;

    private DeviceCompat() {}

    // 当前设备是否为非 Rokid RG-glasses（所有调用方均使用取反语义，故按检查建议反转命名）
    public static boolean isNotRokidGlasses() {
        if (sIsRokidGlasses != null) {
            return !sIsRokidGlasses;
        }
        String manufacturer = Build.MANUFACTURER;
        String model = Build.MODEL;
        String product = Build.PRODUCT;
        String device = Build.DEVICE;

        boolean isManufacturerMatch = "Rokid".equalsIgnoreCase(manufacturer);
        boolean isModelMatch = "RG-glasses".equalsIgnoreCase(model) || (model != null && model.contains("RG-glasses"));
        boolean isProductMatch = "glasses".equalsIgnoreCase(product) || "glasses".equalsIgnoreCase(device);

        sIsRokidGlasses = isManufacturerMatch && (isModelMatch || isProductMatch);
        return !sIsRokidGlasses;
    }

    // Rokid RG-glasses 设备帧画面上下与左右镜像翻转处理（纯 Byte 缓冲版本，零 JNI 跨进程开销）
    public static void checkAndFlipFrame(byte[] yData, int width, int height) {
        if (yData == null || isNotRokidGlasses() || width <= 0 || height <= 0) return;

        int halfH = height / 2;
        for (int r = 0; r < halfH; r++) {
            int topOffset = r * width;
            int bottomOffset = (height - 1 - r) * width;
            for (int c = 0; c < width; c++) {
                int topIdx = topOffset + c;
                int bottomIdx = bottomOffset + (width - 1 - c);
                byte tmp = yData[topIdx];
                yData[topIdx] = yData[bottomIdx];
                yData[bottomIdx] = tmp;
            }
        }
    }

    // Rokid RG-glasses 预览位图像素数组上下与左右镜像翻转处理
    public static void checkAndFlipFrame(int[] pixels, int width, int height) {
        if (pixels == null || isNotRokidGlasses() || width <= 0 || height <= 0) return;

        int halfH = height / 2;
        for (int r = 0; r < halfH; r++) {
            int topOffset = r * width;
            int bottomOffset = (height - 1 - r) * width;
            for (int c = 0; c < width; c++) {
                int topIdx = topOffset + c;
                int bottomIdx = bottomOffset + (width - 1 - c);
                int tmp = pixels[topIdx];
                pixels[topIdx] = pixels[bottomIdx];
                pixels[bottomIdx] = tmp;
            }
        }
    }
}
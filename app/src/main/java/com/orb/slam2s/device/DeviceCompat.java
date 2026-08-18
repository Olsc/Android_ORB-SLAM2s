package com.orb.slam2s.device;

import android.os.Build;

// 设备兼容性处理类：针对特定设备（如 Rokid RG-glasses）进行特殊画面处理或硬件参数调整
public final class DeviceCompat {
    private static final String TAG = "DeviceCompat";

    private static Boolean sIsRokidGlasses = null;

    private DeviceCompat() {}

    public static boolean isRokidGlasses() {
        if (sIsRokidGlasses != null) {
            return sIsRokidGlasses;
        }
        String manufacturer = Build.MANUFACTURER;
        String model = Build.MODEL;
        String product = Build.PRODUCT;
        String device = Build.DEVICE;

        boolean isManufacturerMatch = "Rokid".equalsIgnoreCase(manufacturer);
        boolean isModelMatch = "RG-glasses".equalsIgnoreCase(model) || (model != null && model.contains("RG-glasses"));
        boolean isProductMatch = "glasses".equalsIgnoreCase(product) || "glasses".equalsIgnoreCase(device);

        sIsRokidGlasses = isManufacturerMatch && (isModelMatch || isProductMatch);
        return sIsRokidGlasses;
    }

    // Rokid RG-glasses 设备帧画面上下与左右镜像翻转处理（纯 Byte 缓冲版本，零 JNI 跨进程开销）
    public static void checkAndFlipFrame(byte[] yData, int width, int height) {
        if (yData == null || !isRokidGlasses() || width <= 0 || height <= 0) return;

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
}

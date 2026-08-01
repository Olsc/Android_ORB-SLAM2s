package com.orb.slam2s.compat;

import android.os.Build;

/**
 * 设备兼容性处理类
 * 用于针对特定设备（如 Rokid RG-glasses）进行特殊的画面处理或逻辑调整。
 */
public class DeviceCompat_RokidGlass3 {
    private static final String TAG = "DeviceCompat_RokidGlass3";

    private static Boolean sIsRokidGlasses = null;

    /**
     * 检查当前设备是否为 Rokid RG-glasses。
     * 结果会缓存，后续调用直接返回缓存值。
     *
     * @return 如果是目标设备则返回 true，否则返回 false。
     */
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
}
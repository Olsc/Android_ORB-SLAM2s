package com.orb.slam2s.compat;

import android.os.Build;
import android.util.Log;

import com.orb.slam2s.slamar.OpenCVBridge;

// 设备兼容性处理类：针对特定设备（如 Rokid RG-glasses）进行特殊画面处理或逻辑调整
public class DeviceCompat_RokidGlass3 {
    private static final String TAG = "DeviceCompat_RokidGlass3";

    // 目标设备信息
    private static final String TARGET_MANUFACTURER = "Rokid";
    private static final String TARGET_MODEL = "RG-glasses";
    private static final String TARGET_PRODUCT = "glasses";
    private static final String TARGET_CODENAME = "glasses";

    // 缓存：设备信息不会在运行时改变，只需检测一次
    private static Boolean sIsRokidGlasses = null;

    // 检查当前设备是否为 Rokid RG-glasses，结果缓存后直接返回
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

    // Rokid 设备时对相机画面做上下与左右镜像处理
    public static void checkAndFlipFrame(long matAddr) {
        if (matAddr == 0) return;

        if (isRokidGlasses()) {
            OpenCVBridge.nativeFlipBoth(matAddr);
        }
    }
}
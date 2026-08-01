package com.orb.slam2s.compat;

import android.os.Build;
import android.util.Log;

import com.orb.slam2s.slamar.OpenCVBridge;

/**
 * 设备兼容性处理类
 * 用于针对特定设备（如 Rokid RG-glasses）进行特殊的画面处理或逻辑调整。
 */
public class DeviceCompat_RokidGlass3 {
    private static final String TAG = "DeviceCompat_RokidGlass3";

    // 目标设备信息
    private static final String TARGET_MANUFACTURER = "Rokid";
    private static final String TARGET_MODEL = "RG-glasses";
    private static final String TARGET_PRODUCT = "glasses";
    private static final String TARGET_CODENAME = "glasses";

    // 缓存：设备信息不会在运行时改变，只需检测一次
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

    /**
     * 如果是 Rokid 设备，则对相机画面进行镜像处理。
     * 用户要求：画面需要上下镜像 + 左右镜像。
     *
     * @param matAddr 需要处理的 native Mat 地址
     */
    public static void checkAndFlipFrame(long matAddr) {
        if (matAddr == 0) return;

        if (isRokidGlasses()) {
            OpenCVBridge.nativeFlipBoth(matAddr);
        }
    }
}
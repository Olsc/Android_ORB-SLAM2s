package com.orb.slam2s.slamar;

import android.graphics.Bitmap;

/**
 * OpenCV Bridge - 替代 org.opencv.* 的 Java OpenCV 调用
 *
 * 通过 JNI 调用 Tem/opencv 编译的原生 OpenCV 函数，
 * 提供 Mat 创建、图像处理、Bitmap 转换等功能。
 *
 * 注意：依赖的 native 库 (MenthaAR_Engine) 在 NativeHelper 中加载。
 */
public class OpenCVBridge {
    // OpenCV Mat Type Constants
    public static final int CV_8UC1 = 0;
    public static final int CV_8UC4 = 24;

    // ==================== Mat 生命周期 ====================

    /**
     * 在 native 堆上创建 Mat 对象
     * @param rows 行数（高）
     * @param cols 列数（宽）
     * @param type Mat 类型，如 CV_8UC4=4, CV_8UC1=1
     * @return native Mat 对象的地址（long 指针）
     */
    public static native long nativeCreateMat(int rows, int cols, int type);

    /**
     * 释放 native Mat 对象
     */
    public static native void nativeReleaseMat(long matAddr);

    // ==================== Mat 数据操作 ====================

    // public static native void nativePutData(long matAddr, byte[] data);  // 暂未使用；C++ 侧实现仍被 nativePutBuffer 的 fallback 路径调用，勿删

    /**
     * 将 ByteBuffer 直填入 Mat（减少 Java byte[] 中间拷贝）
     */
    public static native void nativePutBuffer(long matAddr, java.nio.ByteBuffer buffer);

    /**
     * 将 Mat 填充为纯色
     */
    public static native void nativeMatSetTo(long matAddr, double v1, double v2, double v3, double v4);

    /**
     * 将 Y-plane 数据（一行一行的平坦数组）写入 RGBA 和 Gray Mat
     * @param rgbaMatAddr CV_8UC4 Mat（R=G=B=Y, A=255）
     * @param grayMatAddr CV_8UC1 Mat
     * @param yData 已去除 stride 填充的 Y 平面数据（width*height 字节）
     * @param width 图像宽度
     * @param height 图像高度
     */
    public static native void nativeYPlaneToMats(long rgbaMatAddr, long grayMatAddr,
                                                  byte[] yData, int width, int height);

    // ==================== 图像处理 ====================

    /**
     * RGBA 转 Gray
     */
    public static native void nativeRGBA2Gray(long srcAddr, long dstAddr);

    /**
     * 旋转 180 度
     */
    public static native void nativeRotate180(long matAddr);

    /**
     * 同时翻转 X 和 Y 轴（上下+左右镜像）
     */
    public static native void nativeFlipBoth(long matAddr);

    // ==================== Bitmap 转换 ====================

    /**
     * native Mat 转为 Android Bitmap (ARGB_8888)
     */
    public static native void nativeMatToBitmap(long matAddr, Bitmap bitmap);

    /**
     * Android Bitmap 转为 native Mat
     */
    public static native void nativeBitmapToMat(Bitmap bitmap, long matAddr);

    // ==================== 时间测量 ====================

    // public static native double nativeGetTickFrequency();  // 暂未使用（app 无调用点）
    // public static native long nativeGetTickCount();        // 暂未使用（app 无调用点）
}
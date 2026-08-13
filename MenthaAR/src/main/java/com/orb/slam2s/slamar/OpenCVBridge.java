package com.orb.slam2s.slamar;

import android.graphics.Bitmap;

// OpenCV Bridge：替代 org.opencv.* 的 Java 调用，通过 JNI 调用原生 OpenCV 函数
// 提供 Mat 创建、图像处理、Bitmap 转换等功能，依赖库在 NativeHelper 中加载
public class OpenCVBridge {
    // OpenCV Mat 类型常量
    public static final int CV_8UC1 = 0;
    public static final int CV_8UC4 = 24;

    // ==================== Mat 生命周期 ====================

    // 在 native 堆上创建 Mat 对象，返回 native Mat 地址
    public static native long nativeCreateMat(int rows, int cols, int type);

    // 释放 native Mat 对象
    public static native void nativeReleaseMat(long matAddr);

    // ==================== Mat 数据操作 ====================

    // public static native void nativePutData(long matAddr, byte[] data);  // 暂未使用；C++ 侧实现仍被 nativePutBuffer 的 fallback 路径调用，勿删

    // 将 ByteBuffer 直填入 Mat，减少 Java byte[] 中间拷贝
    public static native void nativePutBuffer(long matAddr, java.nio.ByteBuffer buffer);

    // 将 Mat 填充为纯色
    public static native void nativeMatSetTo(long matAddr, double v1, double v2, double v3, double v4);

    // 将去除 stride 填充的 Y-plane 数据写入 RGBA 和 Gray Mat
    public static native void nativeYPlaneToMats(long rgbaMatAddr, long grayMatAddr,
                                                  byte[] yData, int width, int height);

    // 将 Direct ByteBuffer 的 Y-plane 直写入 RGBA 和 Gray Mat，零 Java 中间拷贝
    public static native void nativeDirectYPlaneToMats(long rgbaMatAddr, long grayMatAddr,
                                                      java.nio.ByteBuffer yBufDirect, int width, int height);

    // ==================== 图像处理 ====================

    // RGBA 转 Gray
    public static native void nativeRGBA2Gray(long srcAddr, long dstAddr);

    // 旋转 180 度
    public static native void nativeRotate180(long matAddr);

    // 同时翻转 X 和 Y 轴，实现上下与左右镜像
    public static native void nativeFlipBoth(long matAddr);

    // ==================== Bitmap 转换 ====================

    // native Mat 转为 Android Bitmap（ARGB_8888）
    public static native void nativeMatToBitmap(long matAddr, Bitmap bitmap);

    // Android Bitmap 转为 native Mat
    public static native void nativeBitmapToMat(Bitmap bitmap, long matAddr);

    // ==================== 时间测量 ====================

    // public static native double nativeGetTickFrequency();  // 暂未使用（app 无调用点）
    // public static native long nativeGetTickCount();        // 暂未使用（app 无调用点）
}
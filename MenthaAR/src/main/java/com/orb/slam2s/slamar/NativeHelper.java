package com.orb.slam2s.slamar;

import android.util.Log;

// NativeHelper类：与JNI层交互，处理摄像头帧、平面检测、SLAM初始化等功能
public class NativeHelper {
    private static final String TAG = "NativeHelper";

    // 加载必要的本地库
    static {
        try {
            System.loadLibrary("Mentha_Profiler");
        } catch (UnsatisfiedLinkError e) {
            Log.d(TAG, "未开启Profiler");
        }
        System.loadLibrary("MenthaAR_Engine"); // 整合后的 SLAM_AR（含 OpenCV 原生模块）
    }

    private int planeDetectResult; // 平面检测结果

    private final int[] statusBuf = new int[3];

    public NativeHelper() {
    }

    // 检测平面（在 SLAM 处理线程内调用）
    // MVP 结果由 native 写回共享内存，UI 渲染线程直接读取；返回平面检测结果
    public int detectPlane() {
        detect(statusBuf);
        planeDetectResult = statusBuf[1];
        return planeDetectResult;
    }

    // 本地方法：持久映射共享内存帧缓冲（仅在缓冲创建/尺寸变化时调用一次）
    public native boolean nativeAttachFrameBuffer(int fd, int size);
    public native void nativeDetachFrameBuffer();

    // 本地方法：处理持久映射缓冲中的帧（双缓冲 bufIndex 0/1；每帧只传序号+宽高，无 fd 开销）
    // native 处理完成后会把 tracking/draw/MVP/点云/slamDoneSeq 直接写回共享内存 header
    public native void nativeProcessFrameSharedMem(int bufIndex, int seq, int width, int height, int[] statusBuf);

    public boolean attachFrameBuffer(int fd, int size) {
        return nativeAttachFrameBuffer(fd, size);
    }

    public void detachFrameBuffer() {
        nativeDetachFrameBuffer();
    }

    public void processFrameSharedMem(int bufIndex, int seq, int width, int height, int[] statusBuf) {
        nativeProcessFrameSharedMem(bufIndex, seq, width, height, statusBuf);
    }

    // 统一获取MVP
    public native void nativeGetMVP(float[] M, float[] V, float[] P, int w, int h);

    // 本地方法：进行平面检测
    public native void detect(int[] statusBuf);

    // 本地方法：初始化SLAM
    public native void initSLAM(String path);

    // 关停并释放 SLAM 系统（join 全部工作线程后 delete）
    public native void nativeShutdown();

    public void shutdownSLAM() {
        try {
            nativeShutdown();
        } catch (UnsatisfiedLinkError e) {
            // 库未加载时静默忽略
        }
    }

    // 本地方法：更新相机分辨率并重新计算内参
    public native void nativeUpdateResolution(int cameraWidth, int cameraHeight);

    public void updateResolution(int width, int height) {
        if (width > 0 && height > 0) {
            nativeUpdateResolution(width, height);
        }
    }

    // 本地方法：保存/加载地图
    public native void saveMap(String path);
    public native void loadMapWithId(String path, int mapId, boolean append);
    public native int[] getMapStats();

    // 点云显示控制（控制绿色和蓝色点云）
    public native void setPointCloudDisplay(boolean enable);
    public native boolean isPointCloudDisplayEnabled();

    // AR对象缩放
    public native void updateArObjectScale(float scaleFactor);

    public native float[] getMiniMapPoints(int maxPoints);
    public native float[] getTrackedPoints(int maxPoints);
    public native float[] getAllArObjectsData();

    // 视图矩阵
    public native void getV(float viewM[]);
}
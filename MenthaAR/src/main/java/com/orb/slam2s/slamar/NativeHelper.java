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

package com.orb.slam2s.slamar;

import android.util.Log;

// NativeHelper：JNI 接口封装，负责摄像头帧处理、平面检测与 SLAM 初始化。
public class NativeHelper {
    private static final String TAG = "NativeHelper";

    // 加载必要的本地库
    static {
        // 必须在 dlopen 前关闭 libomp 的亲和性绑定：部分 ROM（MIUI 等）限制了应用进程的
        // CPU 亲和性，libomp 初始化时 sched_setaffinity 失败会直接 abort（引擎进程一启动即崩溃）
        try {
            android.system.Os.setenv("KMP_AFFINITY", "disabled", true);
            // libomp 默认在并行区结束后自旋 200 ms 再休眠。设备实测（slam_bench，1017 帧，同步回放）：
            // 0 ms 时工作线程所在核降频/迁核，跟踪均值 13.6→19.7 ms；30/50 ms 时紧跟关键帧之后的帧慢约 6 ms（p90 27→33）；
            // ≥100 ms 与默认一致。相机 30 fps 时两次提取之间的间隔约 25 ms，60 ms 足以让工作线程在帧间保持唤醒，
            // 同时在暂停/低帧率时把空转限制在 60 ms 内
            android.system.Os.setenv("KMP_BLOCKTIME", "60", true);
        } catch (Throwable t) {
            Log.w(TAG, "设置 KMP 环境变量失败", t);
        }
        try {
            System.loadLibrary("Mentha_Profiler");
        } catch (UnsatisfiedLinkError e) {
            Log.d(TAG, "未开启Profiler");
        }
        System.loadLibrary("MenthaAR_Engine"); // 整合后的 SLAM_AR(含 OpenCV 原生模块)
    }

    private final int[] statusBuf = new int[3];

    // 与 native 共享的 direct 缓冲（布局见 native-lib.cpp POSE_BUF_*）：
    //  float[0..47] 原始 M/V/P；int[48..50] 跟踪状态/绘制标志/地图 id；float[51..98] 融合后的 M/V/P。
    // native 通过 GetDirectBufferAddress 只在注册时取一次地址，此后每帧零 JNI 数组传输
    public static final int POSE_BUF_FLOATS = 99;
    private static final int POSE_RAW = 0;
    private static final int POSE_INFO = 48;
    private static final int POSE_FUSED = 51;
    private final java.nio.ByteBuffer poseBuf =
            java.nio.ByteBuffer.allocateDirect(POSE_BUF_FLOATS * 4).order(java.nio.ByteOrder.nativeOrder());
    private final java.nio.FloatBuffer poseFloats = poseBuf.asFloatBuffer();
    private final java.nio.IntBuffer poseInts = poseBuf.asIntBuffer();
    private final boolean poseBufRegistered;

    public NativeHelper() {
        poseBufRegistered = nativeRegisterPoseBuffer(poseBuf);
        if (!poseBufRegistered) {
            Log.w(TAG, "注册位姿缓冲失败，回退到 native 内部原始位姿");
        }
    }

    // 在 SLAM 处理线程内检测平面，MVP 结果由 native 写回共享内存
    public void detectPlane() {
        detect(statusBuf);
    }

    // 持久映射共享内存帧缓冲(仅在创建或尺寸变化时调用)
    public native boolean nativeAttachFrameBuffer(int fd, int size);
    public native void nativeDetachFrameBuffer();

    // 处理持久映射缓冲中的帧(双缓冲 bufIndex 0/1，每帧只传序号与宽高)
    // native 处理完成后写回 tracking/点云，并把原始位姿写进已注册的 poseBuf；
    // MVP 与 slamDoneSeq 由随后的 publishArPose/publishRawArPose 一次性发布
    public native void nativeProcessFrameSharedMem(int bufIndex, int seq, int width, int height);
    private native boolean nativeRegisterPoseBuffer(java.nio.ByteBuffer buffer);

    public boolean attachFrameBuffer(int fd, int size) {
        return nativeAttachFrameBuffer(fd, size);
    }

    public void detachFrameBuffer() {
        nativeDetachFrameBuffer();
    }

    public void processFrameSharedMem(int bufIndex, int seq, int width, int height) {
        nativeProcessFrameSharedMem(bufIndex, seq, width, height);
    }

    // 执行平面检测
    public native void detect(int[] statusBuf);

    // 初始化 SLAM
    public native void initSLAM(String path);

    // 关停并释放 SLAM 系统(join 全部工作线程后 delete)
    public native void nativeShutdown();

    public void shutdownSLAM() {
        try {
            nativeShutdown();
        } catch (UnsatisfiedLinkError e) {
            // 库未加载时静默忽略
        }
    }

    // 更新相机分辨率并重新计算内参
    public native void nativeUpdateResolution(int cameraWidth, int cameraHeight);

    public void updateResolution(int width, int height) {
        if (width > 0 && height > 0) {
            nativeUpdateResolution(width, height);
        }
    }

    // 保存与加载地图
    public native void saveMap(String path);
    public native void loadMapWithId(String path, int mapId, boolean append);
    public native int[] getMapStats();

    // 点云显示控制(控制绿色和蓝色点云)
    public native void setPointCloudDisplay(boolean enable);
    public native boolean isPointCloudDisplayEnabled();

    // 发布最终 AR 位姿（每帧只写一次 MVP，seqlock 包裹）并推进 slamDoneSeq。useRaw 时直接发布原始位姿
    private native void nativePublishArPose(boolean draw, boolean useRaw);

    // 读取当前帧原始 AR 位姿：out48 = Model/View/Proj，outInfo = [跟踪状态, 绘制标志, 地图ID]（直接读共享缓冲，无 JNI）
    public boolean getArPose(float[] out48, int[] outInfo) {
        if (!poseBufRegistered) return false;
        poseFloats.position(POSE_RAW);
        poseFloats.get(out48, 0, 48);
        outInfo[0] = poseInts.get(POSE_INFO);
        outInfo[1] = poseInts.get(POSE_INFO + 1);
        outInfo[2] = poseInts.get(POSE_INFO + 2);
        return true;
    }

    // 把融合结果写进共享缓冲后发布
    public void publishArPose(float[] model, float[] view, float[] projection, boolean draw) {
        if (!poseBufRegistered) {
            nativePublishArPose(draw, true);
            return;
        }
        poseFloats.position(POSE_FUSED);
        poseFloats.put(model, 0, 16);
        poseFloats.put(view, 0, 16);
        poseFloats.put(projection, 0, 16);
        nativePublishArPose(draw, false);
    }

    // 融合关闭或失败时直接发布原始位姿（绘制标志由 native 取原始值）
    public void publishRawArPose() {
        nativePublishArPose(false, true);
    }

    // AR对象缩放
    public native void updateArObjectScale(float scaleFactor);
}
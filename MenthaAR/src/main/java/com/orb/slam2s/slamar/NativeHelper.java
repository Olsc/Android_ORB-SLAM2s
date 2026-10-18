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
        try {
            System.loadLibrary("Mentha_Profiler");
        } catch (UnsatisfiedLinkError e) {
            Log.d(TAG, "未开启Profiler");
        }
        System.loadLibrary("MenthaAR_Engine"); // 整合后的 SLAM_AR（含 OpenCV 原生模块）
    }

    private final int[] statusBuf = new int[3];

    public NativeHelper() {
    }

    // 在 SLAM 处理线程内检测平面，MVP 结果由 native 写回共享内存
    public void detectPlane() {
        detect(statusBuf);
    }

    // 持久映射共享内存帧缓冲（仅在创建或尺寸变化时调用）
    public native boolean nativeAttachFrameBuffer(int fd, int size);
    public native void nativeDetachFrameBuffer();

    // 处理持久映射缓冲中的帧（双缓冲 bufIndex 0/1，每帧只传序号与宽高）
    // native 处理完成后将 tracking/draw/MVP/点云/slamDoneSeq 写回共享内存 header
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

    // 执行平面检测
    public native void detect(int[] statusBuf);

    // 初始化 SLAM
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

    // 点云显示控制（控制绿色和蓝色点云）
    public native void setPointCloudDisplay(boolean enable);
    public native boolean isPointCloudDisplayEnabled();

    // AR对象缩放
    public native void updateArObjectScale(float scaleFactor);
}
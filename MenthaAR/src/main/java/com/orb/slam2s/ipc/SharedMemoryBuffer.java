/*
 * Copyright (C) 2026 Olsc <OlscStudio@outlook.com>
 *
 * This file is part of the Android ORB-SLAM2s IPC Client Module.
 *
 * Dual-licensed under the Apache License, Version 2.0 (the "Apache License")
 * or the GNU General Public License, version 3 (the "GPLv3").
 *
 * Under the Apache License, Version 2.0:
 *   Licensed under the Apache License, Version 2.0 (the "License");
 *   you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at
 *       http://www.apache.org/licenses/LICENSE-2.0
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *
 * Under the GPLv3:
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *   See <https://www.gnu.org/licenses/> for details.
 */
package com.orb.slam2s.ipc;

import android.annotation.SuppressLint;
import android.os.Build;
import android.os.MemoryFile;
import android.os.ParcelFileDescriptor;
import android.os.SharedMemory;
import android.util.Log;

import java.io.FileDescriptor;
import java.lang.reflect.Method;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * SLAM 跨进程共享内存（帧 + 结果回传）。
 *
 * 布局（header 用 LITTLE_ENDIAN，与 native 侧 C/C++ 一致）：
 * <pre>
 *   [0..3]    magic "MNTH"
 *   [4..7]    version
 *   [8..11]   frameW
 *   [12..15]  frameH
 *   [16..19]  uiWriteSeq     (UI 进程写：最新已写入共享内存的帧序号)
 *   [20..23]  slamDoneSeq    (SLAM 进程写：已处理完成的帧序号)
 *   [24..27]  trackingState  (SLAM 进程写)
 *   [28..31]  drawFlag       (SLAM 进程写)
 *   [32..35]  pointCloudBytes(SLAM 进程写：点云区有效字节数)
 *   [36..39]  reserved
 *   [40..231] mvp[48] float  (SLAM 进程写：M/V/P 各 16，共 48)
 *   [232..255] reserved
 *   [256..]              Y 帧缓冲 0 (frameW*frameH 字节)
 *   [256+w*h..]          Y 帧缓冲 1 (frameW*frameH 字节)
 *   [256+2*w*h..]        点云区 (POINTCLOUD_MAX_BYTES)
 * </pre>
 *
 * 双缓冲 + 序号同步：UI 写 buf[seq%2] 前必须满足 slamDoneSeq >= seq-2，
 * 保证 SLAM 正在读的缓冲（seq-1）不会被覆盖；SLAM 完成后写 slamDoneSeq。
 * 所有 header 字段用绝对偏移读写（不移动 ByteBuffer position），线程安全。
 */
public class SharedMemoryBuffer {
    private static final String TAG = "SharedMemoryBuffer";

    // 布局常量（与 native-lib.cpp 中 SH_* 保持一致，勿单独修改）
    public static final int HEADER_MAGIC = 0x4D4E5448; // "MNTH"
    public static final int HEADER_VERSION = 2;
    public static final int HEADER_SIZE = 256;

    public static final int OFF_FRAME_W = 8;
    public static final int OFF_FRAME_H = 12;
    public static final int OFF_UI_WRITE_SEQ = 16;
    public static final int OFF_SLAM_DONE_SEQ = 20;
    public static final int OFF_DRAW_FLAG = 28;
    public static final int OFF_POINTCLOUD_BYTES = 32;
    public static final int OFF_MVP = 40;   // 48 floats = 192 字节

    /** 点云区上限（3000 点 × 7 floats × 4B = 84000，取 96KB 对齐） */
    public static final int POINTCLOUD_MAX_BYTES = 96 * 1024;

    private SharedMemory sharedMemory;
    private MemoryFile memoryFile;
    private ByteBuffer mappedBuffer;
    private ParcelFileDescriptor pfd;
    private ParcelFileDescriptor mFdOwner;
    private final int bufferSize;

    private int frameW;
    private int frameH;

    public SharedMemoryBuffer(String name, int size) {
        this.bufferSize = size;
        try {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O_MR1) { // API 27+
                sharedMemory = SharedMemory.create(name, size);
                mappedBuffer = sharedMemory.mapReadWrite();
                mappedBuffer.order(ByteOrder.LITTLE_ENDIAN); // 与 native 字节序一致
                mFdOwner = ParcelFileDescriptor.fromFd(getRawFd(sharedMemory));
                pfd = ParcelFileDescriptor.dup(mFdOwner.getFileDescriptor());
            } else {
                memoryFile = new MemoryFile(name, size);
                @SuppressLint("DiscouragedPrivateApi") Method getFdMethod = MemoryFile.class.getDeclaredMethod("getFileDescriptor");
                FileDescriptor fd = (FileDescriptor) getFdMethod.invoke(memoryFile);
                pfd = ParcelFileDescriptor.dup(fd);
            }
        } catch (Exception e) {
            Log.e(TAG, "创建共享内存失败: " + e.getMessage(), e);
        }
    }

    // 通过反射获取 SharedMemory 底层 fd（getFd 在高版本系统上仍被允许访问）
    @SuppressLint("DiscouragedPrivateApi")
    private int getRawFd(SharedMemory sharedMemory) {
        try {
            Method getFdMethod = null;
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O_MR1) {
                getFdMethod = SharedMemory.class.getDeclaredMethod("getFd");
            }
            if (getFdMethod == null) {
                return -1;
            }
            getFdMethod.setAccessible(true);
            Object fdObj = getFdMethod.invoke(sharedMemory);
            return (fdObj != null) ? (int) fdObj : -1;
        } catch (Exception e) {
            Log.w(TAG, "获取 SharedMemory fd 异常: " + e.getMessage());
            return -1;
        }
    }

    // 布局计算

    /** 设置当前帧尺寸（影响 Y 双缓冲与点云区偏移）。必须在写帧前调用。 */
    public void setFrameSize(int w, int h) {
        this.frameW = w;
        this.frameH = h;
    }

    public int getFrameW() { return frameW; }
    public int getFrameH() { return frameH; }

    /** Y 缓冲区起始偏移 */
    private int yOffset(int bufIndex) {
        return HEADER_SIZE + (bufIndex & 1) * (frameW * frameH);
    }

    /** 点云区起始偏移 */
    private int pointCloudOffset() {
        return HEADER_SIZE + 2 * (frameW * frameH);
    }

    /** 按当前帧尺寸计算完整共享内存所需大小（供创建/重建判断） */
    public static int requiredSize(int w, int h) {
        return HEADER_SIZE + 2 * w * h + POINTCLOUD_MAX_BYTES;
    }

    // header 读写（绝对偏移，线程安全）

    /** 初始化 header（首次写帧前调用一次）。返回 false 表示缓冲区不可用。 */
    public boolean initHeader(int w, int h) {
        setFrameSize(w, h);
        if (mappedBuffer == null && memoryFile == null) return false;
        writeInt(0, HEADER_MAGIC);
        writeInt(4, HEADER_VERSION);
        writeInt(OFF_FRAME_W, w);
        writeInt(OFF_FRAME_H, h);
        writeInt(OFF_UI_WRITE_SEQ, 0);
        writeInt(OFF_SLAM_DONE_SEQ, 0);
        return true;
    }

    private void writeInt(int offset, int value) {
        if (mappedBuffer != null) {
            mappedBuffer.putInt(offset, value);
        } else if (memoryFile != null) {
            byte[] b = new byte[]{
                    (byte) (value & 0xFF), (byte) ((value >> 8) & 0xFF),
                    (byte) ((value >> 16) & 0xFF), (byte) ((value >> 24) & 0xFF)};
            try {
                memoryFile.writeBytes(b, 0, offset, 4);
            } catch (Exception e) {
                Log.e(TAG, "writeInt 失败: " + e.getMessage());
            }
        }
    }

    private int readInt(int offset) {
        if (mappedBuffer != null) {
            return mappedBuffer.getInt(offset);
        } else if (memoryFile != null) {
            byte[] b = new byte[4];
            try {
                memoryFile.readBytes(b, offset, 0, 4);
            } catch (Exception e) {
                Log.e(TAG, "readInt 失败: " + e.getMessage());
                return 0;
            }
            return (b[0] & 0xFF) | ((b[1] & 0xFF) << 8) | ((b[2] & 0xFF) << 16) | ((b[3] & 0xFF) << 24);
        }
        return 0;
    }

    public void writeUiWriteSeq(int seq) { writeInt(OFF_UI_WRITE_SEQ, seq); }

    /** SLAM 已处理完成的帧序号（UI 读取，用于背压判断与渲染版本检测） */
    public int readSlamDoneSeq() { return readInt(OFF_SLAM_DONE_SEQ); }

    /** 是否应绘制 AR 物体（SLAM 进程写入） */
    public int readDrawFlag() { return readInt(OFF_DRAW_FLAG); }

    /** 读取最新 MVP（48 floats：M[16]+V[16]+P[16]）。返回 false 表示无有效数据。 */
    public boolean readMvp(float[] out48) {
        if (out48 == null || out48.length < 48) return false;
        if (mappedBuffer != null) {
            for (int i = 0; i < 48; i++) {
                out48[i] = mappedBuffer.getFloat(OFF_MVP + i * 4);
            }
            return true;
        } else if (memoryFile != null) {
            byte[] b = new byte[48 * 4];
            try {
                memoryFile.readBytes(b, OFF_MVP, 0, b.length);
            } catch (Exception e) {
                Log.e(TAG, "readMvp 失败: " + e.getMessage());
                return false;
            }
            ByteBuffer bb = ByteBuffer.wrap(b).order(ByteOrder.LITTLE_ENDIAN);
            for (int i = 0; i < 48; i++) out48[i] = bb.getFloat(i * 4);
            return true;
        }
        return false;
    }

    /** 读取点云区（每点 7 floats：[x,y,z,r,g,b,size]）。返回点数，0 表示无数据。 */
    public int readPointCloud(float[] out, int maxFloats) {
        int bytes = readInt(OFF_POINTCLOUD_BYTES);
        if (bytes <= 0 || bytes > POINTCLOUD_MAX_BYTES || out == null) return 0;
        int floats = Math.min(bytes / 4, Math.min(maxFloats, out.length));
        if (floats <= 0) return 0;
        if (mappedBuffer != null) {
            int base = pointCloudOffset();
            for (int i = 0; i < floats; i++) {
                out[i] = mappedBuffer.getFloat(base + i * 4);
            }
        } else if (memoryFile != null) {
            byte[] b = new byte[floats * 4];
            try {
                memoryFile.readBytes(b, pointCloudOffset(), 0, b.length);
            } catch (Exception e) {
                Log.e(TAG, "readPointCloud 失败: " + e.getMessage());
                return 0;
            }
            ByteBuffer bb = ByteBuffer.wrap(b).order(ByteOrder.LITTLE_ENDIAN);
            for (int i = 0; i < floats; i++) out[i] = bb.getFloat(i * 4);
        }
        return floats;
    }

    // 帧写入

    // 将一帧灰度 Y 数据写入指定缓冲（bufIndex 0/1）
    // 调用方必须已确认该缓冲可写（slamDoneSeq >= seq-2）
    public boolean writeFrame(byte[] yData, int bufIndex, int w, int h) {
        if (frameW != w || frameH != h) {
            setFrameSize(w, h);
        }
        int count = Math.min(w * h, yData.length);
        if (mappedBuffer != null) {
            int base = yOffset(bufIndex);
            mappedBuffer.position(base);
            mappedBuffer.put(yData, 0, count);
        } else if (memoryFile != null) {
            try {
                memoryFile.writeBytes(yData, 0, yOffset(bufIndex), count);
            } catch (Exception e) {
                Log.e(TAG, "writeFrame 失败: " + e.getMessage());
                return false;
            }
        } else {
            return false;
        }
        return true;
    }

    // 通用访问

    public ParcelFileDescriptor getParcelFileDescriptor() {
        return pfd;
    }

    public int getBufferSize() {
        return bufferSize;
    }

    public void close() {
        try {
            if (mappedBuffer != null && Build.VERSION.SDK_INT >= Build.VERSION_CODES.O_MR1) {
                SharedMemory.unmap(mappedBuffer);
                mappedBuffer = null;
            }
            if (sharedMemory != null && Build.VERSION.SDK_INT >= Build.VERSION_CODES.O_MR1) {
                sharedMemory.close();
                sharedMemory = null;
            }
            if (memoryFile != null) {
                memoryFile.close();
                memoryFile = null;
            }
            if (mFdOwner != null) {
                try {
                    mFdOwner.close();
                } catch (Exception ignored) {}
                mFdOwner = null;
            }
            if (pfd != null) {
                pfd.close();
                pfd = null;
            }
        } catch (Exception e) {
            Log.e(TAG, "关闭共享内存失败: " + e.getMessage());
        }
    }
}
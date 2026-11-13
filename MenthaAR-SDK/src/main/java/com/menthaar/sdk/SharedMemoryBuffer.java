/*
 * Copyright (C) 2026 Olsc <OlscStudio@outlook.com>
 *
 * This file is part of the Android ORB-SLAM2s SDK.
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
package com.menthaar.sdk;

import android.annotation.SuppressLint;
import android.os.Build;
import android.os.MemoryFile;
import android.os.Parcel;
import android.os.ParcelFileDescriptor;
import android.os.SharedMemory;
import android.util.Log;

import java.io.FileDescriptor;
import java.lang.reflect.Method;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

// 共享内存布局：256 字节头部 + Y 灰度双缓冲 + 点云区，常量与 native 的 SH_* 一致。
final class SharedMemoryBuffer {
    private static final String TAG = "MenthaAR";

    private static final int HEADER_SIZE = 256;

    private static final int HEADER_MAGIC = 0x4D4E5448; // "MNTH"
    private static final int HEADER_VERSION = 2;

    private static final int OFF_FRAME_W = 8;
    private static final int OFF_FRAME_H = 12;
    private static final int OFF_UI_WRITE_SEQ = 16;
    private static final int OFF_SLAM_DONE_SEQ = 20;
    private static final int OFF_TRACKING_STATE = 24;
    private static final int OFF_DRAW_FLAG = 28;
    private static final int OFF_POINTCLOUD_BYTES = 32;
    private static final int OFF_ACTIVE_MAP_ID = 36;
    private static final int OFF_MVP = 40;          // 48 floats = M[16] + V[16] + P[16]
    private static final int OFF_MVP_SEQ = 232;     // seqlock，奇数表示服务端正在写
    private static final int OFF_POINTCLOUD_SEQ = 236;

    // 点云上限：3000 点 * 7 floats * 4B，取 96 KB 对齐
    static final int POINTCLOUD_MAX_BYTES = 96 * 1024;

    // readResult 输出数组的下标。
    static final int STATUS_TRACKING = 0;
    static final int STATUS_DRAWABLE = 1;
    static final int STATUS_MAP_ID = 2;
    static final int STATUS_POINTCLOUD_SEQ = 3;
    static final int STATUS_FRAME_ID = 4;
    static final int STATUS_LENGTH = 5;

    private final int bufferSize;

    private SharedMemory sharedMemory;          // API 27+ 共享内存
    private MemoryFile memoryFile;              // API < 27 匿名内存文件 (ashmem)
    private ByteBuffer mappedBuffer;            // API 27+ 本进程映射的直接内存
    private FloatBuffer mappedFloatView;        // mappedBuffer 的 float 视图
    private ParcelFileDescriptor pfd;           // 传递给 SLAM 进程的 fd

    // 读映射与关闭共用这把锁，close() 会 munmap，锁外访问 mappedBuffer 即 use-after-unmap。
    private final Object readLock = new Object();

    // seqlock 读端全屏障：一次 volatile 写接一次 volatile 读构成 StoreLoad 屏障，保证序号与数据的读取不被重排（ART 对 volatile 生成 ldar/stlr/dmb）
    private volatile int fence;
    private void fullFence() {
        fence = 0;
        int v = fence;
        if (v != 0) fence = v;
    }

    // 帧尺寸由写线程（sliceFrame/writeFrame）与读线程（pointCloudOffset/yOffset）共享，volatile 保证可见性。
    private volatile int frameW;
    private volatile int frameH;

    @SuppressWarnings("JavaReflectionMemberAccess")
    SharedMemoryBuffer(String name, int size) {
        this.bufferSize = size;
        try {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O_MR1) {
                sharedMemory = SharedMemory.create(name, size);
                mappedBuffer = sharedMemory.mapReadWrite();
                mappedBuffer.order(ByteOrder.LITTLE_ENDIAN);
                mappedFloatView = mappedBuffer.asFloatBuffer();

                // 借 Parcelable 序列化在底层 dup fd
                Parcel parcel = Parcel.obtain();
                try {
                    sharedMemory.writeToParcel(parcel, 0);
                    parcel.setDataPosition(0);
                    pfd = parcel.readFileDescriptor();
                } finally {
                    parcel.recycle();
                }
                if (pfd == null) {
                    throw new IllegalStateException("无法从 SharedMemory 获取 ParcelFileDescriptor");
                }
            } else {
                // API 23~26: 使用系统 MemoryFile (ashmem 纯内存)
                memoryFile = new MemoryFile(name, size);
                @SuppressLint("DiscouragedPrivateApi")
                Method getFdMethod = MemoryFile.class.getDeclaredMethod("getFileDescriptor");
                getFdMethod.setAccessible(true);
                FileDescriptor fd = (FileDescriptor) getFdMethod.invoke(memoryFile);
                if (fd != null && fd.valid()) {
                    pfd = ParcelFileDescriptor.dup(fd);
                } else {
                    throw new IllegalStateException("无法从 MemoryFile 获取有效 FileDescriptor");
                }
            }
        } catch (Exception e) {
            Log.e(TAG, "创建跨进程共享内存失败: " + e.getMessage(), e);
            close();
        }
    }

    static int requiredSize(int width, int height) {
        return HEADER_SIZE + 2 * width * height + POINTCLOUD_MAX_BYTES;
    }

    static int pointCloudCapacity() {
        return POINTCLOUD_MAX_BYTES / 4;
    }

    boolean isAvailable() {
        return mappedBuffer != null || memoryFile != null;
    }

    boolean initHeader(int width, int height) {
        if (!isAvailable()) return false;
        frameW = width;
        frameH = height;
        writeInt(0, HEADER_MAGIC);
        writeInt(4, HEADER_VERSION);
        writeInt(OFF_FRAME_W, width);
        writeInt(OFF_FRAME_H, height);
        writeInt(OFF_UI_WRITE_SEQ, 0);
        writeInt(OFF_SLAM_DONE_SEQ, 0);
        writeInt(OFF_MVP_SEQ, 0);
        writeInt(OFF_POINTCLOUD_SEQ, 0);
        return true;
    }

    int frameWidth() {
        return frameW;
    }

    int frameHeight() {
        return frameH;
    }

    int capacity() {
        return bufferSize;
    }

    ParcelFileDescriptor getParcelFileDescriptor() {
        return pfd;
    }

    void writeUiWriteSeq(int seq) {
        writeInt(OFF_UI_WRITE_SEQ, seq);
    }

    // 主线程每个 vsync 都会调用：必须与 close() 互斥，否则可能读到已 munmap 的映射
    int readSlamDoneSeq() {
        synchronized (readLock) {
            return readInt(OFF_SLAM_DONE_SEQ);
        }
    }

    // 一次 seqlock 读窗口内读取 48 个矩阵 float，窗口内只做数据拷贝，失败返回 false。
    // 状态字段在序号校验通过后补读，不参与一致性判定。
    boolean readResult(float[] outMvp, int[] outStatus) {
        if (outMvp == null || outMvp.length < 48 || outStatus == null || outStatus.length < STATUS_LENGTH) {
            return false;
        }
        synchronized (readLock) {
            if (mappedFloatView != null) {
                for (int attempt = 0; attempt < 4; attempt++) {
                    final int s1 = mappedBuffer.getInt(OFF_MVP_SEQ);
                    if ((s1 & 1) != 0) continue;
                    fullFence();
                    mappedFloatView.position(OFF_MVP / 4);
                    mappedFloatView.get(outMvp, 0, 48);
                    fullFence();
                    if (s1 != mappedBuffer.getInt(OFF_MVP_SEQ)) continue;
                    readStatus(outStatus);
                    return true;
                }
                return false;
            }
            if (memoryFile != null) {
                try {
                    byte[] bytes = new byte[48 * 4];
                    memoryFile.readBytes(bytes, OFF_MVP, 0, bytes.length);
                    ByteBuffer bb = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN);
                    for (int i = 0; i < 48; i++) {
                        outMvp[i] = bb.getFloat(i * 4);
                    }
                    readStatus(outStatus);
                    return true;
                } catch (Exception e) {
                    Log.e(TAG, "读取追踪结果失败: " + e.getMessage());
                }
            }
            return false;
        }
    }

    // 读取点云（每点 7 floats: x, y, z, r, g, b, size），返回 float 个数。
    // 点云区无服务端 seqlock，用前后序号做乐观校验，序号变化则重试一次，两侧不一致返回 0。
    int readPointCloud(float[] out, int maxFloats) {
        if (out == null || out.length <= 0) return 0;
        synchronized (readLock) {
            final boolean mapped = mappedFloatView != null;
            if (!mapped && memoryFile == null) return 0;
            for (int attempt = 0; attempt < 2; attempt++) {
                final int seq1 = readInt(OFF_POINTCLOUD_SEQ);
                final int bytes = readInt(OFF_POINTCLOUD_BYTES);
                if (bytes <= 0 || bytes > POINTCLOUD_MAX_BYTES) return 0;
                final int floats = Math.min(bytes / 4, Math.min(maxFloats, out.length));
                if (floats <= 0) return 0;

                if (mapped) {
                    mappedFloatView.position(pointCloudOffset() / 4);
                    mappedFloatView.get(out, 0, floats);
                } else {
                    try {
                        byte[] bytesBuf = new byte[floats * 4];
                        memoryFile.readBytes(bytesBuf, pointCloudOffset(), 0, bytesBuf.length);
                        ByteBuffer bb = ByteBuffer.wrap(bytesBuf).order(ByteOrder.LITTLE_ENDIAN);
                        for (int i = 0; i < floats; i++) {
                            out[i] = bb.getFloat(i * 4);
                        }
                    } catch (Exception e) {
                        Log.e(TAG, "读取点云失败: " + e.getMessage());
                        return 0;
                    }
                }
                fullFence();
                if (seq1 == readInt(OFF_POINTCLOUD_SEQ)) return floats;
            }
            // 连续两次都被服务端重写打断：本帧放弃，等下一次点云序号变化再读
            return 0;
        }
    }

    // 读状态字段（调用方已持有 readLock，且位姿快照已校验通过）。
    // frameId 取 SLAM_DONE_SEQ：它由服务端在发布完 MVP 之后才推进，因此可能比快照领先一帧。
    private void readStatus(int[] outStatus) {
        outStatus[STATUS_TRACKING] = readInt(OFF_TRACKING_STATE);
        outStatus[STATUS_DRAWABLE] = readInt(OFF_DRAW_FLAG);
        outStatus[STATUS_MAP_ID] = readInt(OFF_ACTIVE_MAP_ID);
        outStatus[STATUS_POINTCLOUD_SEQ] = readInt(OFF_POINTCLOUD_SEQ);
        outStatus[STATUS_FRAME_ID] = readInt(OFF_SLAM_DONE_SEQ);
    }

    // 返回 Y 缓冲的直接切片供零拷贝写入；API 27 以下无映射时返回 null。
    ByteBuffer sliceFrame(int bufIndex, int width, int height) {
        if (mappedBuffer == null) return null;
        frameW = width;
        frameH = height;
        final int offset = yOffset(bufIndex);
        final int length = width * height;
        if (offset < 0 || length <= 0 || offset + length > bufferSize) return null;
        ByteBuffer dup = mappedBuffer.duplicate();
        dup.position(offset);
        dup.limit(offset + length);
        return dup.slice().order(ByteOrder.nativeOrder());
    }

    // 把一帧灰度拷入第 bufIndex 个 Y 缓冲。
    boolean writeFrame(byte[] gray, int bufIndex, int width, int height) {
        if (!isAvailable() || gray == null) return false;
        frameW = width;
        frameH = height;
        final int count = Math.min(width * height, gray.length);
        if (mappedBuffer != null) {
            mappedBuffer.position(yOffset(bufIndex));
            mappedBuffer.put(gray, 0, count);
            return true;
        }
        try {
            memoryFile.writeBytes(gray, 0, yOffset(bufIndex), count);
            return true;
        } catch (Exception e) {
            Log.e(TAG, "写入灰度帧失败: " + e.getMessage());
            return false;
        }
    }

    private int yOffset(int bufIndex) {
        return HEADER_SIZE + (bufIndex & 1) * (frameW * frameH);
    }

    private int pointCloudOffset() {
        return HEADER_SIZE + 2 * (frameW * frameH);
    }

    private void writeInt(int offset, int value) {
        if (mappedBuffer != null) {
            mappedBuffer.putInt(offset, value);
        } else if (memoryFile != null) {
            byte[] bytes = new byte[]{
                    (byte) (value & 0xFF),
                    (byte) ((value >> 8) & 0xFF),
                    (byte) ((value >> 16) & 0xFF),
                    (byte) ((value >> 24) & 0xFF)};
            try {
                memoryFile.writeBytes(bytes, 0, offset, 4);
            } catch (Exception e) {
                Log.e(TAG, "写入头字段失败: " + e.getMessage());
            }
        }
    }

    private int readInt(int offset) {
        if (mappedBuffer != null) {
            return mappedBuffer.getInt(offset);
        }
        if (memoryFile != null) {
            byte[] bytes = new byte[4];
            try {
                memoryFile.readBytes(bytes, offset, 0, 4);
                return (bytes[0] & 0xFF) | ((bytes[1] & 0xFF) << 8)
                        | ((bytes[2] & 0xFF) << 16) | ((bytes[3] & 0xFF) << 24);
            } catch (Exception e) {
                Log.e(TAG, "读取头字段失败: " + e.getMessage());
            }
        }
        return 0;
    }

    // 释放映射；与读取共用 readLock，写入侧由调用方保证已停止。
    void close() {
        synchronized (readLock) {
            try {
                if (mappedBuffer != null && Build.VERSION.SDK_INT >= Build.VERSION_CODES.O_MR1) {
                    SharedMemory.unmap(mappedBuffer);
                }
                mappedBuffer = null;
                mappedFloatView = null;
                if (sharedMemory != null && Build.VERSION.SDK_INT >= Build.VERSION_CODES.O_MR1) {
                    sharedMemory.close();
                    sharedMemory = null;
                }
                if (memoryFile != null) {
                    memoryFile.close();
                    memoryFile = null;
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
}
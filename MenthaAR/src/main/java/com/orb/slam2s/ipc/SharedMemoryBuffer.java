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

import android.os.Build;
import android.os.Parcel;
import android.os.ParcelFileDescriptor;
import android.os.SharedMemory;
import android.util.Log;

import java.io.File;
import java.io.FileInputStream;
import java.io.RandomAccessFile;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.FileChannel;

/**
 * SLAM 跨进程共享内存：Header + Y 双缓冲 + 点云区。
 * 布局常量需与 native-lib.cpp 中的 SH_* 保持一致。
 */
public class SharedMemoryBuffer {
    private static final String TAG = "SharedMemoryBuffer";

    public static final int HEADER_MAGIC = 0x4D4E5448; // "MNTH"
    public static final int HEADER_VERSION = 2;
    public static final int HEADER_SIZE = 256;

    public static final int OFF_FRAME_W = 8;
    public static final int OFF_FRAME_H = 12;
    public static final int OFF_UI_WRITE_SEQ = 16;
    public static final int OFF_SLAM_DONE_SEQ = 20;
    public static final int OFF_DRAW_FLAG = 28;
    public static final int OFF_POINTCLOUD_BYTES = 32;
    public static final int OFF_MVP = 40; // 48 floats = 192 bytes

    // 点云上限：3000 点 × 7 floats × 4B，取 96KB 对齐
    public static final int POINTCLOUD_MAX_BYTES = 96 * 1024;

    private final int bufferSize;

    private SharedMemory sharedMemory; // API 27+ 的底层共享内存
    private ParcelFileDescriptor pfd;  // 交给 SLAM 进程的 fd
    private FileInputStream mapStream; // API 27- 的文件映射流
    private ByteBuffer mappedBuffer;   // 本进程映射视图

    private int frameW;
    private int frameH;

    public SharedMemoryBuffer(String name, int size, File cacheDir) {
        this.bufferSize = size;
        try {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O_MR1) {
                sharedMemory = SharedMemory.create(name, size);
                mappedBuffer = sharedMemory.mapReadWrite();
                // 借 Parcelable 序列化复制 fd，避免反射隐藏接口 SharedMemory#getFd()
                Parcel parcel = Parcel.obtain();
                try {
                    sharedMemory.writeToParcel(parcel, 0);
                    parcel.setDataPosition(0);
                    pfd = parcel.readFileDescriptor();
                } finally {
                    parcel.recycle();
                }
                if (pfd == null) {
                    throw new IllegalStateException("无法获取共享内存 fd");
                }
            } else {
                pfd = openAnonymousFile(cacheDir, name, size);
                mapStream = new FileInputStream(pfd.getFileDescriptor());
                mappedBuffer = mapStream.getChannel().map(
                        FileChannel.MapMode.READ_WRITE, 0, size);
            }
            mappedBuffer.order(ByteOrder.LITTLE_ENDIAN);
        } catch (Exception e) {
            Log.e(TAG, "创建共享内存失败: " + e.getMessage(), e);
            close();
        }
    }

    // API 27 以下无 SharedMemory，用匿名临时文件承载跨进程 mmap 区域
    private static ParcelFileDescriptor openAnonymousFile(File dir, String name, int size) throws Exception {
        File file = new File(dir, name);
        RandomAccessFile raf = new RandomAccessFile(file, "rw");
        try {
            raf.setLength(size);
        } finally {
            raf.close();
        }
        ParcelFileDescriptor descriptor = ParcelFileDescriptor.open(
                file, ParcelFileDescriptor.MODE_READ_WRITE);
        file.delete(); // fd 已持有 inode，删除目录项避免残留
        return descriptor;
    }

    public void setFrameSize(int w, int h) {
        this.frameW = w;
        this.frameH = h;
    }

    public int getFrameW() { return frameW; }
    public int getFrameH() { return frameH; }

    private int yOffset(int bufIndex) {
        return HEADER_SIZE + (bufIndex & 1) * (frameW * frameH);
    }

    private int pointCloudOffset() {
        return HEADER_SIZE + 2 * (frameW * frameH);
    }

    public static int requiredSize(int w, int h) {
        return HEADER_SIZE + 2 * w * h + POINTCLOUD_MAX_BYTES;
    }

    public boolean initHeader(int w, int h) {
        if (mappedBuffer == null) return false;
        setFrameSize(w, h);
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
        }
    }

    private int readInt(int offset) {
        return mappedBuffer != null ? mappedBuffer.getInt(offset) : 0;
    }

    public void writeUiWriteSeq(int seq) { writeInt(OFF_UI_WRITE_SEQ, seq); }

    public int readSlamDoneSeq() { return readInt(OFF_SLAM_DONE_SEQ); }

    public int readDrawFlag() { return readInt(OFF_DRAW_FLAG); }

    public boolean readMvp(float[] out48) {
        if (mappedBuffer == null || out48 == null || out48.length < 48) return false;
        for (int i = 0; i < 48; i++) {
            out48[i] = mappedBuffer.getFloat(OFF_MVP + i * 4);
        }
        return true;
    }

    // 每点 7 floats: [x, y, z, r, g, b, size]，返回点数
    public int readPointCloud(float[] out, int maxFloats) {
        if (mappedBuffer == null || out == null) return 0;
        int bytes = readInt(OFF_POINTCLOUD_BYTES);
        if (bytes <= 0 || bytes > POINTCLOUD_MAX_BYTES) return 0;
        int floats = Math.min(bytes / 4, Math.min(maxFloats, out.length));
        int base = pointCloudOffset();
        for (int i = 0; i < floats; i++) {
            out[i] = mappedBuffer.getFloat(base + i * 4);
        }
        return floats;
    }

    // 调用方需先确认目标缓冲空闲（readSlamDoneSeq() >= seq - 2）
    public boolean writeFrame(byte[] yData, int bufIndex, int w, int h) {
        if (mappedBuffer == null) return false;
        if (frameW != w || frameH != h) {
            setFrameSize(w, h);
        }
        int count = Math.min(w * h, yData.length);
        mappedBuffer.position(yOffset(bufIndex));
        mappedBuffer.put(yData, 0, count);
        return true;
    }

    public ParcelFileDescriptor getParcelFileDescriptor() {
        return pfd;
    }

    public int getBufferSize() {
        return bufferSize;
    }

    public void close() {
        try {
            if (mappedBuffer != null && sharedMemory != null
                    && Build.VERSION.SDK_INT >= Build.VERSION_CODES.O_MR1) {
                SharedMemory.unmap(mappedBuffer);
            }
            mappedBuffer = null;
            if (mapStream != null) {
                mapStream.close();
                mapStream = null;
            }
            if (sharedMemory != null) {
                sharedMemory.close();
                sharedMemory = null;
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
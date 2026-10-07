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
import android.os.Parcel;
import android.os.ParcelFileDescriptor;
import android.os.SharedMemory;
import android.util.Log;

import java.io.File;
import java.io.FileDescriptor;
import java.lang.reflect.Method;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

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

    private SharedMemory sharedMemory; // API 27+ 共享内存
    private MemoryFile memoryFile;     // API < 27 匿名内存文件 (ashmem)
    private ByteBuffer mappedBuffer;   // API 27+ 本进程映射直接内存
    private ParcelFileDescriptor pfd;  // 跨进程传递给 SLAM 服务端的 fd

    private int frameW;
    private int frameH;

    public SharedMemoryBuffer(String name, int size) {
        this.bufferSize = size;
        try {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O_MR1) {
                sharedMemory = SharedMemory.create(name, size);
                mappedBuffer = sharedMemory.mapReadWrite();
                mappedBuffer.order(ByteOrder.LITTLE_ENDIAN);

                // 借 Parcelable 序列化在底层 dup fd，避免调用非公开隐藏接口 SharedMemory#getFd()
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

    @Deprecated
    public SharedMemoryBuffer(String name, int size, @SuppressWarnings("unused") File cacheDir) {
        this(name, size);
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
        if (mappedBuffer == null && memoryFile == null) return false;
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
        } else if (memoryFile != null) {
            byte[] b = new byte[]{
                    (byte) (value & 0xFF),
                    (byte) ((value >> 8) & 0xFF),
                    (byte) ((value >> 16) & 0xFF),
                    (byte) ((value >> 24) & 0xFF)};
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
                return (b[0] & 0xFF) | ((b[1] & 0xFF) << 8) | ((b[2] & 0xFF) << 16) | ((b[3] & 0xFF) << 24);
            } catch (Exception e) {
                Log.e(TAG, "readInt 失败: " + e.getMessage());
            }
        }
        return 0;
    }

    public void writeUiWriteSeq(int seq) { writeInt(OFF_UI_WRITE_SEQ, seq); }

    public int readSlamDoneSeq() { return readInt(OFF_SLAM_DONE_SEQ); }

    public int readDrawFlag() { return readInt(OFF_DRAW_FLAG); }

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
                ByteBuffer bb = ByteBuffer.wrap(b).order(ByteOrder.LITTLE_ENDIAN);
                for (int i = 0; i < 48; i++) {
                    out48[i] = bb.getFloat(i * 4);
                }
                return true;
            } catch (Exception e) {
                Log.e(TAG, "readMvp 失败: " + e.getMessage());
            }
        }
        return false;
    }

    // 每点 7 floats: [x, y, z, r, g, b, size]，返回点数
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
            return floats;
        } else if (memoryFile != null) {
            byte[] b = new byte[floats * 4];
            try {
                memoryFile.readBytes(b, pointCloudOffset(), 0, b.length);
                ByteBuffer bb = ByteBuffer.wrap(b).order(ByteOrder.LITTLE_ENDIAN);
                for (int i = 0; i < floats; i++) {
                    out[i] = bb.getFloat(i * 4);
                }
                return floats;
            } catch (Exception e) {
                Log.e(TAG, "readPointCloud 失败: " + e.getMessage());
            }
        }
        return 0;
    }

    // 调用方需先确认目标缓冲空闲（readSlamDoneSeq() >= seq - 2）
    public boolean writeFrame(byte[] yData, int bufIndex, int w, int h) {
        if (mappedBuffer == null && memoryFile == null) return false;
        if (frameW != w || frameH != h) {
            setFrameSize(w, h);
        }
        int count = Math.min(w * h, yData.length);
        if (mappedBuffer != null) {
            mappedBuffer.position(yOffset(bufIndex));
            mappedBuffer.put(yData, 0, count);
            return true;
        } else if (memoryFile != null) {
            try {
                memoryFile.writeBytes(yData, 0, yOffset(bufIndex), count);
                return true;
            } catch (Exception e) {
                Log.e(TAG, "writeFrame 失败: " + e.getMessage());
            }
        }
        return false;
    }

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
            }
            mappedBuffer = null;
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
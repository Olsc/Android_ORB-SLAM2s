/**
 * Copyright (C) 2026 Olsc <OlscStudio@outlook.com>
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

public class SharedMemoryBuffer {
    private static final String TAG = "SharedMemoryBuffer";

    private SharedMemory sharedMemory;
    private MemoryFile memoryFile;
    private ByteBuffer mappedBuffer;
    private ParcelFileDescriptor pfd;
    private ParcelFileDescriptor mFdOwner;
    private final int bufferSize;

    public SharedMemoryBuffer(String name, int size) {
        this.bufferSize = size;
        try {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O_MR1) { // API 27+
                sharedMemory = SharedMemory.create(name, size);
                mappedBuffer = sharedMemory.mapReadWrite();
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

    /**
     * 通过反射获取 SharedMemory 底层 fd（getFd 在高版本系统上仍被允许访问）。
     */
    @SuppressLint("DiscouragedPrivateApi")
    private int getRawFd(SharedMemory sharedMemory) {
        try {
            Method getFdMethod = null;
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O_MR1) {
                getFdMethod = SharedMemory.class.getDeclaredMethod("getFd");
            }
            assert getFdMethod != null;
            getFdMethod.setAccessible(true);
            return (int) getFdMethod.invoke(sharedMemory);
        } catch (Exception e) {
            Log.w(TAG, "获取 SharedMemory fd 异常: " + e.getMessage());
            return -1;
        }
    }

    public void writeData(byte[] data, int length) {
        if (mappedBuffer != null) {
            mappedBuffer.position(0);
            mappedBuffer.put(data, 0, Math.min(length, bufferSize));
        } else if (memoryFile != null) {
            try {
                memoryFile.writeBytes(data, 0, 0, Math.min(length, bufferSize));
            } catch (Exception e) {
                Log.e(TAG, "MemoryFile 写入失败: " + e.getMessage());
            }
        }
    }

    public ByteBuffer getByteBuffer() {
        return mappedBuffer;
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

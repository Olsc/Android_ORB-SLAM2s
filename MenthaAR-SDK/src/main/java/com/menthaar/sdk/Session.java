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

import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.os.IBinder;
import android.os.ParcelFileDescriptor;
import android.os.RemoteException;
import android.util.Log;

import com.menthaar.sdk.internal.ISlamService;

import java.nio.ByteBuffer;

// SDK 内部实现：服务绑定、控制指令下发、共享内存收发帧与结果。
final class Session {
    private static final String TAG = "MenthaAR";

    // SLAM 服务组件，由整合后的 AAR 提供，运行在独立进程 :slam_process。
    private static final String SERVICE_CLASS = "com.orb.slam2s.ipc.SlamService";

    interface Observer {
        // 服务已连接，且已下发引擎初始化指令。
        void onConnected();

        // 服务进程断开。
        void onDisconnected();
    }

    private final Context appContext;

    private volatile ISlamService service;
    private volatile boolean bindRequested;
    private volatile String workingDir;

    private volatile Observer observer;

    private volatile SharedMemoryBuffer frameBuffer;
    // 引擎初始化且调用方配置下发完成后才接受帧，防止未 initSLAM 的服务进程消耗帧序号。
    private volatile boolean framesAllowed;
    private final Object frameLock = new Object();
    private int lastSeq;
    private boolean attachPending;      // 重连后需要把已有共享内存重新交给新服务进程
    private ByteBuffer scratchBuffer;   // API < 27 无直接映射时的中转缓冲
    private byte[] copyBuffer;          // 中转缓冲拷入共享内存时的复用数组

    private final ServiceConnection connection = new ServiceConnection() {
        @Override
        public void onServiceConnected(ComponentName name, IBinder binder) {
            if (!bindRequested) return;
            final ISlamService s = ISlamService.Stub.asInterface(binder);
            Log.i(TAG, "SLAM 服务已连接");
            // 每次（重）连接都重新初始化引擎：服务进程可能是被系统重建的全新实例。
            // 服务端 initSLAM 幂等，重复下发是安全的。
            final String dir = workingDir;
            if (dir != null) {
                try {
                    s.initSLAM(dir);
                } catch (RemoteException e) {
                    Log.e(TAG, "初始化 SLAM 失败: " + e.getMessage());
                }
            }
            // 初始化指令下发之后才发布 service：帧提交一定排在 initSLAM 之后
            service = s;
            framesAllowed = false;
            // 新服务进程没有旧的共享内存映射：标记待重连，下一帧提交时重新交出 fd（不在主线程做 Binder 调用）
            synchronized (frameLock) {
                if (frameBuffer != null) {
                    attachPending = true;
                }
            }
            final Observer o = observer;
            if (o != null) o.onConnected();
        }

        @Override
        public void onServiceDisconnected(ComponentName name) {
            service = null;
            framesAllowed = false;
            Log.w(TAG, "SLAM 服务已断开");
            final Observer o = observer;
            if (o != null) o.onDisconnected();
        }

        @Override
        public void onNullBinding(ComponentName name) {
            // 部分 ROM（如 MIUI）在子进程频繁崩溃后会拒绝拉起服务，返回空 binder
            handleBindingFailure("SLAM 服务返回空 binder，绑定失败", name);
        }

        @Override
        public void onBindingDied(ComponentName name) {
            handleBindingFailure("SLAM 服务绑定失效（进程死亡）", name);
        }
    };

    Session(Context context) {
        this.appContext = context.getApplicationContext();
    }

    void setObserver(Observer observer) {
        this.observer = observer;
    }

    // 绑定失败/失效：清理注册标记并通知调用方连接已断开（系统限制无法自动恢复时由用户重启应用）。
    private synchronized void handleBindingFailure(String reason, ComponentName name) {
        service = null;
        bindRequested = false;
        try {
            appContext.unbindService(connection);
        } catch (IllegalArgumentException ignored) {
            // 绑定已由系统清理
        }
        Log.e(TAG, reason + ": " + name);
        final Observer o = observer;
        if (o != null) o.onDisconnected();
    }

    boolean isConnected() {
        return service != null;
    }

    // 由调用方在引擎初始化 + 配置下发完成后开启送帧
    void setFramesAllowed(boolean allowed) {
        framesAllowed = allowed;
    }

    // 是否已发起绑定（绑定中或已连接）。
    boolean isBinding() {
        return bindRequested;
    }

    // 绑定服务并记住工作目录，重连时自动用于初始化引擎。
    void bind(String workingDir) {
        synchronized (this) {
            this.workingDir = workingDir;
            if (bindRequested) return;
            Intent intent = new Intent();
            intent.setComponent(new ComponentName(appContext.getPackageName(), SERVICE_CLASS));
            bindRequested = appContext.bindService(intent, connection, Context.BIND_AUTO_CREATE);
            if (!bindRequested) {
                Log.e(TAG, "绑定 SLAM 服务失败：服务未声明或被系统限制");
            }
        }
    }

    // 解绑服务并释放共享内存。重复调用是安全的。
    void unbind() {
        synchronized (this) {
            service = null;
            framesAllowed = false;
            if (bindRequested) {
                bindRequested = false;
                try {
                    appContext.unbindService(connection);
                } catch (IllegalArgumentException e) {
                    Log.w(TAG, "解绑 SLAM 服务失败: " + e.getMessage());
                }
            }
        }
        synchronized (frameLock) {
            if (frameBuffer != null) {
                frameBuffer.close();
                frameBuffer = null;
            }
            lastSeq = 0;
            attachPending = false;
        }
    }

    void setCameraRotation(int degrees) {
        ISlamService s = service;
        if (s == null) return;
        try {
            s.setCameraImageRotation(degrees);
        } catch (RemoteException e) {
            Log.e(TAG, "设置相机旋转角失败: " + e.getMessage());
        }
    }

    void setPoseAssistEnabled(boolean enabled) {
        ISlamService s = service;
        if (s == null) return;
        try {
            s.set3DofAssistEnabled(enabled);
        } catch (RemoteException e) {
            Log.e(TAG, "切换姿态辅助失败: " + e.getMessage());
        }
    }

    void setPoseAssistReversed(boolean reversed) {
        ISlamService s = service;
        if (s == null) return;
        try {
            s.setSensorReverse(reversed);
        } catch (RemoteException e) {
            Log.e(TAG, "设置传感器方向失败: " + e.getMessage());
        }
    }

    int getPoseAssistDof() {
        ISlamService s = service;
        if (s == null) return 0;
        try {
            return s.getSensorDof();
        } catch (RemoteException e) {
            Log.e(TAG, "读取姿态传感器能力失败: " + e.getMessage());
            return 0;
        }
    }

    void setPointCloudVisible(boolean visible) {
        ISlamService s = service;
        if (s == null) return;
        try {
            s.setPointCloudDisplay(visible);
        } catch (RemoteException e) {
            Log.e(TAG, "切换点云显示失败: " + e.getMessage());
        }
    }

    void detectPlane() {
        ISlamService s = service;
        if (s == null) return;
        try {
            s.detectPlane();
        } catch (RemoteException e) {
            Log.e(TAG, "平面检测失败: " + e.getMessage());
        }
    }

    void setArObjectScale(float scale) {
        ISlamService s = service;
        if (s == null) return;
        try {
            s.updateArObjectScale(scale);
        } catch (RemoteException e) {
            Log.e(TAG, "设置 AR 物体缩放失败: " + e.getMessage());
        }
    }

    void saveMap(String path) {
        ISlamService s = service;
        if (s == null || path == null) return;
        try {
            s.saveMap(path);
        } catch (RemoteException e) {
            Log.e(TAG, "保存地图失败: " + e.getMessage());
        }
    }

    void loadMap(String path, int mapId, boolean append) {
        ISlamService s = service;
        if (s == null || path == null) return;
        try {
            s.loadMapWithId(path, mapId, append);
        } catch (RemoteException e) {
            Log.e(TAG, "加载地图失败: " + e.getMessage());
        }
    }

    int[] getMapStats() {
        ISlamService s = service;
        if (s == null) return null;
        try {
            return s.getMapStats();
        } catch (RemoteException e) {
            Log.e(TAG, "读取地图统计失败: " + e.getMessage());
            return null;
        }
    }

    // 提交一帧灰度图（内部拷贝到共享内存）。
    boolean submitFrame(byte[] gray, int width, int height) {
        if (gray == null || width <= 0 || height <= 0 || gray.length < width * height) return false;
        synchronized (frameLock) {
            final int seq = prepareFrameLocked(width, height);
            if (seq <= 0) return false;
            final int index = seq & 1;
            if (!frameBuffer.writeFrame(gray, index, width, height)) return false;
            commitFrameLocked(seq, index, width, height);
            return true;
        }
    }

    // 获取可写帧槽位；null 表示未接收，写完须调用 submitFrame 提交。
    MenthaAR.Frame acquireFrame(int width, int height) {
        if (width <= 0 || height <= 0) return null;
        synchronized (frameLock) {
            final int seq = prepareFrameLocked(width, height);
            if (seq <= 0) return null;
            final int index = seq & 1;
            final ByteBuffer slice = frameBuffer.sliceFrame(index, width, height);
            if (slice != null) {
                return new MenthaAR.Frame(slice, width, height, seq, index, false);
            }
            // API < 27 的 MemoryFile 没有直接映射：给一块 direct 中转缓冲，提交时再拷入共享内存
            final int size = width * height;
            if (scratchBuffer == null || scratchBuffer.capacity() < size) {
                scratchBuffer = ByteBuffer.allocateDirect(size);
            }
            scratchBuffer.clear();
            scratchBuffer.limit(size);
            return new MenthaAR.Frame(scratchBuffer, width, height, seq, index, true);
        }
    }

    // 提交 acquireFrame 得到的帧。
    boolean submitFrame(MenthaAR.Frame frame) {
        if (frame == null) return false;
        synchronized (frameLock) {
            if (frameBuffer == null || service == null) return false;
            // 期间共享内存若被重建（分辨率变化）或已有新帧提交，该槽位作废
            if (frame.seq != lastSeq + 1
                    || frameBuffer.frameWidth() != frame.width
                    || frameBuffer.frameHeight() != frame.height) {
                return false;
            }
            if (frame.needsCopy) {
                final int size = frame.width * frame.height;
                if (copyBuffer == null || copyBuffer.length < size) {
                    copyBuffer = new byte[size];
                }
                frame.gray.position(0);
                frame.gray.get(copyBuffer, 0, size);
                if (!frameBuffer.writeFrame(copyBuffer, frame.bufferIndex, frame.width, frame.height)) {
                    return false;
                }
            }
            commitFrameLocked(frame.seq, frame.bufferIndex, frame.width, frame.height);
            return true;
        }
    }

    boolean readResult(float[] outMvp, int[] outStatus) {
        final SharedMemoryBuffer buffer = frameBuffer;
        return buffer != null && buffer.readResult(outMvp, outStatus);
    }

    int readPointCloud(float[] out, int maxFloats) {
        final SharedMemoryBuffer buffer = frameBuffer;
        return buffer != null ? buffer.readPointCloud(out, maxFloats) : 0;
    }

    // 最新结果序号；无共享内存时返回 -1（表示无数据）。
    int readSlamDoneSeq() {
        final SharedMemoryBuffer buffer = frameBuffer;
        return buffer != null ? buffer.readSlamDoneSeq() : -1;
    }

    // 持 frameLock 创建/挂接共享内存并做背压检查，返回帧序号，0 表示丢弃。
    private int prepareFrameLocked(int width, int height) {
        if (service == null || !framesAllowed) return 0;

        // 服务进程重建后，先把已有缓冲重新交给它
        if (attachPending) {
            attachPending = false;
            if (frameBuffer != null) {
                attachFrameBufferLocked();
            }
        }

        final int requiredSize = SharedMemoryBuffer.requiredSize(width, height);
        if (frameBuffer == null
                || frameBuffer.capacity() < requiredSize
                || frameBuffer.frameWidth() != width
                || frameBuffer.frameHeight() != height) {
            if (frameBuffer != null) {
                frameBuffer.close();
            }
            frameBuffer = new SharedMemoryBuffer("MenthaARFrameBuffer", requiredSize);
            if (!frameBuffer.initHeader(width, height)) {
                frameBuffer = null;
                return 0;
            }
            attachFrameBufferLocked();
            lastSeq = 0;
        }
        if (frameBuffer == null) return 0;

        final int seq = lastSeq + 1;
        // 背压：目标缓冲（seq & 1）必须空闲，即 SLAM 已完成 seq-2。落后时丢帧，不覆盖正在处理的缓冲
        if (frameBuffer.readSlamDoneSeq() < seq - 2) {
            return 0;
        }
        return seq;
    }

    // 持 frameLock：写入帧序号并 oneway 投递，不阻塞调用线程。
    private void commitFrameLocked(int seq, int bufferIndex, int width, int height) {
        frameBuffer.writeUiWriteSeq(seq);
        lastSeq = seq;
        final ISlamService s = service;
        if (s == null) return;
        try {
            s.processFrame(seq, bufferIndex, width, height);
        } catch (RemoteException e) {
            Log.e(TAG, "投递帧失败: " + e.getMessage());
        }
    }

    // 持 frameLock：把共享内存 fd 一次性交给 SLAM 进程。
    private void attachFrameBufferLocked() {
        final ISlamService s = service;
        if (s == null || frameBuffer == null) return;
        final ParcelFileDescriptor pfd = frameBuffer.getParcelFileDescriptor();
        if (pfd == null) return;
        try {
            if (!s.attachFrameBuffer(pfd, frameBuffer.capacity())) {
                attachPending = true;
                Log.e(TAG, "SLAM 进程映射共享内存失败，将在下一帧重试");
            }
        } catch (RemoteException e) {
            Log.e(TAG, "映射共享内存失败: " + e.getMessage());
        }
    }
}
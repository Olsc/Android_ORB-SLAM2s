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

import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.os.IBinder;
import android.os.ParcelFileDescriptor;
import android.os.RemoteException;
import android.util.Log;

import java.nio.ByteBuffer;

// MenthaAR 定位 SDK 对外接口：控制指令经 AIDL 下发，位姿与点云经共享内存读取。
public class SlamIPCClient {
    private static final String TAG = "SlamIPCClient";

    // 以组件名绑定
    private static final String SERVICE_CLASS = "com.orb.slam2s.ipc.SlamService";

    // 服务连接回调，供 App 在连接后同步配置项
    public interface ConnectionListener {
        void onServiceConnected();

        void onServiceDisconnected();
    }

    private final Context context;
    private ISlamService slamService;
    private boolean isBound;
    private volatile ConnectionListener connectionListener;

    private volatile SharedMemoryBuffer sharedMemoryBuffer;

    private final Object sendLock = new Object();
    private int lastSeq = 0;  // 已投递的帧序号

    // 资源目录在首次 initSLAM 时记录，之后每次 onServiceConnected 都重新下发：
    // onPause 解绑后服务进程被系统销毁，onResume 重新绑定得到的是全新进程，必须再初始化一次（服务端 initSLAM 幂等）
    private String resDir;

    private final ServiceConnection serviceConnection = new ServiceConnection() {
        @Override
        public void onServiceConnected(ComponentName name, IBinder service) {
            slamService = ISlamService.Stub.asInterface(service);
            isBound = true;
            Log.d(TAG, "成功绑定到 MenthaAR SlamService 进程");
            try {
                if (resDir != null) {
                    Log.d(TAG, "服务已连接，下发 SLAM 初始化: " + resDir);
                    slamService.initSLAM(resDir);
                }
            } catch (RemoteException e) {
                Log.e(TAG, "初始化异常: " + e.getMessage());
            }
            final ConnectionListener listener = connectionListener;
            if (listener != null) {
                listener.onServiceConnected();
            }
        }

        @Override
        public void onServiceDisconnected(ComponentName name) {
            slamService = null;
            isBound = false;
            Log.w(TAG, "MenthaAR SlamService 进程连接断开");
            final ConnectionListener listener = connectionListener;
            if (listener != null) {
                listener.onServiceDisconnected();
            }
        }
    };

    public SlamIPCClient(Context context) {
        this.context = context;
    }

    public void setConnectionListener(ConnectionListener listener) {
        this.connectionListener = listener;
    }

    // 绑定 SDK 服务，以组件名寻址，不依赖 GPL 实现类
    public void bindService() {
        Intent intent = new Intent();
        intent.setComponent(new ComponentName(context.getPackageName(), SERVICE_CLASS));
        context.bindService(intent, serviceConnection, Context.BIND_AUTO_CREATE);
    }

    public void unbindService() {
        if (isBound && slamService != null) {
            context.unbindService(serviceConnection);
            isBound = false;
        }
        // 发送线程可能正在 sendFrameData 内写共享内存（持 sendLock）；SharedMemory.unmap 之后再访问会直接崩溃
        synchronized (sendLock) {
            if (sharedMemoryBuffer != null) {
                sharedMemoryBuffer.close();
                sharedMemoryBuffer = null;
            }
        }
    }

    public boolean isConnected() {
        return isBound && slamService != null;
    }

    // 初始化引擎，未连接时在连接后执行；资源目录会保留，重连后自动重新初始化
    public void initSLAM(String resDir) {
        this.resDir = resDir;
        if (slamService != null) {
            try {
                slamService.initSLAM(resDir);
            } catch (RemoteException e) {
                Log.e(TAG, "initSLAM 异常: " + e.getMessage());
            }
        } else {
            Log.w(TAG, "SlamService 尚未就绪，挂起 SLAM 初始化指令: " + resDir);
        }
    }

    // 将一帧灰度 Y 数据写入共享内存并投递给 SLAM 进程(oneway，不阻塞)
    // 内部双缓冲 + 背压丢帧，可在任意线程调用(内部已串行化)
    public void sendFrameData(byte[] frameData, int width, int height) {
        if (slamService == null || frameData == null) return;
        if (width <= 0 || height <= 0) return;

        synchronized (sendLock) {
            final int seq = prepareFrameSlotLocked(width, height);
            if (seq <= 0) return;
            final int bufIndex = seq & 1;

            if (!sharedMemoryBuffer.writeFrame(frameData, bufIndex, width, height)) {
                return;
            }
            commitFrameLocked(seq, bufIndex, width, height);
        }
    }

    // 已获取的帧槽位：gray 是共享内存中目标 Y 缓冲的 direct 切片，调用方写满后 commitFrame
    public static final class FrameSlot {
        public final ByteBuffer gray;
        public final int seq;
        final int bufIndex;
        final int width;
        final int height;

        FrameSlot(ByteBuffer gray, int seq, int bufIndex, int width, int height) {
            this.gray = gray;
            this.seq = seq;
            this.bufIndex = bufIndex;
            this.width = width;
            this.height = height;
        }
    }

    // 获取下一帧的共享内存槽位（含重建/attach/背压检查），让调用方的转换代码直接把灰度写进共享内存，
    // 省去 byte[] → 共享内存的一次整帧拷贝。背压丢帧或没有直接映射（API<27）时返回 null
    public FrameSlot acquireFrameSlot(int width, int height) {
        if (slamService == null || width <= 0 || height <= 0) return null;
        synchronized (sendLock) {
            final int seq = prepareFrameSlotLocked(width, height);
            if (seq <= 0) return null;
            final int bufIndex = seq & 1;
            final ByteBuffer slice = sharedMemoryBuffer.sliceFrame(bufIndex, width, height);
            if (slice == null) return null;
            return new FrameSlot(slice, seq, bufIndex, width, height);
        }
    }

    // 灰度已写入槽位后投递给 SLAM 进程。期间若共享内存被重建（尺寸变化），该槽位作废
    public void commitFrame(FrameSlot slot) {
        if (slot == null) return;
        synchronized (sendLock) {
            if (sharedMemoryBuffer == null || slamService == null) return;
            if (slot.seq != lastSeq + 1
                    || sharedMemoryBuffer.getFrameW() != slot.width
                    || sharedMemoryBuffer.getFrameH() != slot.height) {
                return;
            }
            commitFrameLocked(slot.seq, slot.bufIndex, slot.width, slot.height);
        }
    }

    // 持 sendLock：按需重建共享内存并 attach，做背压检查；返回下一帧序号，0 表示本帧应丢弃
    private int prepareFrameSlotLocked(int width, int height) {
        // 尺寸变化或缓冲不足时，重建共享内存并重新 attach
        int requiredSize = SharedMemoryBuffer.requiredSize(width, height);
        if (sharedMemoryBuffer == null
                || sharedMemoryBuffer.getBufferSize() < requiredSize
                || sharedMemoryBuffer.getFrameW() != width
                || sharedMemoryBuffer.getFrameH() != height) {
            if (sharedMemoryBuffer != null) {
                sharedMemoryBuffer.close();
            }
            sharedMemoryBuffer = new SharedMemoryBuffer(
                    "MenthaSlamFrameBuffer", requiredSize);
            if (!sharedMemoryBuffer.initHeader(width, height)) {
                sharedMemoryBuffer = null;
                return 0;
            }
            attachFrameBuffer();
            lastSeq = 0;
        }
        if (sharedMemoryBuffer == null) return 0;

        int seq = lastSeq + 1;

        // 背压：目标缓冲(seq模2)必须空闲，即 SLAM 已完成 seq-2
        if (sharedMemoryBuffer.readSlamDoneSeq() < seq - 2) {
            // SLAM 处理落后于发送，丢帧(不覆盖正在处理的缓冲)
            return 0;
        }
        return seq;
    }

    // 持 sendLock：写序号并 oneway 投递
    private void commitFrameLocked(int seq, int bufIndex, int width, int height) {
        sharedMemoryBuffer.writeUiWriteSeq(seq);
        lastSeq = seq;
        try {
            slamService.processFrame(seq, bufIndex, width, height);  // oneway
        } catch (RemoteException e) {
            Log.e(TAG, "processFrame 异常: " + e.getMessage());
        }
    }

    // 将共享内存缓冲的 fd 一次性绑定到 SLAM 进程
    private void attachFrameBuffer() {
        if (slamService == null || sharedMemoryBuffer == null) return;
        ParcelFileDescriptor pfd = sharedMemoryBuffer.getParcelFileDescriptor();
        if (pfd == null) return;
        try {
            slamService.attachFrameBuffer(pfd, sharedMemoryBuffer.getBufferSize());
        } catch (Exception e) {
            Log.e(TAG, "attachFrameBuffer 异常: " + e.getMessage());
        }
    }

    // 读取最终 AR 位姿(48 floats：M[16]+V[16]+P[16])，失败表示暂无有效数据
    public boolean readMvp(float[] out48) {
        return sharedMemoryBuffer != null && sharedMemoryBuffer.readMvp(out48);
    }

    // 是否绘制 AR 物体(已含丢失保持与跨地图判定)
    public boolean readDrawFlag() {
        return sharedMemoryBuffer != null && sharedMemoryBuffer.readDrawFlag() != 0;
    }

    // 读取点云(每点 7 floats)，返回点数，0 表示无点云
    public int readPointCloud(float[] out, int maxFloats) {
        return sharedMemoryBuffer != null ? sharedMemoryBuffer.readPointCloud(out, maxFloats) : 0;
    }

    // 点云序号：服务端每次重写点云区 +1，渲染端只在变化时重新读取与上传
    public int readPointCloudSeq() {
        return sharedMemoryBuffer != null ? sharedMemoryBuffer.readPointCloudSeq() : 0;
    }

    // SLAM 已处理完成的帧序号（每处理完一帧 +1），渲染端用它判断是否有新结果
    public int readSlamDoneSeq() {
        return sharedMemoryBuffer != null ? sharedMemoryBuffer.readSlamDoneSeq() : 0;
    }

    // 姿态辅助开关：融合在 SDK 进程内执行，App 只需下发开关
    public void set3DofAssistEnabled(boolean enabled) {
        if (slamService == null) return;
        try {
            slamService.set3DofAssistEnabled(enabled);
        } catch (RemoteException e) {
            Log.e(TAG, "set3DofAssistEnabled 异常: " + e.getMessage());
        }
    }

    // 反向传感器方向：前后摄像头光学朝向相反时使用
    public void setSensorReverse(boolean reversed) {
        if (slamService == null) return;
        try {
            slamService.setSensorReverse(reversed);
        } catch (RemoteException e) {
            Log.e(TAG, "setSensorReverse 异常: " + e.getMessage());
        }
    }

    // 相机图像相对设备坐标系的旋转角，取值来自 CameraX 的 rotationDegrees
    public void setCameraImageRotation(int degrees) {
        if (slamService == null) return;
        try {
            slamService.setCameraImageRotation(degrees);
        } catch (RemoteException e) {
            Log.e(TAG, "setCameraImageRotation 异常: " + e.getMessage());
        }
    }

    // 姿态传感器可观测自由度：3 含航向，2 仅俯仰与滚转，0 表示不可用
    public int getSensorDof() {
        if (slamService == null) return 0;
        try {
            return slamService.getSensorDof();
        } catch (RemoteException e) {
            Log.e(TAG, "getSensorDof 异常: " + e.getMessage());
            return 0;
        }
    }

    public void detectPlane() {
        if (slamService != null) {
            try {
                slamService.detectPlane();
            } catch (RemoteException e) {
                Log.e(TAG, "detectPlane 异常: " + e.getMessage());
            }
        }
    }

    public void updateResolution(int width, int height) {
        if (slamService != null) {
            try {
                slamService.updateResolution(width, height);
            } catch (RemoteException e) {
                Log.e(TAG, "updateResolution 异常: " + e.getMessage());
            }
        }
    }

    public void setPointCloudDisplay(boolean enable) {
        if (slamService != null) {
            try {
                slamService.setPointCloudDisplay(enable);
            } catch (RemoteException e) {
                Log.e(TAG, "setPointCloudDisplay 异常: " + e.getMessage());
            }
        }
    }

    public boolean isPointCloudDisplayEnabled() {
        if (slamService != null) {
            try {
                return slamService.isPointCloudDisplayEnabled();
            } catch (RemoteException e) {
                Log.e(TAG, "isPointCloudDisplayEnabled 异常: " + e.getMessage());
            }
        }
        return false;
    }

    public void saveMap(String mapPath) {
        if (slamService != null) {
            try {
                slamService.saveMap(mapPath);
            } catch (RemoteException e) {
                Log.e(TAG, "saveMap 异常: " + e.getMessage());
            }
        }
    }

    public void loadMapWithId(String mapPath, int mapId, boolean append) {
        if (slamService != null) {
            try {
                slamService.loadMapWithId(mapPath, mapId, append);
            } catch (RemoteException e) {
                Log.e(TAG, "loadMapWithId 异常: " + e.getMessage());
            }
        }
    }

    public int[] getMapStats() {
        if (slamService != null) {
            try {
                return slamService.getMapStats();
            } catch (RemoteException e) {
                Log.e(TAG, "getMapStats 异常: " + e.getMessage());
            }
        }
        return new int[0];
    }

    public void updateArObjectScale(float scaleFactor) {
        if (slamService != null) {
            try {
                slamService.updateArObjectScale(scaleFactor);
            } catch (RemoteException e) {
                Log.e(TAG, "updateArObjectScale 异常: " + e.getMessage());
            }
        }
    }
}
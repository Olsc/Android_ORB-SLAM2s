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

// SLAM IPC 客户端（UI 进程）。
//
// 每帧路径（性能关键）：
// 1. sendFrameData() 在发送线程调用：写 Y 帧到共享内存双缓冲之一，
//    更新 uiWriteSeq，然后 oneway 投递 processFrame(seq, bufIndex, w, h)，不阻塞。
// 2. SLAM 进程专用线程处理帧，native 把 tracking/draw/MVP/点云/slamDoneSeq 写回共享内存。
// 3. 渲染线程（GL / Filament）通过 readMvp()/readPointCloud() 等直接读共享内存，全程零 binder。
// 4. 平面检测、地图保存/加载等低频控制指令走 binder（非每帧路径）。
//
// 背压：写帧 seq 前要求 slamDoneSeq >= seq-2（目标缓冲已被 SLAM 处理完），
// 否则丢帧，保证双缓冲不被覆盖，同时 SLAM 满负荷运行。
public class SlamIPCClient {
    private static final String TAG = "SlamIPCClient";

    private final Context context;
    private ISlamService slamService;
    private boolean isBound;

    private volatile SharedMemoryBuffer sharedMemoryBuffer;

    private final Object sendLock = new Object();
    private int lastSeq = 0;  // 已投递的帧序号

    private String pendingResDir;

    private final ServiceConnection serviceConnection = new ServiceConnection() {
        @Override
        public void onServiceConnected(ComponentName name, IBinder service) {
            slamService = ISlamService.Stub.asInterface(service);
            isBound = true;
            Log.d(TAG, "成功绑定到 MenthaAR SlamService 进程");
            try {
                if (pendingResDir != null) {
                    Log.d(TAG, "自动执行挂起的 SLAM 初始化: " + pendingResDir);
                    slamService.initSLAM(pendingResDir);
                    pendingResDir = null;
                }
            } catch (RemoteException e) {
                Log.e(TAG, "延迟初始化异常: " + e.getMessage());
            }
        }

        @Override
        public void onServiceDisconnected(ComponentName name) {
            slamService = null;
            isBound = false;
            Log.w(TAG, "MenthaAR SlamService 进程连接断开");
        }
    };

    public SlamIPCClient(Context context) {
        this.context = context;
    }

    public void bindService() {
        Intent intent = new Intent(context, SlamService.class);
        context.bindService(intent, serviceConnection, Context.BIND_AUTO_CREATE);
    }

    public void unbindService() {
        if (isBound && slamService != null) {
            context.unbindService(serviceConnection);
            isBound = false;
        }
        if (sharedMemoryBuffer != null) {
            sharedMemoryBuffer.close();
            sharedMemoryBuffer = null;
        }
    }

    public boolean isConnected() {
        return isBound && slamService != null;
    }

    public void initSLAM(String resDir) {
        this.pendingResDir = resDir;
        if (slamService != null) {
            try {
                slamService.initSLAM(resDir);
                pendingResDir = null;
            } catch (RemoteException e) {
                Log.e(TAG, "initSLAM 异常: " + e.getMessage());
            }
        } else {
            Log.w(TAG, "SlamService 尚未就绪，挂起 SLAM 初始化指令: " + resDir);
        }
    }

    // 将一帧灰度 Y 数据写入共享内存并投递给 SLAM 进程（oneway，不阻塞）
    // 内部双缓冲 + 背压丢帧，可在任意线程调用（内部已串行化）
    public void sendFrameData(byte[] frameData, int width, int height) {
        if (slamService == null || frameData == null) return;
        if (width <= 0 || height <= 0) return;

        synchronized (sendLock) {
            // 尺寸变化或缓冲不足 → 重建共享内存并重新 attach
            int requiredSize = SharedMemoryBuffer.requiredSize(width, height);
            if (sharedMemoryBuffer == null
                    || sharedMemoryBuffer.getBufferSize() < requiredSize
                    || sharedMemoryBuffer.getFrameW() != width
                    || sharedMemoryBuffer.getFrameH() != height) {
                if (sharedMemoryBuffer != null) {
                    sharedMemoryBuffer.close();
                }
                sharedMemoryBuffer = new SharedMemoryBuffer("MenthaSlamFrameBuffer", requiredSize);
                if (!sharedMemoryBuffer.initHeader(width, height)) {
                    sharedMemoryBuffer = null;
                    return;
                }
                attachFrameBuffer();
                lastSeq = 0;
            }
            if (sharedMemoryBuffer == null) return;

            int seq = lastSeq + 1;
            int bufIndex = seq & 1;

            // 背压：目标缓冲（seq%2）必须空闲 —— SLAM 已完成 seq-2
            if (sharedMemoryBuffer.readSlamDoneSeq() < seq - 2) {
                // SLAM 处理落后于发送，丢帧（不覆盖正在处理的缓冲）
                return;
            }

            if (!sharedMemoryBuffer.writeFrame(frameData, bufIndex, width, height)) {
                return;
            }
            sharedMemoryBuffer.writeUiWriteSeq(seq);
            lastSeq = seq;
            try {
                slamService.processFrame(seq, bufIndex, width, height);  // oneway
            } catch (RemoteException e) {
                Log.e(TAG, "processFrame 异常: " + e.getMessage());
            }
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

    // 读取最新 MVP（48 floats：M[16]+V[16]+P[16]），返回 false 表示无有效数据
    public boolean readMvp(float[] out48) {
        return sharedMemoryBuffer != null && sharedMemoryBuffer.readMvp(out48);
    }

    // 是否应绘制 AR 物体
    public boolean readDrawFlag() {
        return sharedMemoryBuffer != null && sharedMemoryBuffer.readDrawFlag() != 0;
    }

    // 读取点云（每点 7 floats），返回点数，0 表示无点云
    public int readPointCloud(float[] out, int maxFloats) {
        return sharedMemoryBuffer != null ? sharedMemoryBuffer.readPointCloud(out, maxFloats) : 0;
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
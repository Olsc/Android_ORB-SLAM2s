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

import android.app.Service;
import android.content.Intent;
import android.os.IBinder;
import android.os.ParcelFileDescriptor;
import android.os.RemoteCallbackList;
import android.os.RemoteException;
import android.util.Log;

import com.orb.slam2s.constant.GlobalConstant;
import com.orb.slam2s.slamar.NativeHelper;

public class SlamService extends Service {
    private static final String TAG = "SlamService";

    private NativeHelper nativeHelper;
    private final RemoteCallbackList<ISlamCallback> callbacks = new RemoteCallbackList<>();

    private final float[] modelMatrix = new float[16];
    private final float[] viewMatrix = new float[16];
    private final float[] projectionMatrix = new float[16];

    // 持久化帧共享内存：仅 attach 一次，之后每帧只传宽高（见 attachFrameBuffer / processFrame）
    private ParcelFileDescriptor framePfd;
    private int frameBufferSize;
    private final Object frameLock = new Object();
    private final int[] frameStatus = new int[2];  // [0]=tracking, [1]=shouldDraw

    private final ISlamService.Stub binder = new ISlamService.Stub() {
        @Override
        public void registerCallback(ISlamCallback callback) {
            if (callback != null) {
                callbacks.register(callback);
            }
        }

        @Override
        public void unregisterCallback(ISlamCallback callback) {
            if (callback != null) {
                callbacks.unregister(callback);
            }
        }

        @Override
        public void initSLAM(String resDir) {
            try {
                Log.d(TAG, "[:slam_process] 初始化 SLAM 资源目录: " + resDir);
                if (nativeHelper == null) {
                    nativeHelper = new NativeHelper(getApplicationContext());
                }
                nativeHelper.initSLAM(resDir);
                notifySLAMInitialized(true, "SLAM 系统初始化完成");
            } catch (Exception e) {
                Log.e(TAG, "[:slam_process] 初始化失败: " + e.getMessage(), e);
                notifySLAMInitialized(false, e.getMessage());
            }
        }

        @Override
        public void attachFrameBuffer(ParcelFileDescriptor pfd, int size) {
            if (nativeHelper == null || pfd == null || size <= 0) return;
            try {
                if (framePfd != null) {
                    try {
                        framePfd.close();
                    } catch (Exception ignored) {}
                }
                framePfd = pfd;
                frameBufferSize = size;
                nativeHelper.attachFrameBuffer(pfd.getFd(), size);
            } catch (Exception e) {
                Log.e(TAG, "attachFrameBuffer 异常: " + e.getMessage());
            }
        }

        @Override
        public void processFrame(int width, int height) {
            if (nativeHelper == null || framePfd == null || width <= 0 || height <= 0) return;
            // 串行化帧处理：CameraGLView 分析线程与 Web 线程都可能调用，防止并发 mmap/写缓冲
            synchronized (frameLock) {
                try {
                    int trackingResult = nativeHelper.processFrameSharedMem(width, height, frameStatus);
                    notifyTrackingStatus(trackingResult);
                    // 逐帧推送最新 MVP（含 View 矩阵），等价于旧架构每帧 nativeGetMVP，
                    // 保证 AR 物体在平面检测后连续跟踪。draw 沿用 native 的
                    // gShouldDrawArObject 语义（跟踪正常 + 平面存在 + 对齐成功）。
                    boolean draw = (frameStatus[1] == 1);
                    nativeHelper.nativeGetMVP(modelMatrix, viewMatrix, projectionMatrix, width, height);
                    notifyMVPUpdated(modelMatrix, viewMatrix, projectionMatrix, draw);
                } catch (Exception e) {
                    Log.e(TAG, "处理共享内存帧异常: " + e.getMessage());
                }
            }
        }

        @Override
        public void detectPlane() {
            if (nativeHelper == null) return;
            int result = nativeHelper.detectPlane();
            if (result == GlobalConstant.PLANE_DETECTED) {
                nativeHelper.nativeGetMVP(modelMatrix, viewMatrix, projectionMatrix,
                        GlobalConstant.RESOLUTION_WIDTH, GlobalConstant.RESOLUTION_HEIGHT);
                notifyMVPUpdated(modelMatrix, viewMatrix, projectionMatrix, true);
            } else {
                notifyMVPUpdated(modelMatrix, viewMatrix, projectionMatrix, false);
            }
            notifyPlaneDetected(result);
        }

        @Override
        public void updateResolution(int width, int height) {
            if (nativeHelper != null) {
                nativeHelper.updateResolution(width, height);
            }
        }

        @Override
        public void setEnableSLAM(boolean enable) {
            if (nativeHelper != null) {
                nativeHelper.setEnableSLAM(enable);
            }
        }

        @Override
        public boolean isEnableSLAM() {
            return nativeHelper != null && nativeHelper.isEnableSLAM();
        }

        @Override
        public void getV(float[] viewMatrix) {
            if (nativeHelper != null && viewMatrix != null && viewMatrix.length == 16) {
                nativeHelper.getV(viewMatrix);
            }
        }

        @Override
        public int getTrackingStatus() {
            return frameStatus[0];
        }

        @Override
        public void setPointCloudDisplay(boolean enable) {
            if (nativeHelper != null) {
                nativeHelper.setPointCloudDisplay(enable);
            }
        }

        @Override
        public boolean isPointCloudDisplayEnabled() {
            return nativeHelper != null && nativeHelper.isPointCloudDisplayEnabled();
        }

        @Override
        public void saveMap(String mapPath) {
            if (nativeHelper != null) {
                nativeHelper.saveMap(mapPath);
            }
        }

        @Override
        public void loadMapWithId(String mapPath, int mapId, boolean append) {
            if (nativeHelper != null) {
                nativeHelper.loadMapWithId(mapPath, mapId, append);
            }
        }

        @Override
        public int getCurrentMapId() {
            return nativeHelper != null ? nativeHelper.getCurrentMapId() : 0;
        }

        @Override
        public int[] getMapStats() {
            return nativeHelper != null ? nativeHelper.getMapStats() : new int[0];
        }

        @Override
        public float[] getTrackedPoints(int maxPoints) {
            return nativeHelper != null ? nativeHelper.getTrackedPoints(maxPoints) : new float[0];
        }

        @Override
        public float[] getMiniMapPoints(int maxPoints) {
            return nativeHelper != null ? nativeHelper.getMiniMapPoints(maxPoints) : new float[0];
        }

        @Override
        public float[] getAllArObjectsData() {
            return nativeHelper != null ? nativeHelper.getAllArObjectsData() : new float[0];
        }

        @Override
        public void updateArObjectScale(float scaleFactor) {
            if (nativeHelper != null) {
                nativeHelper.updateArObjectScale(scaleFactor);
            }
        }

        @Override
        public float getArObjectScale() {
            return nativeHelper != null ? nativeHelper.getArObjectScale() : 1.0f;
        }
    };

    @Override
    public void onCreate() {
        super.onCreate();
        Log.d(TAG, "SlamService 创建 (进程: :slam_process)");
        nativeHelper = new NativeHelper(getApplicationContext());
    }

    @Override
    public IBinder onBind(Intent intent) {
        Log.d(TAG, "SlamService onBind 被绑定");
        return binder;
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        if (nativeHelper != null) {
            nativeHelper.detachFrameBuffer();
        }
        if (framePfd != null) {
            try {
                framePfd.close();
            } catch (Exception ignored) {}
            framePfd = null;
        }
    }

    private void notifyMVPUpdated(float[] M, float[] V, float[] P, boolean draw) {
        int n = callbacks.beginBroadcast();
        for (int i = 0; i < n; i++) {
            try {
                callbacks.getBroadcastItem(i).onMVPUpdated(M, V, P, draw);
            } catch (RemoteException e) {
                Log.e(TAG, "广播 onMVPUpdated 异常: " + e.getMessage());
            }
        }
        callbacks.finishBroadcast();
    }

    private void notifyTrackingStatus(int status) {
        int n = callbacks.beginBroadcast();
        for (int i = 0; i < n; i++) {
            try {
                callbacks.getBroadcastItem(i).onTrackingStatusChanged(status);
            } catch (RemoteException e) {
                Log.e(TAG, "广播 onTrackingStatusChanged 异常: " + e.getMessage());
            }
        }
        callbacks.finishBroadcast();
    }

    private void notifyPlaneDetected(int result) {
        int n = callbacks.beginBroadcast();
        for (int i = 0; i < n; i++) {
            try {
                callbacks.getBroadcastItem(i).onPlaneDetected(result);
            } catch (RemoteException e) {
                Log.e(TAG, "广播 onPlaneDetected 异常: " + e.getMessage());
            }
        }
        callbacks.finishBroadcast();
    }

    private void notifySLAMInitialized(boolean success, String msg) {
        int n = callbacks.beginBroadcast();
        for (int i = 0; i < n; i++) {
            try {
                callbacks.getBroadcastItem(i).onSLAMInitialized(success, msg);
            } catch (RemoteException e) {
                Log.e(TAG, "广播 onSLAMInitialized 异常: " + e.getMessage());
            }
        }
        callbacks.finishBroadcast();
    }
}

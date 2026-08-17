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

import com.orb.slam2s.slamar.NativeHelper;

/**
 * SLAM 独立进程服务。
 *
 * 架构要点：
 * 1. SLAM 处理（TrackMonocular 等耗时操作）运行在专用高优先级线程 {@link #slamThread}，
 *    绝不占用 binder 线程 —— binder 线程池不再被 30fps 的帧处理长期阻塞。
 * 2. 帧投递使用 oneway AIDL（processFrame 只入队立即返回），binder 事务开销降至最低。
 * 3. 处理结果（tracking/draw/MVP/点云）由 native 层直接写回共享内存 header，
 *    每帧仅一次轻量 oneway 调用，无数组回调。
 * 4. initSLAM（加载词汇表约 1 秒）同样投递到处理线程，避免 binder 线程被长时间占用。
 */
public class SlamService extends Service {
    private static final String TAG = "SlamService";

    private NativeHelper nativeHelper;
    private final RemoteCallbackList<ISlamCallback> callbacks = new RemoteCallbackList<>();

    // 专用 SLAM 处理线程
    private Thread slamThread;
    private final Object queueLock = new Object();
    private final java.util.ArrayDeque<Runnable> taskQueue = new java.util.ArrayDeque<>();
    private volatile boolean running = false;

    // 帧共享内存（仅持有 fd，native 层 mmap 后直接读写 header/帧/点云）
    private ParcelFileDescriptor framePfd;
    private final int[] frameStatus = new int[2];  // [0]=tracking, [1]=shouldDraw
    private final java.util.concurrent.atomic.AtomicInteger lastTracking = new java.util.concurrent.atomic.AtomicInteger(0);

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
            // oneway 投递到处理线程执行，耗时加载不阻塞 binder 线程
            postTask(() -> {
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
            });
        }

        @Override
        public boolean attachFrameBuffer(ParcelFileDescriptor pfd, int size) {
            if (nativeHelper == null || pfd == null || size <= 0) return false;
            try {
                if (framePfd != null) {
                    try {
                        framePfd.close();
                    } catch (Exception ignored) {}
                }
                framePfd = pfd;
                return nativeHelper.attachFrameBuffer(pfd.getFd(), size);
            } catch (Exception e) {
                Log.e(TAG, "attachFrameBuffer 异常: " + e.getMessage());
                return false;
            }
        }

        @Override
        public void processFrame(int seq, int bufIndex, int width, int height) {
            if (nativeHelper == null || framePfd == null || width <= 0 || height <= 0) return;
            enqueueFrame(seq, bufIndex, width, height);
        }

        @Override
        public void detectPlane() {
            // 作为任务入队：与帧处理在同一处理线程串行执行（避免并发调用 native）
            postTask(SlamService.this::doDetectPlane);
        }

        @Override
        public void updateResolution(int width, int height) {
            // 并入处理线程：避免 binder 线程与 slamThread 并发修改 native 全局内参
            postTask(() -> {
                if (nativeHelper != null) {
                    nativeHelper.updateResolution(width, height);
                }
            });
        }

        @Override
        public void setEnableSLAM(boolean enable) {
            // 并入处理线程：避免 binder 线程与 slamThread 并发修改 native 全局状态
            postTask(() -> {
                if (nativeHelper != null) {
                    nativeHelper.setEnableSLAM(enable);
                }
            });
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
            return lastTracking.get();
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
    };

    // 处理线程

    private void postTask(Runnable r) {
        synchronized (queueLock) {
            if (!running) return;
            taskQueue.add(r);
            queueLock.notifyAll();
        }
    }

    private void enqueueFrame(int seq, int bufIndex, int w, int h) {
        postTask(() -> processFrameInternal(seq, bufIndex, w, h));
    }

    private void startSlamThread() {
        if (running) return;
        running = true;
        slamThread = new Thread(() -> {
            // 高优先级：SLAM 跟踪对实时性敏感，避免独立进程被调度器降频/降优先
            android.os.Process.setThreadPriority(android.os.Process.THREAD_PRIORITY_URGENT_AUDIO);
            while (running) {
                Runnable r = null;
                synchronized (queueLock) {
                    while (taskQueue.isEmpty() && running) {
                        try {
                            queueLock.wait();
                        } catch (InterruptedException e) {
                            Thread.currentThread().interrupt();
                            return;
                        }
                    }
                    if (!running) break;
                    r = taskQueue.poll();
                }
                try {
                    if (r != null) r.run();
                } catch (Throwable t) {
                    Log.e(TAG, "处理任务异常: " + t.getMessage(), t);
                }
            }
        }, "SlamFrameHandler");
        slamThread.start();
    }

    private void stopSlamThread() {
        running = false;
        synchronized (queueLock) {
            queueLock.notifyAll();
        }
        if (slamThread != null) {
            try {
                slamThread.join(500);
            } catch (InterruptedException ignored) {}
            slamThread = null;
        }
        synchronized (queueLock) {
            taskQueue.clear();
        }
    }

    // 帧处理

    private void processFrameInternal(int seq, int bufIndex, int w, int h) {
        try {
            // native 处理共享内存中的帧，并把 tracking/draw/MVP/点云/slamDoneSeq 写回共享内存
            nativeHelper.processFrameSharedMem(bufIndex, seq, w, h, frameStatus);
        } catch (Exception e) {
            Log.e(TAG, "处理共享内存帧异常: " + e.getMessage());
        }
        lastTracking.set(frameStatus[0]);
    }

    private void doDetectPlane() {
        if (nativeHelper == null) return;
        try {
            int result = nativeHelper.detectPlane();
            // MVP 由 native 写回共享内存，UI 渲染线程直接读取，无需 binder 推送
            notifyPlaneDetected(result);
        } catch (Exception e) {
            Log.e(TAG, "detectPlane 异常: " + e.getMessage());
        }
    }

    // 生命周期

    @Override
    public void onCreate() {
        super.onCreate();
        Log.d(TAG, "SlamService 创建 (进程: :slam_process)");
        nativeHelper = new NativeHelper(getApplicationContext());
        startSlamThread();
    }

    @Override
    public IBinder onBind(Intent intent) {
        Log.d(TAG, "SlamService onBind 被绑定");
        return binder;
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        Log.d(TAG, "SlamService onDestroy: 停止处理线程并释放 SLAM 系统");
        stopSlamThread();
        // 服务进程销毁时依次：停帧处理 → shutdownSLAM（join 工作线程）→ detach 共享内存，
        // 确保三条常驻线程（LM/LC/GlobalReloc）全部释放，避免内存泄漏与 mmap 残留
        if (nativeHelper != null) {
            nativeHelper.shutdownSLAM();
            nativeHelper.detachFrameBuffer();
        }
        if (framePfd != null) {
            try {
                framePfd.close();
            } catch (Exception ignored) {}
            framePfd = null;
        }
    }

    // 回调

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
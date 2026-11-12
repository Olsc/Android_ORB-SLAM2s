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
import android.util.Log;

import com.menthaar.sdk.internal.ISlamService;
import com.orb.slam2s.sensors.OrientationSensor;
import com.orb.slam2s.sensors.PoseFusion;
import com.orb.slam2s.slamar.NativeHelper;

// SLAM 独立后台服务：在独立高优先级线程中执行单目特征提取与位姿跟踪
// 姿态辅助融合也在本进程完成，App 只通过共享内存读取最终 AR 位姿
@SuppressWarnings("unused")
public class SlamService extends Service {
    private static final String TAG = "SlamService";

    private NativeHelper nativeHelper;

    // 姿态辅助：传感器读取与位姿融合均在 SDK 进程内完成
    private final PoseFusion poseFusion = new PoseFusion();
    private OrientationSensor orientationSensor;
    private volatile boolean assistEnabled;
    private final float[] rawPose = new float[48];
    private final int[] poseInfo = new int[3];
    private final float[] sensorRotation = new float[16];
    private final float[] fusedModel = new float[16];
    private final float[] fusedView = new float[16];
    private final float[] fusedProj = new float[16];

    // 专用 SLAM 处理线程
    private Thread slamThread;
    private final Object queueLock = new Object();
    private final java.util.ArrayDeque<Runnable> taskQueue = new java.util.ArrayDeque<>();
    private volatile boolean running = false;

    // 帧共享内存(仅持有 fd，native 层 mmap 后直接读写 header/帧/点云)
    private ParcelFileDescriptor framePfd;

    private final ISlamService.Stub binder = new ISlamService.Stub() {
        @Override
        public void initSLAM(String resDir) {
            // oneway 投递到处理线程执行，耗时加载不阻塞 binder 线程
            postTask(() -> {
                try {
                    Log.d(TAG, "[:slam_process] 初始化 SLAM 资源目录: " + resDir);
                    if (nativeHelper == null) {
                        nativeHelper = new NativeHelper();
                    }
                    nativeHelper.initSLAM(resDir);
                } catch (Exception e) {
                    Log.e(TAG, "[:slam_process] 初始化失败: " + e.getMessage(), e);
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
            // 入队到处理线程，与帧处理串行执行（native 调用不可并发）。
            postTask(SlamService.this::doDetectPlane);
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
        public void saveMap(final String mapPath) {
            // 与帧处理串行执行，SaveMap 会遍历全部 KF/MP 并可能补齐描述子，不可与跟踪并发。
            postTask(() -> {
                if (nativeHelper != null) {
                    nativeHelper.saveMap(mapPath);
                }
            });
        }

        @Override
        public void loadMapWithId(final String mapPath, final int mapId, final boolean append) {
            // 与帧处理串行执行，加载会重建当前地图；native 侧 gProcessingFrames 提供第二重互斥。
            postTask(() -> {
                if (nativeHelper != null) {
                    nativeHelper.loadMapWithId(mapPath, mapId, append);
                }
            });
        }

        @Override
        public int[] getMapStats() {
            // GetMapStats 会遍历关键帧与地图点，不能与 slamThread 并发，也不能在 shutdown 期间执行。
            // 这里返回上次快照并在处理线程异步刷新，1 Hz 轮询下次取到新值。
            requestMapStatsRefresh();
            synchronized (statsLock) {
                return cachedMapStats.clone();
            }
        }

        @Override
        public void updateArObjectScale(float scaleFactor) {
            if (nativeHelper != null) {
                nativeHelper.updateArObjectScale(scaleFactor);
            }
        }

        @Override
        public void set3DofAssistEnabled(boolean enable) {
            postTask(() -> apply3DofEnabled(enable));
        }

        @Override
        public void setSensorReverse(boolean reversed) {
            poseFusion.setSensorReverse(reversed);
        }

        @Override
        public void setCameraImageRotation(int degrees) {
            poseFusion.setCameraImageRotation(degrees);
        }

        @Override
        public int getSensorDof() {
            final OrientationSensor sensor = ensureSensor();
            return sensor != null ? sensor.getRotationDof() : 0;
        }
    };

    // 地图规模快照：只在 SLAM 处理线程上写，binder 线程只读
    private final Object statsLock = new Object();
    private final int[] cachedMapStats = new int[3];
    private boolean statsRefreshPending;

    private void requestMapStatsRefresh() {
        synchronized (statsLock) {
            if (statsRefreshPending) return;
            statsRefreshPending = true;
        }
        final boolean queued = postTask(() -> {
            try {
                final int[] stats = nativeHelper != null ? nativeHelper.getMapStats() : null;
                if (stats != null && stats.length >= 3) {
                    synchronized (statsLock) {
                        System.arraycopy(stats, 0, cachedMapStats, 0, 3);
                    }
                }
            } catch (Throwable t) {
                Log.w(TAG, "刷新地图统计失败: " + t.getMessage());
            } finally {
                synchronized (statsLock) {
                    statsRefreshPending = false;
                }
            }
        });
        if (!queued) {
            synchronized (statsLock) {
                statsRefreshPending = false;
            }
        }
    }

    // 开启与关闭姿态辅助：开关传感器监听并同步融合器状态
    private void apply3DofEnabled(boolean enabled) {
        // 幂等：重连/重复下发配置时不重复重启传感器（重启会重置姿态基准）
        if (assistEnabled == enabled) return;

        if (enabled) {
            final OrientationSensor sensor = ensureSensor();
            if (sensor == null) {
                Log.w(TAG, "无可用姿态传感器，姿态辅助不可用");
                poseFusion.setEnabled(false);
                return;
            }
            assistEnabled = true;
            poseFusion.setEnabled(true);
            sensor.start(this);
            // 传感器重启会重置姿态基准，旧冻结偏移必须丢弃，否则首帧会大幅偏移
            poseFusion.resetAlignment();
        } else {
            assistEnabled = false;
            poseFusion.setEnabled(false);
            if (orientationSensor != null) {
                orientationSensor.stop();
            }
        }
    }

    // 懒创建传感器，首次调用时探测可用姿态来源；binder 线程与处理线程都可能进来，必须串行化
    private synchronized OrientationSensor ensureSensor() {
        if (orientationSensor == null) {
            orientationSensor = new OrientationSensor();
            final OrientationSensor.Mode mode = orientationSensor.detectMode(this);
            Log.i(TAG, "姿态来源: " + mode + "，自由度: " + orientationSensor.getRotationDof());
        }
        return orientationSensor.getRotationDof() > 0 ? orientationSensor : null;
    }

    private boolean postTask(Runnable r) {
        synchronized (queueLock) {
            if (!running) return false;
            taskQueue.add(r);
            queueLock.notifyAll();
            return true;
        }
    }

    private void enqueueFrame(int seq, int bufIndex, int w, int h) {
        postTask(() -> processFrameInternal(seq, bufIndex, w, h));
    }

    private void startSlamThread() {
        if (running) return;
        running = true;
        slamThread = new Thread(() -> {
            // 提高线程优先级，降低独立进程被调度器降频的概率。
            android.os.Process.setThreadPriority(android.os.Process.THREAD_PRIORITY_URGENT_AUDIO);
            while (running) {
                Runnable r;
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
            // 必须等处理线程真正退出：随后的 shutdownSLAM/detachFrameBuffer 会释放它正在使用的引擎与映射。
            // 单帧处理是有界的（最多一帧跟踪 + 结果写回），不会无限等待
            boolean interrupted = false;
            while (true) {
                try {
                    slamThread.join();
                    break;
                } catch (InterruptedException e) {
                    interrupted = true;
                }
            }
            if (interrupted) Thread.currentThread().interrupt();
            slamThread = null;
        }
        synchronized (queueLock) {
            taskQueue.clear();
        }
    }

    private void processFrameInternal(int seq, int bufIndex, int w, int h) {
        boolean processed = false;
        try {
            // native 处理共享内存中的帧，把 tracking/点云写回共享内存、原始位姿写进共享缓冲；
            // MVP 与 slamDoneSeq 由下面的发布一次性写出（App 端只会看到最终结果，不会先看到原始位姿再看到融合位姿）
            nativeHelper.processFrameSharedMem(bufIndex, seq, w, h);
            processed = true;
            publishArPose();
        } catch (Exception e) {
            Log.e(TAG, "处理共享内存帧异常: " + e.getMessage());
            // 发布失败时仍要推进序号，否则 App 端背压会在两帧后永久停发
            if (processed) {
                try { nativeHelper.publishRawArPose(); } catch (Exception ignored) {}
            }
        }
    }

    // 读取原始位姿并融合后发布；融合关闭或不可用时直接发布原始位姿
    private void publishArPose() {
        if (!poseFusion.isEnabled() || nativeHelper == null || !nativeHelper.getArPose(rawPose, poseInfo)) {
            if (nativeHelper != null) nativeHelper.publishRawArPose();
            return;
        }
        final boolean sensorReady = orientationSensor != null && orientationSensor.isReady();
        if (sensorReady) {
            // 保持态只需相对最后一帧视觉的旋转，取陀螺积分加重力校倾的相对姿态，不含磁力计航向。
            if (!orientationSensor.copyRelativeRotationMatrix(sensorRotation)) {
                orientationSensor.copyRotationMatrix(sensorRotation);
            }
        }
        final boolean draw = poseFusion.fuse(poseInfo[1] != 0, poseInfo[0], poseInfo[2], rawPose,
                sensorReady ? sensorRotation : null, sensorReady,
                fusedModel, fusedView, fusedProj);
        nativeHelper.publishArPose(fusedModel, fusedView, fusedProj, draw);
    }

    private void doDetectPlane() {
        if (nativeHelper == null) return;
        try {
            // MVP 由 native 写回共享内存，UI 渲染线程直接读取，无需 binder 推送
            nativeHelper.detectPlane();
        } catch (Exception e) {
            Log.e(TAG, "detectPlane 异常: " + e.getMessage());
        }
    }

    @Override
    public void onCreate() {
        super.onCreate();
        Log.d(TAG, "SlamService 创建 (进程: :slam_process)");
        nativeHelper = new NativeHelper();
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
        if (orientationSensor != null) {
            orientationSensor.stop();
            orientationSensor = null;
        }
        // 服务销毁依次执行停帧处理、shutdownSLAM、detach 共享内存，确保常驻线程全部释放。
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
}
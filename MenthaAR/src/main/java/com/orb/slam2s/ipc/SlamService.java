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

import com.orb.slam2s.sensors.OrientationSensor;
import com.orb.slam2s.sensors.PoseFusion;
import com.orb.slam2s.slamar.NativeHelper;

// SLAM 独立后台服务：在独立高优先级线程中执行单目特征提取与位姿跟踪
// 姿态辅助融合也在本进程完成，App 只通过共享内存读取最终 AR 位姿
public class SlamService extends Service {
    private static final String TAG = "SlamService";

    private NativeHelper nativeHelper;

    // 姿态辅助：传感器读取与位姿融合均在 SDK 进程内完成
    private final PoseFusion poseFusion = new PoseFusion();
    private OrientationSensor orientationSensor;
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
    private final int[] frameStatus = new int[2];  // [0]=tracking, [1]=shouldDraw

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
            // 作为任务入队：与帧处理在同一处理线程串行执行(避免并发调用 native)
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
            // 与帧处理同一处理线程串行执行：避免保存过程中与帧/建图线程并发遍历并改写地图(native 侧 SaveMap 会读取全部 KF/MP，并可能补齐描述子)
            postTask(() -> {
                if (nativeHelper != null) {
                    nativeHelper.saveMap(mapPath);
                }
            });
        }

        @Override
        public void loadMapWithId(final String mapPath, final int mapId, final boolean append) {
            // 与帧处理同一处理线程串行执行：加载会重建当前地图内容，必须与帧处理互斥(native 侧另有 gProcessingFrames 等待作为双重保障)
            postTask(() -> {
                if (nativeHelper != null) {
                    nativeHelper.loadMapWithId(mapPath, mapId, append);
                }
            });
        }

        @Override
        public int[] getMapStats() {
            return nativeHelper != null ? nativeHelper.getMapStats() : new int[0];
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

    // 开启与关闭姿态辅助：开关传感器监听并同步融合器状态
    private void apply3DofEnabled(boolean enabled) {
        poseFusion.setEnabled(enabled);
        if (enabled) {
            final OrientationSensor sensor = ensureSensor();
            if (sensor == null) {
                Log.w(TAG, "无可用姿态传感器，姿态辅助不可用");
                return;
            }
            sensor.start(this);
            // 传感器重启会重置姿态基准，旧冻结偏移必须丢弃，否则首帧会大幅偏移
            poseFusion.resetAlignment();
        } else if (orientationSensor != null) {
            orientationSensor.stop();
        }
    }

    // 懒创建传感器，首次调用时探测可用姿态来源
    private OrientationSensor ensureSensor() {
        if (orientationSensor == null) {
            orientationSensor = new OrientationSensor();
            final OrientationSensor.Mode mode = orientationSensor.detectMode(this);
            Log.i(TAG, "姿态来源: " + mode + "，自由度: " + orientationSensor.getRotationDof());
        }
        return orientationSensor.getRotationDof() > 0 ? orientationSensor : null;
    }

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
            try {
                slamThread.join(500);
            } catch (InterruptedException ignored) {}
            slamThread = null;
        }
        synchronized (queueLock) {
            taskQueue.clear();
        }
    }

    private void processFrameInternal(int seq, int bufIndex, int w, int h) {
        try {
            // native 处理共享内存中的帧，并把 tracking/draw/MVP/点云/slamDoneSeq 写回共享内存
            nativeHelper.processFrameSharedMem(bufIndex, seq, w, h, frameStatus);
            publishArPose();
        } catch (Exception e) {
            Log.e(TAG, "处理共享内存帧异常: " + e.getMessage());
        }
    }

    // 读取原始位姿并融合后回写，App 端只会看到最终结果
    private void publishArPose() {
        if (!poseFusion.isEnabled() || nativeHelper == null) {
            return;
        }
        if (!nativeHelper.getArPose(rawPose, poseInfo)) {
            return;
        }
        final boolean sensorReady = orientationSensor != null && orientationSensor.isReady();
        if (sensorReady) {
            // 保持态只需要相对最后一次视觉帧的旋转，优先用陀螺积分 + 重力校倾的相对姿态：
            // 该来源不含磁力计航向，避免磁干扰或航向偏差导致方向漂移甚至翻转
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
        // 服务进程销毁时依次执行：停帧处理、shutdownSLAM(join 工作线程)、detach 共享内存，
        // 确保三条常驻线程(LM/LC/GlobalReloc)全部释放，避免内存泄漏与 mmap 残留
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
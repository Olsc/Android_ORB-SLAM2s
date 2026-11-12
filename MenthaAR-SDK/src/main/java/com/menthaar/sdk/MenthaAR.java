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

import android.content.Context;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.view.Choreographer;

import java.io.File;
import java.nio.ByteBuffer;
import java.util.List;

// MenthaAR 空间定位 SDK 入口：生命周期、送帧、结果读取与地图操作。
public final class MenthaAR {
    private static final String TAG = "MenthaAR";

    // 点云中每个点的 float 个数：x, y, z, r, g, b, size。
    public static final int POINT_CLOUD_FLOATS_PER_POINT = 7;

    // SDK 事件回调，均在主线程执行；按需覆写。
    public interface Listener {
        // 服务已连接、引擎初始化指令已下发（每次重连都会回调）。
        default void onReady() {}

        // 服务进程断开（SDK 会在下次 start 时重新连接）。
        default void onDisconnected() {}

        // SLAM 产出新结果，可调用 readResult 读取；用于触发重绘。
        default void onTrackingResult() {}
    }

    // 可写灰度槽位：写满 gray 后调用 submitFrame(Frame) 提交。
    public static final class Frame {
        // 灰度写入目标：direct 缓冲，position=0，limit=width*height，每个字节一个像素。
        public final ByteBuffer gray;
        public final int width;
        public final int height;

        // 内部字段：帧序号与槽位号，用于提交时校验帧是否仍然有效
        final int seq;
        final int bufferIndex;
        final boolean needsCopy;

        Frame(ByteBuffer gray, int width, int height, int seq, int bufferIndex, boolean needsCopy) {
            this.gray = gray;
            this.width = width;
            this.height = height;
            this.seq = seq;
            this.bufferIndex = bufferIndex;
            this.needsCopy = needsCopy;
        }
    }

    // 追踪结果快照，位姿与状态取自同一次发布；复用实例可做到每帧零分配。
    public static final class Result {
        public static final int TRACKING_NOT_READY = -1;
        public static final int TRACKING_NO_IMAGE = 0;
        public static final int TRACKING_INITIALIZING = 1;
        public static final int TRACKING_OK = 2;
        public static final int TRACKING_LOST = 3;

        // 模型矩阵（列主序，4x4）。
        public final float[] model = new float[16];
        // 视图矩阵（列主序，4x4）。
        public final float[] view = new float[16];
        // 投影矩阵（列主序，4x4，已包含显示分辨率与相机内参）。
        public final float[] projection = new float[16];

        // 当前是否应该绘制 AR 内容（已包含跟踪丢失保持与跨地图判定）。
        public boolean drawable;
        // 跟踪状态，取值 TRACKING_*。
        public int trackingState;
        // 当前激活的地图 ID。
        public int mapId;
        // SLAM 已完成的最新帧序号，单调递增；序号在矩阵发布之后才推进，可能领先快照一帧。
        public int frameId;
        // 点云版本号，变化时才需要重新 readPointCloud 读取；点云区无 seqlock，可能与矩阵相差一帧。
        public int pointCloudId;

        // 内部复用缓冲，读取结果时不再分配临时数组
        final float[] rawMatrices = new float[48];
        final int[] rawStatus = new int[SharedMemoryBuffer.STATUS_LENGTH];

        // 是否处于正常跟踪状态。
        public boolean isTracked() {
            return trackingState == TRACKING_OK;
        }
    }

    // 当前地图规模统计。
    public static final class MapStats {
        public final int keyFrames;
        public final int mapPoints;
        public final boolean hasPlane;

        MapStats(int keyFrames, int mapPoints, boolean hasPlane) {
            this.keyFrames = keyFrames;
            this.mapPoints = mapPoints;
            this.hasPlane = hasPlane;
        }
    }

    private final Context appContext;
    private final Session session;
    private final Handler mainHandler = new Handler(Looper.getMainLooper());

    private volatile Listener listener;

    // 配置缓存：连接（含重连）后由 applySettings 统一重新下发，调用方无需关心重连
    private volatile int cameraRotation;
    private volatile boolean poseAssistEnabled;
    private volatile boolean poseAssistReversed;
    private volatile boolean pointCloudVisible = true;

    private final Object lifecycleLock = new Object();
    private volatile boolean running;

    // 结果轮询：每个显示帧读一次 4 字节完成序号，变化时回调 onTrackingResult
    private volatile boolean polling;
    private int lastFrameId = -1;
    // start/stop 快速交替时用代次号作废尚未执行的启动任务，stop 后不会残留回调链
    private final java.util.concurrent.atomic.AtomicInteger pollGeneration =
            new java.util.concurrent.atomic.AtomicInteger();
    private final Choreographer.FrameCallback frameCallback = new Choreographer.FrameCallback() {
        @Override
        public void doFrame(long frameTimeNanos) {
            if (!polling) return;
            final int frameId = session.readSlamDoneSeq();
            if (frameId != lastFrameId) {
                lastFrameId = frameId;
                final Listener l = listener;
                if (l != null) l.onTrackingResult();
            }
            if (polling) {
                Choreographer.getInstance().postFrameCallback(this);
            }
        }
    };

    // 引擎资源目录；null 表示使用 SDK 自己的默认目录（<externalFilesDir>/MenthaAR）
    private volatile File resourceDirectory;

    // 仅持有 application context
    public MenthaAR(Context context) {
        if (context == null) {
            throw new IllegalArgumentException("context 不能为 null");
        }
        appContext = context.getApplicationContext();
        session = new Session(appContext);
        session.setObserver(new Session.Observer() {
            @Override
            public void onConnected() {
                // 先下发配置再放行送帧：保证服务端处理的第一帧就带着调用方的开关/旋转角
                applySettings();
                session.setFramesAllowed(true);
                startResultPolling();
                final Listener l = listener;
                if (l != null) l.onReady();
            }

            @Override
            public void onDisconnected() {
                session.setFramesAllowed(false);
                stopResultPolling();
                final Listener l = listener;
                if (l != null) l.onDisconnected();
            }
        });
    }

    // 启动定位：绑定服务、初始化引擎并下发配置；异步完成，就绪回调 onReady。
    public void start() {
        synchronized (lifecycleLock) {
            if (running) {
                // 上次绑定失败（服务未启动）时允许重试；已连接或正在绑定则直接返回
                if (session.isConnected() || session.isBinding()) return;
            }
            running = true;
        }
        session.bind(workingDirectory().getAbsolutePath());
    }

    // 停止定位：解绑服务并释放共享内存，之后可再次调用 start。
    public void stop() {
        synchronized (lifecycleLock) {
            if (!running) return;
            running = false;
        }
        stopResultPolling();
        session.unbind();
    }

    // 是否已启动；引擎是否就绪以 Listener.onReady 回调为准。
    public boolean isRunning() {
        return running;
    }

    // 服务是否已连接；引擎是否就绪仍以 Listener.onReady 回调为准。
    public boolean isConnected() {
        return session.isConnected();
    }

    // 设置事件回调；在主线程回调。传入 null 可取消。
    public void setListener(Listener listener) {
        this.listener = listener;
    }

    // 提交一帧灰度图，长度至少 width*height；false 表示未就绪或背压丢帧。
    public boolean submitFrame(byte[] gray, int width, int height) {
        return session.submitFrame(gray, width, height);
    }

    // 申请可写帧槽位实现零拷贝；返回 null 表示本帧未被接收，可跳过灰度转换。
    public Frame beginFrame(int width, int height) {
        return session.acquireFrame(width, height);
    }

    // 提交 beginFrame 得到的帧。
    public boolean submitFrame(Frame frame) {
        return session.submitFrame(frame);
    }

    // 读取结果快照并返回新对象；高频路径请改用 readResult(Result)。
    public Result readResult() {
        final Result result = new Result();
        return readResult(result) ? result : null;
    }

    // 读取结果到可复用实例（零分配）；false 表示暂无数据，out 保持不变。
    public boolean readResult(Result out) {
        if (out == null) return false;
        if (!session.readResult(out.rawMatrices, out.rawStatus)) return false;

        System.arraycopy(out.rawMatrices, 0, out.model, 0, 16);
        System.arraycopy(out.rawMatrices, 16, out.view, 0, 16);
        System.arraycopy(out.rawMatrices, 32, out.projection, 0, 16);
        out.drawable = out.rawStatus[SharedMemoryBuffer.STATUS_DRAWABLE] != 0;
        out.trackingState = out.rawStatus[SharedMemoryBuffer.STATUS_TRACKING];
        out.mapId = out.rawStatus[SharedMemoryBuffer.STATUS_MAP_ID];
        out.frameId = out.rawStatus[SharedMemoryBuffer.STATUS_FRAME_ID];
        out.pointCloudId = out.rawStatus[SharedMemoryBuffer.STATUS_POINTCLOUD_SEQ];
        return true;
    }

    // 读取点云，每点 7 个 float：x, y, z, r, g, b, size；返回点数。
    public int readPointCloud(float[] out) {
        if (out == null) return 0;
        return session.readPointCloud(out, out.length) / POINT_CLOUD_FLOATS_PER_POINT;
    }

    // 点云缓冲建议长度（float 个数，= 最大点云点数 * 7）。
    public static int pointCloudCapacity() {
        return SharedMemoryBuffer.pointCloudCapacity();
    }

    // 设置相机图像相对设备坐标系的旋转角，取值 0/90/180/270。
    public void setCameraRotation(int degrees) {
        final int normalized = ((degrees % 360) + 360) % 360;
        cameraRotation = normalized;
        session.setCameraRotation(normalized);
    }

    // 姿态辅助开关，默认关闭；无可用姿态传感器时不生效。
    public void setPoseAssistEnabled(boolean enabled) {
        poseAssistEnabled = enabled;
        session.setPoseAssistEnabled(enabled);
    }

    // 反向姿态传感器方向，用于前后摄像头光学朝向与传感器相反的场景。
    public void setPoseAssistReversed(boolean reversed) {
        poseAssistReversed = reversed;
        session.setPoseAssistReversed(reversed);
    }

    // 当前设备是否具备姿态辅助能力（需要陀螺仪，能提供至少俯仰/滚转）。
    public boolean isPoseAssistAvailable() {
        return session.getPoseAssistDof() > 0;
    }

    // 请求一次平面检测，结果通过后续 readResult 读取。
    public void detectPlane() {
        session.detectPlane();
    }

    // 是否输出点云（关闭后不再产生点云数据），默认开启，与服务端默认值一致。
    public void setPointCloudVisible(boolean visible) {
        pointCloudVisible = visible;
        session.setPointCloudVisible(visible);
    }

    // 点云输出是否开启（返回 SDK 记住的配置；连接/重连时已自动同步到服务端）。
    public boolean isPointCloudVisible() {
        return pointCloudVisible;
    }

    // 调整 AR 物体缩放：参数为手势缩放步进，1.0 表示不变。
    public void setArObjectScale(float scaleStep) {
        session.setArObjectScale(scaleStep);
    }

    // 保存当前地图到指定文件（含配套的 .arinfo）；父目录不存在时自动创建。
    public void saveMap(String path) {
        if (path == null) return;
        final File parent = new File(path).getParentFile();
        if (parent != null && !parent.exists() && !parent.mkdirs()) {
            Log.w(TAG, "无法创建地图目录: " + parent.getAbsolutePath());
        }
        session.saveMap(path);
    }

    // 加载一张地图并替换当前地图。
    public void loadMap(String path) {
        session.loadMap(path, 0, false);
    }

    // 合并加载多张地图：第一张替换当前地图，其余追加，地图 ID 由 SDK 分配。
    public void loadMaps(List<String> paths) {
        if (paths == null || paths.isEmpty()) return;
        int mapId = 0;
        for (String path : paths) {
            if (path == null) continue;
            session.loadMap(path, mapId, mapId > 0);
            mapId++;
        }
    }

    // 当前地图规模统计。未就绪时返回全 0。
    public MapStats getMapStats() {
        final int[] raw = session.getMapStats();
        if (raw == null || raw.length < 3) {
            return new MapStats(0, 0, false);
        }
        return new MapStats(raw[0], raw[1], raw[2] != 0);
    }

    // 连接（含重连）后统一下发缓存配置；在 onConnected（主线程）内调用，均为常数级 Binder 调用。
    private void applySettings() {
        session.setCameraRotation(cameraRotation);
        session.setPoseAssistReversed(poseAssistReversed);
        session.setPoseAssistEnabled(poseAssistEnabled);
        session.setPointCloudVisible(pointCloudVisible);
    }

    private void startResultPolling() {
        final int generation = pollGeneration.incrementAndGet();
        mainHandler.post(() -> {
            if (generation != pollGeneration.get()) return;
            if (polling) return;
            polling = true;
            lastFrameId = -1;
            Choreographer.getInstance().postFrameCallback(frameCallback);
        });
    }

    private void stopResultPolling() {
        // 立即生效（不依赖主线程队列）：doFrame 不会再重排自己
        pollGeneration.incrementAndGet();
        polling = false;
        mainHandler.post(() -> Choreographer.getInstance().removeFrameCallback(frameCallback));
    }

    // 引擎资源目录（如 <externalFilesDir>/SLAM）；未设置时用 SDK 默认目录，需在 start() 前调用。
    public void setResourceDirectory(File directory) {
        this.resourceDirectory = directory;
    }

    private File workingDirectory() {
        File dir = resourceDirectory;
        if (dir == null) {
            dir = appContext.getExternalFilesDir("MenthaAR");
            if (dir == null) {
                dir = new File(appContext.getFilesDir(), "MenthaAR");
            }
        }
        if (!dir.exists() && !dir.mkdirs()) {
            Log.w(TAG, "无法创建 SDK 工作目录: " + dir.getAbsolutePath());
        }
        return dir;
    }
}
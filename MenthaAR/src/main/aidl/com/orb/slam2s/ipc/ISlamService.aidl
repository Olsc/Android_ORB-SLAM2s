/*
 * Copyright (C) 2026 Olsc <OlscStudio@outlook.com>
 *
 * This file is part of the Android ORB-SLAM2s IPC Interface.
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

import android.os.ParcelFileDescriptor;

// App 与 SLAM 服务进程之间的控制通道，帧数据与结果走共享内存。
// 服务可能位于另一个 APK（com.mentha.ar），不能共享文件路径：资源目录由服务自管，地图经 PFD 传输。
interface ISlamService {
    // 低频耗时操作，oneway 调用不占用调用方 binder 线程；资源目录由服务端自管
    oneway void initSLAM();

    // 共享内存 attach：低频（仅创建缓冲/分辨率变化时），保持同步以便确认映射成功
    boolean attachFrameBuffer(in ParcelFileDescriptor pfd, int size);

    // 每帧路径：oneway 投递帧序号，SLAM 进程内部专用线程处理，binder 线程不被占用
    oneway void processFrame(int seq, int bufIndex, int width, int height);

    oneway void detectPlane();
    void setPointCloudDisplay(boolean enable);
    boolean isPointCloudDisplayEnabled();
    // 地图经文件描述符交换：mapOut 接收地图主文件，arInfoOut 接收配套 .arinfo；服务端写完后关闭
    oneway void saveMap(in ParcelFileDescriptor mapOut, in ParcelFileDescriptor arInfoOut);
    // arInfoIn 可为 null（旧地图没有 .arinfo）；服务端读完后关闭
    oneway void loadMapWithId(in ParcelFileDescriptor mapIn, in ParcelFileDescriptor arInfoIn, int mapId, boolean append);
    int[] getMapStats();
    void updateArObjectScale(float scaleFactor);

    // 姿态辅助（3DOF）开关与方向反向，由 App 作为配置下发
    void set3DofAssistEnabled(boolean enable);
    void setSensorReverse(boolean reversed);
    oneway void setCameraImageRotation(int degrees);

    // 姿态传感器可观测自由度：3 含航向，2 仅俯仰与滚转，0 表示不可用
    int getSensorDof();
}
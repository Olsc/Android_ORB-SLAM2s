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

import com.orb.slam2s.ipc.ISlamCallback;
import android.os.ParcelFileDescriptor;

interface ISlamService {
    void registerCallback(ISlamCallback callback);
    void unregisterCallback(ISlamCallback callback);

    // 低频/耗时操作：oneway 避免占用 binder 线程（initSLAM 需加载词汇表约 1 秒）
    oneway void initSLAM(String resDir);

    // 共享内存 attach：低频（仅缓冲创建/尺寸变化时），保持同步以便确认成功
    boolean attachFrameBuffer(in ParcelFileDescriptor pfd, int size);

    // 每帧路径：oneway 投递帧序号，SLAM 进程内部专用线程处理，binder 线程不被占用
    oneway void processFrame(int seq, int bufIndex, int width, int height);

    oneway void detectPlane();
    void updateResolution(int width, int height);
    oneway void setEnableSLAM(boolean enable);
    boolean isEnableSLAM();
    void getV(inout float[] viewMatrix);
    int getTrackingStatus();
    void setPointCloudDisplay(boolean enable);
    boolean isPointCloudDisplayEnabled();
    void saveMap(String mapPath);
    void loadMapWithId(String mapPath, int mapId, boolean append);
    int[] getMapStats();
    float[] getTrackedPoints(int maxPoints);
    float[] getMiniMapPoints(int maxPoints);
    float[] getAllArObjectsData();
    void updateArObjectScale(float scaleFactor);
}

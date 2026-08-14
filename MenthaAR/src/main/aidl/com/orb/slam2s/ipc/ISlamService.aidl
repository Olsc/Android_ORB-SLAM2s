/* Copyright (C) 2026 Olsc <OlscStudio@outlook.com>
 *
 * This file is part of the Android ORB-SLAM2s project (a fork of ORB-SLAM2).
 * Licensed under the GNU General Public License v3.0 (or later).
 * See <https://www.gnu.org/licenses/> for the full license text.
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

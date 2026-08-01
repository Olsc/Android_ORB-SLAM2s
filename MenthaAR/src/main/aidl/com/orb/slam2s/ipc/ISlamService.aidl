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

    void initSLAM(String resDir);
    void attachFrameBuffer(in ParcelFileDescriptor pfd, int size);
    void processFrame(int width, int height);
    void detectPlane();
    void updateResolution(int width, int height);
    void setEnableSLAM(boolean enable);
    boolean isEnableSLAM();
    void getV(inout float[] viewMatrix);
    int getTrackingStatus();
    void setPointCloudDisplay(boolean enable);
    boolean isPointCloudDisplayEnabled();
    void saveMap(String mapPath);
    void loadMapWithId(String mapPath, int mapId, boolean append);
    int getCurrentMapId();
    int[] getMapStats();
    float[] getTrackedPoints(int maxPoints);
    float[] getMiniMapPoints(int maxPoints);
    float[] getAllArObjectsData();
    void updateArObjectScale(float scaleFactor);
    float getArObjectScale();
}

package com.orb.slam2s.ipc;

import com.orb.slam2s.ipc.ISlamCallback;
import android.os.ParcelFileDescriptor;

interface ISlamService {
    void registerCallback(ISlamCallback callback);
    void unregisterCallback(ISlamCallback callback);

    void initSLAM(String resDir);
    void processFrameSharedMem(in ParcelFileDescriptor pfd, int size, int width, int height);
    void detectPlane();
    void updateResolution(int width, int height);
    void setEnableSLAM(boolean enable);
    boolean isEnableSLAM();
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

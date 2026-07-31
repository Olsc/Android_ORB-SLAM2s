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
        public void processFrameSharedMem(ParcelFileDescriptor pfd, int size, int width, int height) {
            if (nativeHelper == null || pfd == null) return;
            try {
                int fd = pfd.getFd();
                int trackingResult = nativeHelper.processFrameSharedMemFd(fd, size, width, height);
                notifyTrackingStatus(trackingResult);
            } catch (Exception e) {
                Log.e(TAG, "处理共享内存帧异常: " + e.getMessage());
            } finally {
                try {
                    pfd.close();
                } catch (Exception ignored) {}
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

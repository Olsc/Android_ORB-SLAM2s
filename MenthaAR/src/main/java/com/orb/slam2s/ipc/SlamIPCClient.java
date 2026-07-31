package com.orb.slam2s.ipc;

import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.os.IBinder;
import android.os.ParcelFileDescriptor;
import android.os.RemoteException;
import android.util.Log;

public class SlamIPCClient {
    private static final String TAG = "SlamIPCClient";

    private final Context context;
    private ISlamService slamService;
    private boolean isBound;

    private SharedMemoryBuffer sharedMemoryBuffer;
    private OnMVPUpdatedCallback mvpUpdatedCallback;
    private SlamInitListener slamInitListener;

    public interface OnMVPUpdatedCallback {
        void onUpdateModelMatrix(float[] M);
        void onUpdateViewMatrix(float[] V);
        void onUpdateProjectionMatrix(float[] P);
        void requestReset();
        void setDraw(boolean draw);
    }

    public interface SlamInitListener {
        void onSLAMInitialized(boolean success, String message);
    }

    private final ISlamCallback.Stub serviceCallback = new ISlamCallback.Stub() {
        @Override
        public void onMVPUpdated(float[] modelMatrix, float[] viewMatrix, float[] projectionMatrix, boolean draw) {
            if (mvpUpdatedCallback != null) {
                mvpUpdatedCallback.setDraw(draw);
                if (draw) {
                    mvpUpdatedCallback.requestReset();
                    mvpUpdatedCallback.onUpdateModelMatrix(modelMatrix);
                    mvpUpdatedCallback.onUpdateProjectionMatrix(projectionMatrix);
                }
            }
        }

        @Override
        public void onTrackingStatusChanged(int trackingStatus) {
            // 可用于全局状态监听
        }

        @Override
        public void onPlaneDetected(int result) {
            // 平面检测结果通知
        }

        @Override
        public void onSLAMInitialized(boolean success, String message) {
            if (slamInitListener != null) {
                slamInitListener.onSLAMInitialized(success, message);
            }
        }
    };

    private String pendingResDir;

    private final ServiceConnection serviceConnection = new ServiceConnection() {
        @Override
        public void onServiceConnected(ComponentName name, IBinder service) {
            slamService = ISlamService.Stub.asInterface(service);
            isBound = true;
            Log.d(TAG, "成功绑定到 MenthaAR SlamService 进程");
            try {
                slamService.registerCallback(serviceCallback);
                if (pendingResDir != null) {
                    Log.d(TAG, "自动执行挂起的 SLAM 初始化: " + pendingResDir);
                    slamService.initSLAM(pendingResDir);
                    pendingResDir = null;
                }
            } catch (RemoteException e) {
                Log.e(TAG, "注册回调与延迟初始化异常: " + e.getMessage());
            }
        }

        @Override
        public void onServiceDisconnected(ComponentName name) {
            slamService = null;
            isBound = false;
            Log.w(TAG, "MenthaAR SlamService 进程连接断开");
        }
    };

    public SlamIPCClient(Context context) {
        this.context = context;
    }

    public void bindService() {
        Intent intent = new Intent(context, SlamService.class);
        context.bindService(intent, serviceConnection, Context.BIND_AUTO_CREATE);
    }

    public void unbindService() {
        if (isBound && slamService != null) {
            try {
                slamService.unregisterCallback(serviceCallback);
            } catch (RemoteException ignored) {}
            context.unbindService(serviceConnection);
            isBound = false;
        }
        if (sharedMemoryBuffer != null) {
            sharedMemoryBuffer.close();
            sharedMemoryBuffer = null;
        }
    }

    public boolean isConnected() {
        return isBound && slamService != null;
    }

    public void setSlamInitListener(SlamInitListener listener) {
        this.slamInitListener = listener;
    }

    public void setOnMVPUpdatedCallback(OnMVPUpdatedCallback callback) {
        this.mvpUpdatedCallback = callback;
    }

    public void initSLAM(String resDir) {
        this.pendingResDir = resDir;
        if (slamService != null) {
            try {
                slamService.initSLAM(resDir);
                pendingResDir = null;
            } catch (RemoteException e) {
                Log.e(TAG, "initSLAM 异常: " + e.getMessage());
            }
        } else {
            Log.w(TAG, "SlamService 尚未就绪，挂起 SLAM 初始化指令: " + resDir);
        }
    }

    public java.nio.ByteBuffer getSharedMemoryBuffer() {
        return (sharedMemoryBuffer != null) ? sharedMemoryBuffer.getByteBuffer() : null;
    }

    /**
     * 将相机帧数据写入 SharedMemory 并通过零拷贝跨进程通知 MenthaAR 后序进程
     */
    public void sendFrameData(byte[] frameData, int width, int height) {
        if (slamService == null || frameData == null) return;

        int requiredSize = frameData.length;
        if (sharedMemoryBuffer == null || sharedMemoryBuffer.getBufferSize() < requiredSize) {
            if (sharedMemoryBuffer != null) {
                sharedMemoryBuffer.close();
            }
            sharedMemoryBuffer = new SharedMemoryBuffer("MenthaSlamFrameBuffer", requiredSize);
        }

        sharedMemoryBuffer.writeData(frameData, frameData.length);
        ParcelFileDescriptor pfd = sharedMemoryBuffer.getParcelFileDescriptor();

        if (pfd != null) {
            try {
                slamService.processFrameSharedMem(ParcelFileDescriptor.dup(pfd.getFileDescriptor()), requiredSize, width, height);
            } catch (Exception e) {
                Log.e(TAG, "sendFrameData 异常: " + e.getMessage());
            }
        }
    }

    public void detectPlane() {
        if (slamService != null) {
            try {
                slamService.detectPlane();
            } catch (RemoteException e) {
                Log.e(TAG, "detectPlane 异常: " + e.getMessage());
            }
        }
    }

    public void updateResolution(int width, int height) {
        if (slamService != null) {
            try {
                slamService.updateResolution(width, height);
            } catch (RemoteException e) {
                Log.e(TAG, "updateResolution 异常: " + e.getMessage());
            }
        }
    }

    public void setEnableSLAM(boolean enable) {
        if (slamService != null) {
            try {
                slamService.setEnableSLAM(enable);
            } catch (RemoteException e) {
                Log.e(TAG, "setEnableSLAM 异常: " + e.getMessage());
            }
        }
    }

    public boolean isEnableSLAM() {
        if (slamService != null) {
            try {
                return slamService.isEnableSLAM();
            } catch (RemoteException e) {
                Log.e(TAG, "isEnableSLAM 异常: " + e.getMessage());
            }
        }
        return false;
    }

    public void setPointCloudDisplay(boolean enable) {
        if (slamService != null) {
            try {
                slamService.setPointCloudDisplay(enable);
            } catch (RemoteException e) {
                Log.e(TAG, "setPointCloudDisplay 异常: " + e.getMessage());
            }
        }
    }

    public boolean isPointCloudDisplayEnabled() {
        if (slamService != null) {
            try {
                return slamService.isPointCloudDisplayEnabled();
            } catch (RemoteException e) {
                Log.e(TAG, "isPointCloudDisplayEnabled 异常: " + e.getMessage());
            }
        }
        return false;
    }

    public void saveMap(String mapPath) {
        if (slamService != null) {
            try {
                slamService.saveMap(mapPath);
            } catch (RemoteException e) {
                Log.e(TAG, "saveMap 异常: " + e.getMessage());
            }
        }
    }

    public void loadMapWithId(String mapPath, int mapId, boolean append) {
        if (slamService != null) {
            try {
                slamService.loadMapWithId(mapPath, mapId, append);
            } catch (RemoteException e) {
                Log.e(TAG, "loadMapWithId 异常: " + e.getMessage());
            }
        }
    }

    public int getCurrentMapId() {
        if (slamService != null) {
            try {
                return slamService.getCurrentMapId();
            } catch (RemoteException e) {
                Log.e(TAG, "getCurrentMapId 异常: " + e.getMessage());
            }
        }
        return 0;
    }

    public int[] getMapStats() {
        if (slamService != null) {
            try {
                return slamService.getMapStats();
            } catch (RemoteException e) {
                Log.e(TAG, "getMapStats 异常: " + e.getMessage());
            }
        }
        return new int[0];
    }

    public float[] getTrackedPoints(int maxPoints) {
        if (slamService != null) {
            try {
                return slamService.getTrackedPoints(maxPoints);
            } catch (RemoteException e) {
                Log.e(TAG, "getTrackedPoints 异常: " + e.getMessage());
            }
        }
        return new float[0];
    }

    public float[] getMiniMapPoints(int maxPoints) {
        if (slamService != null) {
            try {
                return slamService.getMiniMapPoints(maxPoints);
            } catch (RemoteException e) {
                Log.e(TAG, "getMiniMapPoints 异常: " + e.getMessage());
            }
        }
        return new float[0];
    }

    public float[] getAllArObjectsData() {
        if (slamService != null) {
            try {
                return slamService.getAllArObjectsData();
            } catch (RemoteException e) {
                Log.e(TAG, "getAllArObjectsData 异常: " + e.getMessage());
            }
        }
        return new float[0];
    }

    public void updateArObjectScale(float scaleFactor) {
        if (slamService != null) {
            try {
                slamService.updateArObjectScale(scaleFactor);
            } catch (RemoteException e) {
                Log.e(TAG, "updateArObjectScale 异常: " + e.getMessage());
            }
        }
    }

    public float getArObjectScale() {
        if (slamService != null) {
            try {
                return slamService.getArObjectScale();
            } catch (RemoteException e) {
                Log.e(TAG, "getArObjectScale 异常: " + e.getMessage());
            }
        }
        return 1.0f;
    }
}

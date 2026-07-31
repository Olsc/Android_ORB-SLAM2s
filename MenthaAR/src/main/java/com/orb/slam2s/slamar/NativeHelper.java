package com.orb.slam2s.slamar;
/**
 * Created by Ads on 2017/1/30.
 * 由Olsc于2025/8/25开始进行修改
 */

import android.content.Context;
import android.util.Log;

import com.olsc.menthar.R;
import com.orb.slam2s.constant.GlobalConstant;

import org.json.JSONObject;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileWriter;
import java.util.ArrayList;

/**
 * NativeHelper类：用于与JNI层交互，处理摄像头帧、平面检测、SLAM初始化等功能
 */
public class NativeHelper {
    private static final String TAG = "NativeHelper";

    // 加载必要的本地库
    static {
        try {
            System.loadLibrary("Mentha_Profiler");
        } catch (UnsatisfiedLinkError e) {
            Log.d(TAG, "未开启Profiler");
        }
        System.loadLibrary("MenthaAR_Engine");         // 整合后的 SLAM_AR（含 OpenCV 原生模块）
        System.loadLibrary("Mentha_3DOF");            // 独立出来的 3DOF 库
    }

    // 存储MVP回调的集合
    private ArrayList<OnMVPUpdatedCallback> onMVPUpdatedCallbacks;
    private final float[] projectionMatrix = new float[16];  // 投影矩阵
    private final float[] viewMatrix = new float[16];        // 视图矩阵
    private final float[] modelMatrix = new float[16];       // 模型矩阵
    private int lastTrackingResult;                     // 最后的追踪结果
    private int planeDetectResult;                      // 平面检测结果
    private boolean planeDetected;                      // 是否检测到平面

    private final int[] statusBuf = new int[233];             // 状态缓冲区

    private Context context;  // 上下文对象

    // 构造函数，初始化NativeHelper对象
    public NativeHelper(Context context) {
        this.context = context;
        onMVPUpdatedCallbacks = new ArrayList<>();
        planeDetected = false;
    }

    /**
     * 处理摄像头帧
     * @param matAddrGr 摄像头灰度图像地址
     * @param matAddrRgba 摄像头彩色图像地址
     * @return 最后的追踪结果
     */
    public int processCameraFrame(long matAddrGr, long matAddrRgba) {
        nativeProcessFrameMat(matAddrGr, matAddrRgba, statusBuf);
        lastTrackingResult = statusBuf[0];
        boolean shouldDraw = (statusBuf[1] != 0);

        for (OnMVPUpdatedCallback cb : onMVPUpdatedCallbacks) {
            cb.setDraw(shouldDraw);
        }
        return lastTrackingResult;
    }

    /**
     * 检测平面
     * @return 平面检测结果
     */
    public int detectPlane() {
        detect(statusBuf);
        planeDetectResult = statusBuf[1];

        if (planeDetectResult == GlobalConstant.PLANE_DETECTED) {
            planeDetected = true;
            nativeGetMVP(modelMatrix, viewMatrix, projectionMatrix,
                         GlobalConstant.RESOLUTION_WIDTH, GlobalConstant.RESOLUTION_HEIGHT);
            for (OnMVPUpdatedCallback cb : onMVPUpdatedCallbacks) {
                cb.requestReset();
                cb.onUpdateModelMatrix(modelMatrix);
                cb.onUpdateProjectionMatrix(projectionMatrix);
            }
        } else {
            planeDetected = false;
        }

        for (OnMVPUpdatedCallback cb : onMVPUpdatedCallbacks) {
            cb.setDraw(planeDetected);
        }
        return planeDetectResult;
    }

    // 本地方法：处理摄像头帧 (statusBuf=[tracking,shouldDraw,scaleBits])
    public native void nativeProcessFrameMat(long matAddrGr, long matAddrRgba, int[] statusBuf);

    // 本地方法：处理共享内存句柄数据帧
    public native int nativeProcessFrameSharedMemFd(int fd, int size, int width, int height);

    public int processFrameSharedMemFd(int fd, int size, int width, int height) {
        return nativeProcessFrameSharedMemFd(fd, size, width, height);
    }

    // 统一获取MVP（替代getM/getV/getP）
    public native void nativeGetMVP(float[] M, float[] V, float[] P, int w, int h);

    // 本地方法：进行平面检测
    public native void detect(int[] statusBuf);

    // 本地方法：初始化SLAM
    public native void initSLAM(String path);

    // 本地方法：更新相机分辨率并重新计算内参
    public native void nativeUpdateResolution(int cameraWidth, int cameraHeight);

    /**
     * 通知native层相机分辨率已更新
     * 在相机启动或旋转变化时调用
     */
    public void updateResolution(int width, int height) {
        if (width > 0 && height > 0) {
            nativeUpdateResolution(width, height);
        }
    }

    // 本地方法：保存/加载地图
    public native void saveMap(String path);
    public native void loadMap(String path);
    public native void loadMapWithId(String path, int mapId, boolean append);
    public native int getCurrentMapId();
    public native int[] getMapStats();

    // 点云显示控制（控制绿色和蓝色点云）
    public native void setPointCloudDisplay(boolean enable);
    public native boolean isPointCloudDisplayEnabled();
    
    // SLAM 开关控制
    public native void setEnableSLAM(boolean enable);
    public native boolean isEnableSLAM();

    
    // 3DOF功能接口
    public native float[] calculate3DofInsertionPoint(float[] rotationMatrix, int rotation, float distance);
    public native float[] compute3DofMVP(float[] rotationMatrix, int rotation, float ratio, float[] objectPos);
    
    // AR对象缩放
    public native void updateArObjectScale(float scaleFactor);
    public native float getArObjectScale();

    public native float[] getMiniMapPoints(int maxPoints);
    public native float[] getTrackedPoints(int maxPoints);
    public native float[] getAllArObjectsData();

    // 视图矩阵 (WebServer使用)
    public native void getV(float viewM[]);

    // 获取最后的追踪结果
    public int getLastTrackingResult() {
        return lastTrackingResult;
    }

    // 添加MVP更新回调
    public void addOnMVPUpdatedCallback(OnMVPUpdatedCallback onMVPUpdatedCallback) {
        onMVPUpdatedCallbacks.add(onMVPUpdatedCallback);
    }

    // MVP更新回调接口
    public interface OnMVPUpdatedCallback {
        void onUpdateModelMatrix(float M[]);        // 更新模型矩阵
        void onUpdateViewMatrix(float M[]);         // 更新视图矩阵
        void onUpdateProjectionMatrix(float M[]);   // 更新投影矩阵
        void requestReset();                        // 请求重置
        void setDraw(boolean flag);                 // 设置是否绘制
    }

    /**
     * MapManager类：管理地图的保存、加载、删除和查询
     */
    public static class MapManager {
        private static final String TAG = "MapManager";
        private static final String MAP_DIR_NAME = "SLAM/maps";
        private static final String MAP_METADATA_EXT = ".json";
        
        private final Context context;
        private final NativeHelper nativeHelper;
        private final File mapDirectory;

        public MapManager(Context context, NativeHelper nativeHelper) {
            this.context = context;
            this.nativeHelper = nativeHelper;
            this.mapDirectory = new File(context.getExternalFilesDir(null), MAP_DIR_NAME);
            
            // 确保地图目录存在
            if (!mapDirectory.exists()) {
                mapDirectory.mkdirs();
            }
        }

        /**
         * 保存地图
         * @param mapName 地图名称
         */
        public void saveMap(String mapName) {
            try {
                // 清理文件名
                mapName = mapName.replaceAll("[^a-zA-Z0-9_\\-]", "_");
                
                String mapPath = new File(mapDirectory, mapName + ".bin").getAbsolutePath();
                nativeHelper.saveMap(mapPath);
                
                // 获取地图统计信息
                int[] stats = nativeHelper.getMapStats();
                int keyFrames = stats != null && stats.length > 0 ? stats[0] : 0;
                int mapPoints = stats != null && stats.length > 1 ? stats[1] : 0;
                boolean hasPlane = stats != null && stats.length > 2 && stats[2] > 0;
                
                // 保存元数据
                MapInfo info = new MapInfo();
                info.name = mapName;
                info.keyFrames = keyFrames;
                info.mapPoints = mapPoints;
                info.createTime = System.currentTimeMillis();
                info.hasPlane = hasPlane;
                info.fileSize = new File(mapPath).length();
                
                saveMetadata(info);
                
                android.widget.Toast.makeText(context, context.getString(R.string.hint_map_saved, mapName), 
                    android.widget.Toast.LENGTH_SHORT).show();
                Log.d(TAG, "地图已保存: " + mapName + " (KFs: " + keyFrames + ", MPs: " + mapPoints + ")");
            } catch (Exception e) {
                Log.e(TAG, "保存地图失败: " + e.getMessage(), e);
                android.widget.Toast.makeText(context, context.getString(R.string.hint_map_save_failed, e.getMessage()), 
                    android.widget.Toast.LENGTH_SHORT).show();
            }
        }

        /**
         * 加载地图
         * @param mapName 地图名称
         */
        public void loadMap(String mapName) {
            loadMapWithId(mapName, 0, false);
        }

        /**
         * 加载地图（指定ID和追加模式）
         * @param mapName 地图名称
         * @param mapId 地图ID
         * @param append 是否追加
         */
        public void loadMapWithId(String mapName, int mapId, boolean append) {
            try {
                String mapPath = new File(mapDirectory, mapName + ".bin").getAbsolutePath();
                File mapFile = new File(mapPath);
                
                if (!mapFile.exists()) {
                    android.widget.Toast.makeText(context, context.getString(R.string.hint_map_file_not_found), 
                        android.widget.Toast.LENGTH_SHORT).show();
                    return;
                }
                
                nativeHelper.loadMapWithId(mapPath, mapId, append);
                
                if (mapId == 0 && !append) {
                     android.widget.Toast.makeText(context, context.getString(R.string.hint_map_loaded, mapName), 
                        android.widget.Toast.LENGTH_SHORT).show();
                }
                
                Log.d(TAG, "地图已加载: " + mapName + " (ID=" + mapId + ", Append=" + append + ")");
            } catch (Exception e) {
                Log.e(TAG, "加载地图失败: " + e.getMessage(), e);
                android.widget.Toast.makeText(context, context.getString(R.string.hint_map_load_failed, e.getMessage()), 
                    android.widget.Toast.LENGTH_SHORT).show();
            }
        }

        /**
         * 删除地图
         * @param mapName 地图名称
         * @return 是否删除成功
         */
        public boolean deleteMap(String mapName) {
            try {
                File mapFile = new File(mapDirectory, mapName + ".bin");
                File metaFile = new File(mapDirectory, mapName + MAP_METADATA_EXT);
                
                boolean success = true;
                if (mapFile.exists()) {
                    success = mapFile.delete();
                }
                if (metaFile.exists()) {
                    metaFile.delete();
                }
                
                if (success) {
                    Log.d(TAG, "地图已删除: " + mapName);
                }
                return success;
            } catch (Exception e) {
                Log.e(TAG, "删除地图失败: " + e.getMessage(), e);
                return false;
            }
        }

        /**
         * 获取所有已保存的地图
         * @return 地图信息列表
         */
        public ArrayList<MapInfo> getAllMaps() {
            ArrayList<MapInfo> maps = new ArrayList<>();
            
            if (!mapDirectory.exists()) {
                return maps;
            }
            
            File[] files = mapDirectory.listFiles();
            if (files == null) {
                return maps;
            }
            
            for (File file : files) {
                if (file.getName().endsWith(".bin")) {
                    String mapName = file.getName().replace(".bin", "");
                    MapInfo info = loadMetadata(mapName);
                    
                    // 如果没有元数据，创建基本信息
                    if (info == null) {
                        info = new MapInfo();
                        info.name = mapName;
                        info.fileSize = file.length();
                        info.createTime = file.lastModified();
                        info.keyFrames = 0;
                        info.mapPoints = 0;
                        info.hasPlane = false;
                    } else {
                        // 更新文件大小（可能已改变）
                        info.fileSize = file.length();
                    }
                    
                    maps.add(info);
                }
            }
            
            // 按创建时间降序排序（最新的在前）
            java.util.Collections.sort(maps, new java.util.Comparator<MapInfo>() {
                @Override
                public int compare(MapInfo m1, MapInfo m2) {
                    return Long.compare(m2.createTime, m1.createTime);
                }
            });
            
            return maps;
        }

        /**
         * 保存地图元数据
         */
        private void saveMetadata(MapInfo info) {
            try {
                File metaFile = new File(mapDirectory, info.name + MAP_METADATA_EXT);
                JSONObject json = new JSONObject();
                json.put("name", info.name);
                json.put("keyFrames", info.keyFrames);
                json.put("mapPoints", info.mapPoints);
                json.put("createTime", info.createTime);
                json.put("hasPlane", info.hasPlane);
                json.put("fileSize", info.fileSize);
                
                FileWriter writer = new FileWriter(metaFile);
                writer.write(json.toString(2));
                writer.close();
            } catch (Exception e) {
                Log.e(TAG, "保存元数据失败: " + e.getMessage(), e);
            }
        }

        /**
         * 加载地图元数据
         */
        private MapInfo loadMetadata(String mapName) {
            try {
                File metaFile = new File(mapDirectory, mapName + MAP_METADATA_EXT);
                if (!metaFile.exists()) {
                    return null;
                }
                
                FileInputStream fis = new FileInputStream(metaFile);
                byte[] data = new byte[(int) metaFile.length()];
                fis.read(data);
                fis.close();
                
                JSONObject json = new JSONObject(new String(data, "UTF-8"));
                MapInfo info = new MapInfo();
                info.name = json.getString("name");
                info.keyFrames = json.getInt("keyFrames");
                info.mapPoints = json.getInt("mapPoints");
                info.createTime = json.getLong("createTime");
                info.hasPlane = json.getBoolean("hasPlane");
                info.fileSize = json.getLong("fileSize");
                
                return info;
            } catch (Exception e) {
                Log.e(TAG, "加载元数据失败: " + e.getMessage(), e);
                return null;
            }
        }

        /**
         * MapInfo类：存储地图的元数据信息
         */
        public static class MapInfo {
            public String name;         // 地图名称
            public int keyFrames;       // 关键帧数量
            public int mapPoints;       // 地图点数量
            public long fileSize;       // 文件大小（字节）
            public long createTime;     // 创建时间（毫秒时间戳）
            public boolean hasPlane;    // 是否包含平面
        }
    }
}
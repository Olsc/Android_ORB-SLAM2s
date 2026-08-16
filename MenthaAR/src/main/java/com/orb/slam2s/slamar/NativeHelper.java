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

// NativeHelper类：与JNI层交互，处理摄像头帧、平面检测、SLAM初始化等功能
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

    private int lastTrackingResult;                     // 最后的追踪结果
    private int planeDetectResult;                      // 平面检测结果
    private boolean planeDetected;                      // 是否检测到平面

    // J-4：状态缓冲区实际只使用 3 个 int [tracking, shouldDraw, scaleBits]，
    // 原先的 233 是把 PLANE_DETECTED 常量误当作数组长度
    private final int[] statusBuf = new int[3];

    private Context context;  // 上下文对象

    // 构造函数，初始化NativeHelper对象
    public NativeHelper(Context context) {
        this.context = context;
        planeDetected = false;
    }

    /**
     * 检测平面（在 SLAM 处理线程内调用）。
     * MVP 结果由 native 写回共享内存，UI 渲染线程直接读取。
     * @return 平面检测结果
     */
    public int detectPlane() {
        detect(statusBuf);
        planeDetectResult = statusBuf[1];
        planeDetected = (planeDetectResult == GlobalConstant.PLANE_DETECTED);
        return planeDetectResult;
    }

    // 本地方法：持久映射共享内存帧缓冲（仅在缓冲创建/尺寸变化时调用一次）
    public native boolean nativeAttachFrameBuffer(int fd, int size);
    public native void nativeDetachFrameBuffer();
    // 本地方法：处理持久映射缓冲中的帧（双缓冲 bufIndex 0/1；每帧只传序号+宽高，无 fd 开销）
    // native 处理完成后会把 tracking/draw/MVP/点云/slamDoneSeq 直接写回共享内存 header
    // statusBuf: [0]=tracking, [1]=shouldDraw
    public native void nativeProcessFrameSharedMem(int bufIndex, int seq, int width, int height, int[] statusBuf);

    public boolean attachFrameBuffer(int fd, int size) {
        return nativeAttachFrameBuffer(fd, size);
    }

    public void detachFrameBuffer() {
        nativeDetachFrameBuffer();
    }

    public int processFrameSharedMem(int bufIndex, int seq, int width, int height, int[] statusBuf) {
        nativeProcessFrameSharedMem(bufIndex, seq, width, height, statusBuf);
        return statusBuf[0];
    }

    // 统一获取MVP（替代getM/getV/getP）
    public native void nativeGetMVP(float[] M, float[] V, float[] P, int w, int h);

    // 本地方法：进行平面检测
    public native void detect(int[] statusBuf);

    // 本地方法：初始化SLAM
    public native void initSLAM(String path);

    // J-13：关停并释放 SLAM 系统（join 全部工作线程后 delete），
    // Activity.onDestroy 调用，修复 LM/LC/GlobalReloc 三条线程永不退出的问题
    public native void nativeShutdown();

    // 供 Activity 生命周期调用的安全封装
    public void shutdownSLAM() {
        try {
            nativeShutdown();
        } catch (UnsatisfiedLinkError e) {
            // 库未加载（如极早期退出）时静默忽略
        }
    }

    // 本地方法：更新相机分辨率并重新计算内参
    public native void nativeUpdateResolution(int cameraWidth, int cameraHeight);

    // 通知 native 层相机分辨率已更新，在相机启动或旋转变化时调用
    public void updateResolution(int width, int height) {
        if (width > 0 && height > 0) {
            nativeUpdateResolution(width, height);
        }
    }

    // 本地方法：保存/加载地图
    public native void saveMap(String path);
    public native void loadMapWithId(String path, int mapId, boolean append);
    public native int[] getMapStats();

    // 点云显示控制（控制绿色和蓝色点云）
    public native void setPointCloudDisplay(boolean enable);
    public native boolean isPointCloudDisplayEnabled();

    // SLAM 开关控制
    public native void setEnableSLAM(boolean enable);
    public native boolean isEnableSLAM();

    // 3DOF功能接口
    public native float[] calculate3DofInsertionPoint(float[] rotationMatrix, int rotation, float distance);
    // J-9：出参版本——直接填充调用方缓冲，避免每帧 JNI 分配新 float[16]
    public native void compute3DofMVP(float[] outMvp, float[] rotationMatrix, int rotation, float ratio, float[] objectPos);

    // AR对象缩放
    public native void updateArObjectScale(float scaleFactor);

    public native float[] getMiniMapPoints(int maxPoints);
    public native float[] getTrackedPoints(int maxPoints);
    public native float[] getAllArObjectsData();

    // 视图矩阵 (WebServer使用)
    public native void getV(float viewM[]);

    // 获取最后的追踪结果
    public int getLastTrackingResult() {
        return lastTrackingResult;
    }

    // MapManager类：管理地图的保存、加载、删除和查询
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

            if (!mapDirectory.exists()) {
                mapDirectory.mkdirs();
            }
        }

        public void saveMap(String mapName) {
            try {
                mapName = mapName.replaceAll("[^a-zA-Z0-9_\\-]", "_");

                String mapPath = new File(mapDirectory, mapName + ".bin").getAbsolutePath();
                nativeHelper.saveMap(mapPath);

                int[] stats = nativeHelper.getMapStats();
                int keyFrames = stats != null && stats.length > 0 ? stats[0] : 0;
                int mapPoints = stats != null && stats.length > 1 ? stats[1] : 0;
                boolean hasPlane = stats != null && stats.length > 2 && stats[2] > 0;

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

        public void loadMap(String mapName) {
            loadMapWithId(mapName, 0, false);
        }

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

                    if (info == null) {
                        info = new MapInfo();
                        info.name = mapName;
                        info.fileSize = file.length();
                        info.createTime = file.lastModified();
                        info.keyFrames = 0;
                        info.mapPoints = 0;
                        info.hasPlane = false;
                    } else {
                        info.fileSize = file.length();
                    }

                    maps.add(info);
                }
            }

            java.util.Collections.sort(maps, new java.util.Comparator<MapInfo>() {
                @Override
                public int compare(MapInfo m1, MapInfo m2) {
                    return Long.compare(m2.createTime, m1.createTime);
                }
            });

            return maps;
        }

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

        public static class MapInfo {
            public String name;
            public int keyFrames;
            public int mapPoints;
            public long fileSize;
            public long createTime;
            public boolean hasPlane;
        }
    }
}
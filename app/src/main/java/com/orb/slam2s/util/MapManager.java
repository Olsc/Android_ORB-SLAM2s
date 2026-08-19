package com.orb.slam2s.util;

import android.content.Context;
import android.util.Log;
import android.widget.Toast;

import com.orb.slam2s.R;
import com.orb.slam2s.ipc.SlamIPCClient;

import org.json.JSONObject;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileWriter;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Collections;

// 地图管理工具类：负责地图文件的本地元数据存储、查询与跨进程保存/加载交互
public class MapManager {
    private static final String TAG = "MapManager";
    private static final String MAP_DIR_NAME = "SLAM/maps";
    private static final String MAP_METADATA_EXT = ".json";

    private final Context mContext;
    private final SlamIPCClient mSlamIPCClient;
    private final File mMapDirectory;

    public MapManager(Context context, SlamIPCClient client) {
        this.mContext = context;
        this.mSlamIPCClient = client;
        this.mMapDirectory = new File(context.getExternalFilesDir(null), MAP_DIR_NAME);

        if (!mMapDirectory.exists()) {
            mMapDirectory.mkdirs();
        }
    }

    public void saveMap(String mapName) {
        try {
            mapName = mapName.replaceAll("[^a-zA-Z0-9_\\-]", "_");

            String mapPath = new File(mMapDirectory, mapName + ".bin").getAbsolutePath();
            if (mSlamIPCClient != null) {
                mSlamIPCClient.saveMap(mapPath);
            }

            int[] stats = mSlamIPCClient != null ? mSlamIPCClient.getMapStats() : new int[0];
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

            Toast.makeText(mContext, mContext.getString(R.string.hint_map_saved, mapName),
                    Toast.LENGTH_SHORT).show();
            Log.d(TAG, "地图已保存: " + mapName + " (KFs: " + keyFrames + ", MPs: " + mapPoints + ")");
        } catch (Exception e) {
            Log.e(TAG, "保存地图失败: " + e.getMessage(), e);
            Toast.makeText(mContext, mContext.getString(R.string.hint_map_save_failed, e.getMessage()),
                    Toast.LENGTH_SHORT).show();
        }
    }

    public void loadMap(String mapName) {
        loadMapWithId(mapName, 0, false);
    }

    public void loadMapWithId(String mapName, int mapId, boolean append) {
        try {
            String mapPath = new File(mMapDirectory, mapName + ".bin").getAbsolutePath();
            File mapFile = new File(mapPath);

            if (!mapFile.exists()) {
                Toast.makeText(mContext, mContext.getString(R.string.hint_map_file_not_found),
                        Toast.LENGTH_SHORT).show();
                return;
            }

            if (mSlamIPCClient != null) {
                mSlamIPCClient.loadMapWithId(mapPath, mapId, append);
            }

            if (mapId == 0 && !append) {
                Toast.makeText(mContext, mContext.getString(R.string.hint_map_loaded, mapName),
                        Toast.LENGTH_SHORT).show();
            }

            Log.d(TAG, "地图已加载: " + mapName + " (ID=" + mapId + ", Append=" + append + ")");
        } catch (Exception e) {
            Log.e(TAG, "加载地图失败: " + e.getMessage(), e);
            Toast.makeText(mContext, mContext.getString(R.string.hint_map_load_failed, e.getMessage()),
                    Toast.LENGTH_SHORT).show();
        }
    }

    public boolean deleteMap(String mapName) {
        try {
            File mapFile = new File(mMapDirectory, mapName + ".bin");
            File arInfoFile = new File(mMapDirectory, mapName + ".bin.arinfo");
            File metaFile = new File(mMapDirectory, mapName + MAP_METADATA_EXT);

            boolean success = true;
            if (mapFile.exists()) {
                success = mapFile.delete();
            }
            if (arInfoFile.exists()) {
                arInfoFile.delete();
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

        if (!mMapDirectory.exists()) {
            return maps;
        }

        File[] files = mMapDirectory.listFiles();
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

        Collections.sort(maps, (m1, m2) -> Long.compare(m2.createTime, m1.createTime));

        return maps;
    }

    private void saveMetadata(MapInfo info) {
        try {
            File metaFile = new File(mMapDirectory, info.name + MAP_METADATA_EXT);
            JSONObject json = new JSONObject();
            json.put("name", info.name);
            json.put("keyFrames", info.keyFrames);
            json.put("mapPoints", info.mapPoints);
            json.put("createTime", info.createTime);
            json.put("hasPlane", info.hasPlane);
            json.put("fileSize", info.fileSize);

            try (FileWriter writer = new FileWriter(metaFile)) {
                writer.write(json.toString(2));
            }
        } catch (Exception e) {
            Log.e(TAG, "保存元数据失败: " + e.getMessage(), e);
        }
    }

    private MapInfo loadMetadata(String mapName) {
        File metaFile = new File(mMapDirectory, mapName + MAP_METADATA_EXT);
        if (!metaFile.exists()) {
            return null;
        }

        try (FileInputStream fis = new FileInputStream(metaFile)) {
            byte[] data = new byte[(int) metaFile.length()];
            int read = fis.read(data);
            if (read <= 0) return null;

            JSONObject json = new JSONObject(new String(data, StandardCharsets.UTF_8));
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

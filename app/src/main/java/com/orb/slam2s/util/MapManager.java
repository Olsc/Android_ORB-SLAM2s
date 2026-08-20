/*
 * Copyright (C) 2026 Olsc <OlscStudio@outlook.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.orb.slam2s.util;

import android.content.Context;
import android.util.Log;

import com.orb.slam2s.ipc.SlamIPCClient;

import org.json.JSONObject;

import java.io.File;
import java.io.FileInputStream;
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

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
import java.io.FileWriter;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Collections;

// 地图管理工具类：负责地图文件的本地元数据存储、查询与跨进程保存/加载交互
public class MapManager {
    private static final String TAG = "MapManager";
    private static final String MAP_DIR_NAME = "SLAM/maps";
    private static final String MAP_METADATA_EXT = ".json";

    private final File mMapDirectory;

    public MapManager(Context context) {
        this.mMapDirectory = new File(context.getExternalFilesDir(null), MAP_DIR_NAME);

        if (!mMapDirectory.exists()) {
            mMapDirectory.mkdirs();
        }
    }

    public MapManager(Context context, SlamIPCClient client) {
        this(context);
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

                // 若缺少元数据或信息为空，直接从 .bin 实际文件头解析真实帧数与地图点数并补齐 JSON
                if (info == null || (info.keyFrames == 0 && info.mapPoints == 0)) {
                    MapInfo binInfo = readBinHeader(file);
                    if (info == null) {
                        info = binInfo;
                    } else if (binInfo != null && (binInfo.keyFrames > 0 || binInfo.mapPoints > 0)) {
                        info.keyFrames = binInfo.keyFrames;
                        info.mapPoints = binInfo.mapPoints;
                        info.hasPlane = binInfo.hasPlane;
                    }
                    if (info != null) {
                        saveMetadata(info);
                    }
                }

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
                info.filePath = file.getAbsolutePath();

                maps.add(info);
            }
        }

        Collections.sort(maps, (m1, m2) -> Long.compare(m2.createTime, m1.createTime));

        return maps;
    }

    public File getMapDirectory() {
        return mMapDirectory;
    }

    public File getMapFile(String mapName) {
        return new File(mMapDirectory, mapName + ".bin");
    }

    public void saveMetadata(MapInfo info) {
        if (info == null) return;
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

    // 从 .bin 二进制文件头读取关键帧与地图点数量
    private MapInfo readBinHeader(File file) {
        String mapName = file.getName().replace(".bin", "");
        MapInfo info = new MapInfo();
        info.name = mapName;
        info.fileSize = file.length();
        info.createTime = file.lastModified();
        info.keyFrames = 0;
        info.mapPoints = 0;
        info.hasPlane = false;

        // .bin 头部格式: magic(4B) + version(4B) + nKFs(4B) + nMPs(4B)
        if (file.length() >= 16) {
            try (FileInputStream fis = new FileInputStream(file)) {
                byte[] header = new byte[16];
                int read = fis.read(header);
                if (read >= 16) {
                    ByteBuffer buf = ByteBuffer.wrap(header).order(ByteOrder.LITTLE_ENDIAN);
                    int magic = buf.getInt();
                    int version = buf.getInt();
                    int nKFs = buf.getInt();
                    int nMPs = buf.getInt();
                    // 兼容标准地图格式 (0x4D415031: MAP1)
                    if (magic == 0x4D415031 || magic == 0x534D4150 || nKFs > 0 || nMPs > 0) {
                        info.keyFrames = Math.max(0, nKFs);
                        info.mapPoints = Math.max(0, nMPs);
                    }
                }
            } catch (Exception e) {
                Log.w(TAG, "解析 .bin 头失败: " + e.getMessage());
            }
        }

        File arInfoFile = new File(mMapDirectory, mapName + ".bin.arinfo");
        if (arInfoFile.exists()) {
            info.hasPlane = true;
            if (arInfoFile.length() >= 9) {
                try (FileInputStream fis = new FileInputStream(arInfoFile)) {
                    byte[] arHeader = new byte[9];
                    if (fis.read(arHeader) >= 9) {
                        info.hasPlane = (arHeader[8] != 0);
                    }
                } catch (Exception ignored) {
                }
            }
        }

        return info;
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
        public String filePath;
    }
}
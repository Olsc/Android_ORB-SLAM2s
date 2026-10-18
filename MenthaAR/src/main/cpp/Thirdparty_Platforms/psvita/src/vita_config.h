/**
 * Copyright (C) 2025-2026 Olsc <OlscStudio@outlook.com>
 *
 * This file is part of the Android ORB-SLAM2s project (a fork of ORB-SLAM2).
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef MENTHAAR_PSVITA_CONFIG_H
#define MENTHAAR_PSVITA_CONFIG_H

#include <stdint.h>

// 应用信息。
#define APP_NAME        "MenthaAR PSVita"
#define APP_VERSION     "01.00"

// 显示分辨率。
#define SCR_W 960
#define SCR_H 544

// 相机分辨率与帧率。
#define CAM_W 640
#define CAM_H 360
#define CAM_FPS 30

// 点云上限与采样步长。
#define MAX_POINTS   150000
#define SAMPLE_STEP  8          // 640/8 * 360/8 = 3600 点/帧

// 地图存储路径。
#define SAVE_DIR     "ux0:data/MenthaAR"
#define SAVE_PATH    SAVE_DIR "/cloud.mpc"

// RGBA 按 Vita GPU 的 A8B8G8R8 内存布局打包（小端：R, G, B, A）。
#define PACK_RGBA(r, g, b, a) \
	( ((uint32_t)(r) & 0xFF)        | (((uint32_t)(g) & 0xFF) << 8)  | \
	  (((uint32_t)(b) & 0xFF) << 16) | (((uint32_t)(a) & 0xFF) << 24) )

#endif // MENTHAAR_PSVITA_CONFIG_H
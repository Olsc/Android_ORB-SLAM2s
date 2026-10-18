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

#ifndef MENTHAAR_PSVITA_POINTCLOUD_H
#define MENTHAAR_PSVITA_POINTCLOUD_H

#include <stddef.h>
#include <stdint.h>
#include <vector>

#include "vita_config.h"

// 带颜色的三维点，保持 POD 以便按内存直接读写。
struct CloudPoint {
	float x, y, z;
	uint32_t color; // PACK_RGBA 排列
};

class PointCloud {
public:
	PointCloud();

	void clear();
	bool add(const CloudPoint &p);
	size_t size() const { return m_points.size(); }
	bool empty() const { return m_points.empty(); }
	const CloudPoint *data() const { return m_points.empty() ? 0 : &m_points[0]; }

	// 持久化：ux0:data/MenthaAR 下的 MPC1 二进制格式。
	bool save(const char *path) const;
	bool load(const char *path);

private:
	std::vector<CloudPoint> m_points;
};

// 生成程序化形状点云，无相机时用于验证渲染 / 存取流程。
void buildDemoCloud(PointCloud &cloud);

#endif // MENTHAAR_PSVITA_POINTCLOUD_H
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

#ifndef MENTHAAR_PSVITA_RENDERER_H
#define MENTHAAR_PSVITA_RENDERER_H

#include <stdint.h>
#include <vector>

#include <vita2d.h>

#include "math3d.h"
#include "pointcloud.h"

// 软件光栅化点云渲染器：在离屏 A8B8G8R8 纹理上用 CPU z-buffer 绘制，
// 再由 vita2d 以 alpha 合成到相机预览之上。
class Renderer {
public:
	Renderer();

	bool init();
	void shutdown();

	// 清空颜色与深度缓冲，clearColor 应为透明。
	void beginFrame(uint32_t clearColor);

	void drawCloud(const PointCloud &cloud, const Mat3 &viewRot,
	               float camDist, float zoom, int splat);

	// HUD 绘制接口，直接写入离屏缓冲。
	void drawText(int x, int y, const char *text, uint32_t color, int scale);
	void drawTextf(int x, int y, uint32_t color, int scale, const char *fmt, ...);
	void drawPanel(int x, int y, int w, int h, float alpha);
	void drawMarker(int x, int y, uint32_t color, int radius);

	vita2d_texture *texture() const { return m_tex; }

private:
	void putPixel(int x, int y, uint32_t color);
	void blendPixel(int x, int y, uint32_t color, float alpha);

	vita2d_texture *m_tex;
	uint32_t *m_pix;
	int m_stride; // 以像素为单位
	int m_w, m_h;
	std::vector<float> m_zbuf;
};

#endif // MENTHAAR_PSVITA_RENDERER_H
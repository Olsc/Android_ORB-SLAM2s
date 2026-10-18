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

#ifndef MENTHAAR_PSVITA_CAMERA_H
#define MENTHAAR_PSVITA_CAMERA_H

#include <stdint.h>

#include <psp2/types.h>

// SceCamera 的轻量封装：以 ABGR 32bpp、640x360 取帧，可直接映射到 A8B8G8R8 纹理。
class VitaCamera {
public:
	VitaCamera();
	~VitaCamera();

	bool open(int device); // device 0=前置, 1=后置
	void close();
	bool isOpen() const { return m_open; }
	int device() const { return m_dev; }

	// 阻塞读取下一帧，成功返回 true。
	bool readFrame();

	const uint32_t *frame() const { return (const uint32_t *)m_base; }

private:
	int m_dev;
	bool m_open;
	SceUID m_memblock;
	void *m_base;
};

#endif // MENTHAAR_PSVITA_CAMERA_H
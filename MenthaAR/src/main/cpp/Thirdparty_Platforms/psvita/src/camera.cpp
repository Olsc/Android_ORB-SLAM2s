#include "camera.h"
#include "config.h"

#include <string.h>

#include <psp2/camera.h>
#include <psp2/kernel/sysmem.h>

VitaCamera::VitaCamera()
	: m_dev(-1), m_open(false), m_memblock(-1), m_base(0)
{
}

VitaCamera::~VitaCamera()
{
	close();
}

bool VitaCamera::open(int device)
{
	close();

	const int size = CAM_W * CAM_H * 4;
	const int aligned = (size + 0xFFFF) & ~0xFFFF;

	m_memblock = sceKernelAllocMemBlock("MenthaAR_Cam",
	                                    SCE_KERNEL_MEMBLOCK_TYPE_USER_CDRAM_RW,
	                                    aligned, 0);
	if (m_memblock < 0) {
		m_memblock = -1;
		return false;
	}

	if (sceKernelGetMemBlockBase(m_memblock, &m_base) < 0 || !m_base) {
		sceKernelFreeMemBlock(m_memblock);
		m_memblock = -1;
		m_base = 0;
		return false;
	}
	memset(m_base, 0, size);

	SceCameraInfo info;
	memset(&info, 0, sizeof(info));
	info.size = sizeof(info);
	info.priority = SCE_CAMERA_PRIORITY_SHARE;
	info.format = SCE_CAMERA_FORMAT_ABGR;
	info.resolution = SCE_CAMERA_RESOLUTION_640_360;
	info.framerate = CAM_FPS;
	info.sizeIBase = size;
	info.pIBase = m_base;
	info.pitch = 0;

	if (sceCameraOpen(device, &info) < 0) {
		sceKernelFreeMemBlock(m_memblock);
		m_memblock = -1;
		m_base = 0;
		return false;
	}

	if (sceCameraStart(device) < 0) {
		sceCameraClose(device);
		sceKernelFreeMemBlock(m_memblock);
		m_memblock = -1;
		m_base = 0;
		return false;
	}

	m_dev = device;
	m_open = true;
	return true;
}

void VitaCamera::close()
{
	if (m_open && m_dev >= 0) {
		sceCameraStop(m_dev);
		sceCameraClose(m_dev);
	}
	m_open = false;
	m_dev = -1;

	if (m_memblock >= 0) {
		sceKernelFreeMemBlock(m_memblock);
		m_memblock = -1;
	}
	m_base = 0;
}

bool VitaCamera::readFrame()
{
	if (!m_open)
		return false;

	SceCameraRead rd;
	memset(&rd, 0, sizeof(rd));
	rd.size = sizeof(rd);
	rd.mode = 0; // 阻塞模式

	return sceCameraRead(m_dev, &rd) >= 0;
}
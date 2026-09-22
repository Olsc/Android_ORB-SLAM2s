#ifndef MENTHAAR_PSVITA_CAMERA_H
#define MENTHAAR_PSVITA_CAMERA_H

#include <stdint.h>

#include <psp2/types.h>

/* Thin wrapper around the SceCamera device. The frame is requested in ABGR
 * (32bpp) at 640x360, which maps 1:1 onto an A8B8G8R8 texture. */
class VitaCamera {
public:
	VitaCamera();
	~VitaCamera();

	bool open(int device /* 0 = front, 1 = rear */);
	void close();
	bool isOpen() const { return m_open; }
	int device() const { return m_dev; }

	/* Blocking read of the next frame. Returns true on success. */
	bool readFrame();

	const uint32_t *frame() const { return (const uint32_t *)m_base; }

private:
	int m_dev;
	bool m_open;
	SceUID m_memblock;
	void *m_base;
};

#endif /* MENTHAAR_PSVITA_CAMERA_H */

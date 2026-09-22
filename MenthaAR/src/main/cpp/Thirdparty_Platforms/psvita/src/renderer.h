#ifndef MENTHAAR_PSVITA_RENDERER_H
#define MENTHAAR_PSVITA_RENDERER_H

#include <stdint.h>
#include <vector>

#include <vita2d.h>

#include "math3d.h"
#include "pointcloud.h"

/* Software point-cloud rasterizer.
 *
 * Rendering happens into an off-screen A8B8G8R8 texture with a CPU z-buffer,
 * which is then composited (with alpha) over the live camera preview by
 * vita2d. This keeps the GPU load tiny and lets us draw arbitrarily many
 * points without one draw call per point.                                    */
class Renderer {
public:
	Renderer();

	bool init();
	void shutdown();

	/* Clears the color + depth buffers. clearColor should be transparent. */
	void beginFrame(uint32_t clearColor);

	void drawCloud(const PointCloud &cloud, const Mat3 &viewRot,
	               float camDist, float zoom, int splat);

	/* HUD helpers (drawn straight into the off-screen buffer). */
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
	int m_stride; /* in pixels */
	int m_w, m_h;
	std::vector<float> m_zbuf;
};

#endif /* MENTHAAR_PSVITA_RENDERER_H */

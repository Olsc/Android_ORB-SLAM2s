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
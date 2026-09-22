#include "renderer.h"
#include "config.h"
#include "font.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

Renderer::Renderer()
	: m_tex(0), m_pix(0), m_stride(0), m_w(SCR_W), m_h(SCR_H)
{
}

bool Renderer::init()
{
	m_tex = vita2d_create_empty_texture_format(SCR_W, SCR_H,
	                                           SCE_GXM_TEXTURE_FORMAT_A8B8G8R8);
	if (!m_tex)
		return false;

	// 点云使用最近邻过滤，避免双线性模糊。
	vita2d_texture_set_filters(m_tex, SCE_GXM_TEXTURE_FILTER_POINT,
	                           SCE_GXM_TEXTURE_FILTER_POINT);

	m_pix = (uint32_t *)vita2d_texture_get_datap(m_tex);
	m_stride = (int)(vita2d_texture_get_stride(m_tex) / 4);
	m_zbuf.assign((size_t)m_w * m_h, 1e30f);
	memset(m_pix, 0, (size_t)m_stride * m_h * 4);
	return true;
}

void Renderer::shutdown()
{
	if (m_tex) {
		vita2d_free_texture(m_tex);
		m_tex = 0;
	}
	m_pix = 0;
}

void Renderer::beginFrame(uint32_t clearColor)
{
	if (clearColor == 0) {
		memset(m_pix, 0, (size_t)m_stride * m_h * 4);
	} else {
		for (int y = 0; y < m_h; ++y) {
			uint32_t *row = m_pix + (size_t)y * m_stride;
			for (int x = 0; x < m_w; ++x)
				row[x] = clearColor;
		}
	}
	for (size_t i = 0; i < m_zbuf.size(); ++i)
		m_zbuf[i] = 1e30f;
}

void Renderer::drawCloud(const PointCloud &cloud, const Mat3 &viewRot,
                         float camDist, float zoom, int splat)
{
	const float fov = 60.0f * 3.1415926535f / 180.0f;
	const float focal = (0.5f * (float)m_h) / tanf(fov * 0.5f) * zoom;
	const float cx = m_w * 0.5f;
	const float cy = m_h * 0.5f;
	const float znear = 0.05f;
	const float zfar = camDist * 4.0f + 30.0f;

	if (splat < 0)
		splat = 0;
	if (splat > 4)
		splat = 4;

	const CloudPoint *pts = cloud.data();
	const size_t n = cloud.size();

	for (size_t i = 0; i < n; ++i) {
		const CloudPoint &p = pts[i];
		Vec3 v = mat3Mul(viewRot, vec3(p.x, p.y, p.z));
		v.z += camDist;
		if (v.z < znear || v.z > zfar)
			continue;

		float inv = focal / v.z;
		int sx = (int)(cx + v.x * inv);
		int sy = (int)(cy - v.y * inv);
		if (sx < -splat || sx >= m_w + splat || sy < -splat || sy >= m_h + splat)
			continue;

		for (int dy = -splat; dy <= splat; ++dy) {
			int yy = sy + dy;
			if (yy < 0 || yy >= m_h)
				continue;
			uint32_t *row = m_pix + (size_t)yy * m_stride;
			float *zrow = &m_zbuf[(size_t)yy * m_w];
			for (int dx = -splat; dx <= splat; ++dx) {
				int xx = sx + dx;
				if (xx < 0 || xx >= m_w)
					continue;
				if (v.z < zrow[xx]) {
					zrow[xx] = v.z;
					row[xx] = p.color;
				}
			}
		}
	}
}

void Renderer::putPixel(int x, int y, uint32_t color)
{
	if (x < 0 || x >= m_w || y < 0 || y >= m_h)
		return;
	uint32_t a = (color >> 24) & 0xFFu;
	if (a == 0)
		return;
	uint32_t *p = m_pix + (size_t)y * m_stride + x;
	if (a == 255) {
		*p = color;
		return;
	}
	uint32_t d = *p;
	uint32_t dr = d & 0xFFu, dg = (d >> 8) & 0xFFu, db = (d >> 16) & 0xFFu;
	uint32_t sr = color & 0xFFu, sg = (color >> 8) & 0xFFu, sb = (color >> 16) & 0xFFu;
	uint32_t r = (sr * a + dr * (255u - a)) / 255u;
	uint32_t g = (sg * a + dg * (255u - a)) / 255u;
	uint32_t b = (sb * a + db * (255u - a)) / 255u;
	*p = PACK_RGBA(r, g, b, 255);
}

void Renderer::blendPixel(int x, int y, uint32_t color, float alpha)
{
	if (alpha <= 0.0f)
		return;
	if (alpha >= 1.0f) {
		putPixel(x, y, color | 0xFF000000u);
		return;
	}
	if (x < 0 || x >= m_w || y < 0 || y >= m_h)
		return;
	uint32_t *p = m_pix + (size_t)y * m_stride + x;
	uint32_t d = *p;
	uint32_t dr = d & 0xFFu, dg = (d >> 8) & 0xFFu, db = (d >> 16) & 0xFFu;
	uint32_t sr = color & 0xFFu, sg = (color >> 8) & 0xFFu, sb = (color >> 16) & 0xFFu;
	float ia = 1.0f - alpha;
	uint32_t r = (uint32_t)(sr * alpha + dr * ia);
	uint32_t g = (uint32_t)(sg * alpha + dg * ia);
	uint32_t b = (uint32_t)(sb * alpha + db * ia);
	*p = PACK_RGBA(r, g, b, 255);
}

void Renderer::drawText(int x, int y, const char *text, uint32_t color, int scale)
{
	if (!text)
		return;
	if (scale < 1)
		scale = 1;

	const int startX = x;
	const unsigned char *s = (const unsigned char *)text;
	const int gw = psvDebugScreenFont.width;
	const int gh = psvDebugScreenFont.height;

	for (; *s; ++s) {
		unsigned char c = *s;
		if (c == '\n') {
			x = startX;
			y += gh * scale;
			continue;
		}
		if (c < psvDebugScreenFont.first || c > psvDebugScreenFont.last)
			c = '?';
		const unsigned char *g =
			psvDebugScreenFont.glyphs + (size_t)c * gh;
		for (int gy = 0; gy < gh; ++gy) {
			unsigned char bits = g[gy];
			if (!bits)
				continue;
			for (int gx = 0; gx < gw; ++gx) {
				if (!(bits & (0x80u >> gx)))
					continue;
				for (int sy = 0; sy < scale; ++sy)
					for (int sx = 0; sx < scale; ++sx)
						putPixel(x + gx * scale + sx,
						         y + gy * scale + sy, color);
			}
		}
		x += gw * scale;
	}
}

void Renderer::drawTextf(int x, int y, uint32_t color, int scale,
                         const char *fmt, ...)
{
	char buf[256];
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	drawText(x, y, buf, color, scale);
}

void Renderer::drawPanel(int x, int y, int w, int h, float alpha)
{
	uint32_t color = PACK_RGBA(8, 12, 20, 255);
	for (int yy = y; yy < y + h; ++yy)
		for (int xx = x; xx < x + w; ++xx)
			blendPixel(xx, yy, color, alpha);
}

void Renderer::drawMarker(int x, int y, uint32_t color, int radius)
{
	if (radius < 0)
		return;
	for (int dy = -radius; dy <= radius; ++dy)
		for (int dx = -radius; dx <= radius; ++dx)
			putPixel(x + dx, y + dy, color);
}
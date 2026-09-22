#include "pointcloud.h"

#include <math.h>
#include <string.h>

#include <psp2/io/fcntl.h>
#include <psp2/io/stat.h>

namespace {

struct CloudHeader {
	char magic[4];      // "MPC1"
	uint32_t version;
	uint32_t count;
	uint32_t stride;    // sizeof(CloudPoint)
};

} // namespace

PointCloud::PointCloud()
{
	m_points.reserve(MAX_POINTS);
}

void PointCloud::clear()
{
	m_points.clear();
}

bool PointCloud::add(const CloudPoint &p)
{
	if (m_points.size() >= MAX_POINTS)
		return false;
	m_points.push_back(p);
	return true;
}

bool PointCloud::save(const char *path) const
{
	// 确保目录存在，已存在则忽略。
	sceIoMkdir(SAVE_DIR, 0777);

	SceUID fd = sceIoOpen(path, SCE_O_WRONLY | SCE_O_CREAT | SCE_O_TRUNC, 0777);
	if (fd < 0)
		return false;

	CloudHeader h;
	memcpy(h.magic, "MPC1", 4);
	h.version = 1;
	h.count = (uint32_t)m_points.size();
	h.stride = (uint32_t)sizeof(CloudPoint);

	bool ok = sceIoWrite(fd, &h, sizeof(h)) == (int)sizeof(h);
	if (ok && h.count > 0) {
		size_t bytes = (size_t)h.count * sizeof(CloudPoint);
		ok = sceIoWrite(fd, &m_points[0], bytes) == (int)bytes;
	}

	sceIoClose(fd);
	return ok;
}

bool PointCloud::load(const char *path)
{
	SceUID fd = sceIoOpen(path, SCE_O_RDONLY, 0777);
	if (fd < 0)
		return false;

	CloudHeader h;
	if (sceIoRead(fd, &h, sizeof(h)) != (int)sizeof(h)) {
		sceIoClose(fd);
		return false;
	}

	if (memcmp(h.magic, "MPC1", 4) != 0 ||
	    h.stride != sizeof(CloudPoint) ||
	    h.count > MAX_POINTS) {
		sceIoClose(fd);
		return false;
	}

	std::vector<CloudPoint> tmp;
	tmp.resize(h.count);
	size_t bytes = (size_t)h.count * sizeof(CloudPoint);
	int rd = (h.count == 0) ? 0 : sceIoRead(fd, &tmp[0], bytes);
	sceIoClose(fd);

	if (rd != (int)bytes)
		return false;

	m_points.swap(tmp);
	return true;
}

void buildDemoCloud(PointCloud &cloud)
{
	cloud.clear();

	// 用 Lissajous 结加地面网格生成演示点云，色彩丰富、立体感强。
	const int knotPoints = 9000;
	for (int i = 0; i < knotPoints; ++i) {
		float t = (float)i / knotPoints * 6.2831853f * 3.0f;
		float x = 0.9f * sinf(3.0f * t);
		float y = 0.9f * sinf(4.0f * t + 1.5f);
		float z = 0.5f * sinf(5.0f * t + 3.0f);
		float r = 0.5f + 0.5f * sinf(t * 4.0f);
		float g = 0.5f + 0.5f * sinf(t * 4.0f + 2.1f);
		float b = 0.5f + 0.5f * sinf(t * 4.0f + 4.2f);
		CloudPoint p = { x, y, z,
			PACK_RGBA((uint32_t)(r * 255.0f), (uint32_t)(g * 255.0f),
			          (uint32_t)(b * 255.0f), 255) };
		cloud.add(p);

		// 叠加一份略暗的偏移副本，增加线宽。
		CloudPoint q = { x * 0.96f, y * 0.96f, z * 0.96f,
			PACK_RGBA((uint32_t)(r * 140.0f), (uint32_t)(g * 140.0f),
			          (uint32_t)(b * 140.0f), 255) };
		cloud.add(q);
	}

	// 地面网格。
	const int gridN = 28;
	for (int ix = -gridN; ix <= gridN; ++ix) {
		for (int iz = -gridN; iz <= gridN; ++iz) {
			float x = ix * 0.12f;
			float z = iz * 0.12f;
			float d = sqrtf(x * x + z * z);
			if (d > 1.6f)
				continue;
			float fade = 1.0f - d / 1.6f;
			uint32_t c = PACK_RGBA((uint32_t)(40 * fade + 10),
			                       (uint32_t)(120 * fade + 20),
			                       (uint32_t)(200 * fade + 30), 255);
			CloudPoint p = { x, -1.2f, z, c };
			cloud.add(p);
		}
	}
}
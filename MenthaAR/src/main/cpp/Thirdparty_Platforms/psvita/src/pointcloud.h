#ifndef MENTHAAR_PSVITA_POINTCLOUD_H
#define MENTHAAR_PSVITA_POINTCLOUD_H

#include <stddef.h>
#include <stdint.h>
#include <vector>

#include "config.h"

/* A single colored 3D point. Kept POD so it can be written/read as-is. */
struct CloudPoint {
	float x, y, z;
	uint32_t color; /* PACK_RGBA layout */
};

class PointCloud {
public:
	PointCloud();

	void clear();
	bool add(const CloudPoint &p);
	size_t size() const { return m_points.size(); }
	bool empty() const { return m_points.empty(); }
	const CloudPoint *data() const { return m_points.empty() ? 0 : &m_points[0]; }

	/* Persistence: a tiny binary format ("MPC1") in ux0:data/MenthaAR. */
	bool save(const char *path) const;
	bool load(const char *path);

private:
	std::vector<CloudPoint> m_points;
};

/* Fill the cloud with a procedural shape. Used as a fallback when no camera
 * is available (e.g. on an emulator) so that rendering / saving / loading can
 * still be exercised. */
void buildDemoCloud(PointCloud &cloud);

#endif /* MENTHAAR_PSVITA_POINTCLOUD_H */

#ifndef MENTHAAR_PSVITA_MATH3D_H
#define MENTHAAR_PSVITA_MATH3D_H

#include <cmath>

/* Minimal 3D math helpers (vectors, quaternions and 3x3 matrices).
 * Row-major matrices, column-vector convention:  r = M * v              */

struct Vec3 {
	float x, y, z;
};

inline Vec3 vec3(float x, float y, float z)
{
	Vec3 v = { x, y, z };
	return v;
}

inline Vec3 operator+(const Vec3 &a, const Vec3 &b)
{
	return vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

inline Vec3 operator-(const Vec3 &a, const Vec3 &b)
{
	return vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

inline Vec3 operator*(const Vec3 &a, float s)
{
	return vec3(a.x * s, a.y * s, a.z * s);
}

struct Quat {
	float x, y, z, w;
};

inline Quat quatNorm(Quat q)
{
	float n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
	if (n < 1e-8f)
		return Quat{ 0.0f, 0.0f, 0.0f, 1.0f };
	float inv = 1.0f / n;
	return Quat{ q.x * inv, q.y * inv, q.z * inv, q.w * inv };
}

inline Quat quatConj(Quat q)
{
	return Quat{ -q.x, -q.y, -q.z, q.w };
}

inline Quat quatMul(Quat a, Quat b)
{
	return Quat{
		a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
		a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
		a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
		a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
	};
}

inline Vec3 quatRotate(Quat q, Vec3 v)
{
	Quat qv{ v.x, v.y, v.z, 0.0f };
	Quat t = quatMul(q, qv);
	Quat r = quatMul(t, quatConj(q));
	return vec3(r.x, r.y, r.z);
}

struct Mat3 {
	float m[9];
};

inline Mat3 mat3Identity()
{
	Mat3 I = { { 1, 0, 0, 0, 1, 0, 0, 0, 1 } };
	return I;
}

inline Vec3 mat3Mul(const Mat3 &A, const Vec3 &v)
{
	return vec3(
		A.m[0] * v.x + A.m[1] * v.y + A.m[2] * v.z,
		A.m[3] * v.x + A.m[4] * v.y + A.m[5] * v.z,
		A.m[6] * v.x + A.m[7] * v.y + A.m[8] * v.z);
}

inline Mat3 mat3Mul(const Mat3 &A, const Mat3 &B)
{
	Mat3 C;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			float s = 0.0f;
			for (int k = 0; k < 3; ++k)
				s += A.m[i * 3 + k] * B.m[k * 3 + j];
			C.m[i * 3 + j] = s;
		}
	}
	return C;
}

inline Mat3 quatToMat(Quat q)
{
	q = quatNorm(q);
	float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
	float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
	float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
	Mat3 M;
	M.m[0] = 1.0f - 2.0f * (yy + zz); M.m[1] = 2.0f * (xy - wz);       M.m[2] = 2.0f * (xz + wy);
	M.m[3] = 2.0f * (xy + wz);        M.m[4] = 1.0f - 2.0f * (xx + zz); M.m[5] = 2.0f * (yz - wx);
	M.m[6] = 2.0f * (xz - wy);        M.m[7] = 2.0f * (yz + wx);       M.m[8] = 1.0f - 2.0f * (xx + yy);
	return M;
}

/* Orbit matrix: yaw around Y, then pitch around X. */
inline Mat3 eulerYawPitch(float yaw, float pitch)
{
	float cy = std::cos(yaw), sy = std::sin(yaw);
	float cp = std::cos(pitch), sp = std::sin(pitch);
	Mat3 Ry = { { cy, 0, sy, 0, 1, 0, -sy, 0, cy } };
	Mat3 Rx = { { 1, 0, 0, 0, cp, -sp, 0, sp, cp } };
	return mat3Mul(Ry, Rx);
}

#endif /* MENTHAAR_PSVITA_MATH3D_H */

/*
 * PS Vita calib3d compatibility implementation (see header for rationale).
 *
 * Everything here operates on plain double/Matx math so the implementation is
 * independent of the OpenCV build configurability.  The algorithms are the
 * textbook ones used by OpenCV: Rodrigues' formula, iterative undistortion,
 * direct linear transform (DLT) for the initial pose, Levenberg-Marquardt
 * refinement on the reprojection error, and a RANSAC wrapper.
 */

#include "opencv_calib3d_compat.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace {

using cv::Matx33d;
using cv::Vec3d;
using cv::Point2d;
using cv::Point3d;
using cv::InputArray;
using cv::OutputArray;

inline double det3(const Matx33d &m)
{
	return m(0, 0) * (m(1, 1) * m(2, 2) - m(1, 2) * m(2, 1))
	     - m(0, 1) * (m(1, 0) * m(2, 2) - m(1, 2) * m(2, 0))
	     + m(0, 2) * (m(1, 0) * m(2, 1) - m(1, 1) * m(2, 0));
}

inline Matx33d rodriguesVecToMat(const Vec3d &r)
{
	double theta = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
	if (theta < 1e-12)
		return Matx33d(1, 0, 0, 0, 1, 0, 0, 0, 1);

	double kx = r[0] / theta, ky = r[1] / theta, kz = r[2] / theta;
	double c = std::cos(theta), s = std::sin(theta), C = 1.0 - c;

	Matx33d R;
	R(0, 0) = c + kx * kx * C;
	R(0, 1) = kx * ky * C - kz * s;
	R(0, 2) = kx * kz * C + ky * s;
	R(1, 0) = ky * kx * C + kz * s;
	R(1, 1) = c + ky * ky * C;
	R(1, 2) = ky * kz * C - kx * s;
	R(2, 0) = kz * kx * C - ky * s;
	R(2, 1) = kz * ky * C + kx * s;
	R(2, 2) = c + kz * kz * C;
	return R;
}

inline Vec3d rodriguesMatToVec(const Matx33d &R)
{
	double tr = R(0, 0) + R(1, 1) + R(2, 2);
	double c = (tr - 1.0) * 0.5;
	if (c > 1.0) c = 1.0;
	if (c < -1.0) c = -1.0;
	double theta = std::acos(c);
	if (theta < 1e-12)
		return Vec3d(0, 0, 0);
	if (theta < 3.14159265358979323846 - 1e-6) {
		double k = theta / (2.0 * std::sin(theta));
		return Vec3d(k * (R(2, 1) - R(1, 2)),
		             k * (R(0, 2) - R(2, 0)),
		             k * (R(1, 0) - R(0, 1)));
	}
	/* theta close to pi: recover axis from the diagonal. */
	double xx = (R(0, 0) + 1.0) * 0.5;
	double yy = (R(1, 1) + 1.0) * 0.5;
	double zz = (R(2, 2) + 1.0) * 0.5;
	double xy = (R(0, 1) + R(1, 0)) * 0.25;
	double xz = (R(0, 2) + R(2, 0)) * 0.25;
	double yz = (R(1, 2) + R(2, 1)) * 0.25;
	double x, y, z;
	if (xx >= yy && xx >= zz) {
		x = std::sqrt(std::max(0.0, xx));
		if (x > 1e-8) { y = xy / x; z = xz / x; } else { y = 1.0; z = 0.0; }
	} else if (yy >= zz) {
		y = std::sqrt(std::max(0.0, yy));
		if (y > 1e-8) { x = xy / y; z = yz / y; } else { x = 1.0; z = 0.0; }
	} else {
		z = std::sqrt(std::max(0.0, zz));
		if (z > 1e-8) { x = xz / z; y = yz / z; } else { x = 0.0; y = 1.0; }
	}
	Vec3d axis(x, y, z);
	double n = std::sqrt(axis.dot(axis));
	if (n > 1e-12)
		axis *= (theta / n);
	return axis;
}

inline bool projectNormalized(const Point3d &P, const Matx33d &R, const Vec3d &t,
                              double &u, double &v)
{
	Vec3d Xc = R * Vec3d(P.x, P.y, P.z) + t;
	if (Xc[2] <= 1e-9)
		return false;
	u = Xc[0] / Xc[2];
	v = Xc[1] / Xc[2];
	return true;
}

/* Six by six dense solve with partial pivoting. */
bool solve6(double A[6][6], double b[6], double x[6])
{
	int idx[6];
	for (int i = 0; i < 6; ++i)
		idx[i] = i;

	for (int col = 0; col < 6; ++col) {
		int piv = col;
		double best = std::fabs(A[col][col]);
		for (int r = col + 1; r < 6; ++r) {
			if (std::fabs(A[r][col]) > best) {
				best = std::fabs(A[r][col]);
				piv = r;
			}
		}
		if (best < 1e-18)
			return false;
		if (piv != col) {
			for (int c = 0; c < 6; ++c)
				std::swap(A[col][c], A[piv][c]);
			std::swap(b[col], b[piv]);
			std::swap(idx[col], idx[piv]);
		}
		double d = A[col][col];
		for (int r = col + 1; r < 6; ++r) {
			double f = A[r][col] / d;
			if (f == 0.0)
				continue;
			for (int c = col; c < 6; ++c)
				A[r][c] -= f * A[col][c];
			b[r] -= f * b[col];
		}
	}

	for (int i = 5; i >= 0; --i) {
		double s = b[i];
		for (int c = i + 1; c < 6; ++c)
			s -= A[i][c] * x[c];
		x[i] = s / A[i][i];
	}
	return true;
}

/* Extract strongly typed point vectors from OpenCV InputArrays. */
bool extractObj(InputArray _op, std::vector<Point3d> &out)
{
	cv::Mat m = _op.getMat();
	int n = m.checkVector(3);
	if (n <= 0)
		return false;
	out.resize(n);
	if (m.depth() == CV_32F) {
		for (int i = 0; i < n; ++i) {
			cv::Vec3f v = m.at<cv::Vec3f>(i);
			out[i] = Point3d(v[0], v[1], v[2]);
		}
	} else {
		for (int i = 0; i < n; ++i) {
			cv::Vec3d v = m.at<cv::Vec3d>(i);
			out[i] = Point3d(v[0], v[1], v[2]);
		}
	}
	return true;
}

bool extractImg(InputArray _ip, std::vector<Point2d> &out)
{
	cv::Mat m = _ip.getMat();
	int n = m.checkVector(2);
	if (n <= 0)
		return false;
	out.resize(n);
	if (m.depth() == CV_32F) {
		for (int i = 0; i < n; ++i) {
			cv::Vec2f v = m.at<cv::Vec2f>(i);
			out[i] = Point2d(v[0], v[1]);
		}
	} else {
		for (int i = 0; i < n; ++i) {
			cv::Vec2d v = m.at<cv::Vec2d>(i);
			out[i] = Point2d(v[0], v[1]);
		}
	}
	return true;
}

/* Direct linear transform on normalised image coordinates. */
bool dlt(const std::vector<Point3d> &obj, const std::vector<Point2d> &imgN,
         Matx33d &R, Vec3d &t)
{
	int n = (int)obj.size();
	cv::Mat A(2 * n, 12, CV_64F);
	A.setTo(0);
	for (int i = 0; i < n; ++i) {
		double X = obj[i].x, Y = obj[i].y, Z = obj[i].z;
		double x = imgN[i].x, y = imgN[i].y;
		double *r0 = A.ptr<double>(2 * i);
		double *r1 = A.ptr<double>(2 * i + 1);
		r0[0] = X; r0[1] = Y; r0[2] = Z; r0[3] = 1.0;
		r0[8] = -x * X; r0[9] = -x * Y; r0[10] = -x * Z; r0[11] = -x;
		r1[4] = X; r1[5] = Y; r1[6] = Z; r1[7] = 1.0;
		r1[8] = -y * X; r1[9] = -y * Y; r1[10] = -y * Z; r1[11] = -y;
	}

	cv::Mat p;
	cv::SVD::solveZ(A, p);
	if (p.empty())
		return false;

	cv::Mat M = p.reshape(1, 3); /* 3 x 4 */
	cv::Mat Rm = M.colRange(0, 3).clone();
	cv::Mat tm = M.col(3).clone();

	double scale = 0.0;
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j) {
			double v = Rm.at<double>(i, j);
			scale += v * v;
		}
	scale = std::sqrt(scale);
	if (scale < 1e-12)
		return false;

	Matx33d R0;
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			R0(i, j) = Rm.at<double>(i, j) / scale;

	cv::SVD svd(cv::Mat(R0), cv::SVD::FULL_UV);
	cv::Mat U = svd.u, Vt = svd.vt;
	cv::Mat Rm2 = U * Vt;
	Matx33d Rr;
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			Rr(i, j) = Rm2.at<double>(i, j);
	if (det3(Rr) < 0.0) {
		/* Flip to a proper rotation. */
		for (int i = 0; i < 3; ++i)
			Rr(i, 2) = -Rr(i, 2);
	}

	R = Rr;
	t = Vec3d(tm.at<double>(0) / scale,
	          tm.at<double>(1) / scale,
	          tm.at<double>(2) / scale);
	return true;
}

void computeResiduals(const std::vector<Point3d> &obj,
                      const std::vector<Point2d> &imgN,
                      const double *params, std::vector<double> &res)
{
	Vec3d rv(params[0], params[1], params[2]);
	Matx33d R = rodriguesVecToMat(rv);
	Vec3d t(params[3], params[4], params[5]);
	for (size_t i = 0; i < obj.size(); ++i) {
		double u, v;
		if (!projectNormalized(obj[i], R, t, u, v)) {
			res[2 * i] = 1e3;
			res[2 * i + 1] = 1e3;
			continue;
		}
		res[2 * i] = u - imgN[i].x;
		res[2 * i + 1] = v - imgN[i].y;
	}
}

double residualCost(const std::vector<double> &res)
{
	double c = 0.0;
	for (size_t i = 0; i < res.size(); ++i)
		c += res[i] * res[i];
	return c;
}

/* Levenberg-Marquardt refinement on normalised reprojection error. */
void refine(const std::vector<Point3d> &obj, const std::vector<Point2d> &imgN,
            Matx33d &R, Vec3d &t, int maxIter = 40)
{
	if (obj.size() < 4)
		return;

	Vec3d rv = rodriguesMatToVec(R);
	double params[6] = { rv[0], rv[1], rv[2], t[0], t[1], t[2] };
	std::vector<double> res(2 * obj.size()), resP(2 * obj.size());

	computeResiduals(obj, imgN, params, res);
	double cost = residualCost(res);
	double lambda = 1e-3;

	for (int iter = 0; iter < maxIter; ++iter) {
		double J[6][6];
		std::vector<double> Jn(2 * obj.size() * 6);
		for (int k = 0; k < 6; ++k) {
			double old = params[k];
			double h = 1e-6 * (std::fabs(old) + 1.0);
			params[k] = old + h;
			computeResiduals(obj, imgN, params, resP);
			params[k] = old;
			for (size_t i = 0; i < res.size(); ++i)
				Jn[i * 6 + k] = (resP[i] - res[i]) / h;
		}

		double JtJ[6][6], Jtr[6];
		for (int a = 0; a < 6; ++a) {
			Jtr[a] = 0.0;
			for (int b = 0; b < 6; ++b)
				JtJ[a][b] = 0.0;
		}
		for (size_t i = 0; i < res.size(); ++i) {
			for (int a = 0; a < 6; ++a) {
				double va = Jn[i * 6 + a];
				Jtr[a] += va * res[i];
				for (int b = 0; b < 6; ++b)
					JtJ[a][b] += va * Jn[i * 6 + b];
			}
		}
		for (int a = 0; a < 6; ++a)
			JtJ[a][a] *= (1.0 + lambda);

		double rhs[6];
		for (int a = 0; a < 6; ++a)
			rhs[a] = -Jtr[a];
		double dx[6] = { 0 };
		if (!solve6(JtJ, rhs, dx))
			break;

		double trial[6];
		for (int a = 0; a < 6; ++a)
			trial[a] = params[a] + dx[a];
		computeResiduals(obj, imgN, trial, resP);
		double trialCost = residualCost(resP);

		if (trialCost < cost) {
			std::memcpy(params, trial, sizeof(params));
			res.swap(resP);
			cost = trialCost;
			lambda *= 0.5;
			if (lambda < 1e-10)
				lambda = 1e-10;
		} else {
			lambda *= 2.0;
			if (lambda > 1e10)
				break;
		}
	}

	R = rodriguesVecToMat(Vec3d(params[0], params[1], params[2]));
	t = Vec3d(params[3], params[4], params[5]);
}

double pixelReprojectionError(const Point3d &P, const Matx33d &R, const Vec3d &t,
                              double fx, double fy, double cx, double cy,
                              const Point2d &obs)
{
	Vec3d Xc = R * Vec3d(P.x, P.y, P.z) + t;
	if (Xc[2] <= 1e-9)
		return 1e9;
	double u = fx * Xc[0] / Xc[2] + cx;
	double v = fy * Xc[1] / Xc[2] + cy;
	double du = u - obs.x, dv = v - obs.y;
	return std::sqrt(du * du + dv * dv);
}

bool solvePnPImpl(const std::vector<Point3d> &obj, const std::vector<Point2d> &imgN,
                  OutputArray _rvec, OutputArray _tvec)
{
	int n = (int)obj.size();
	if (n < 6)
		return false; /* DLT-based path requires >= 6 points */

	Matx33d R;
	Vec3d t;
	if (!dlt(obj, imgN, R, t))
		return false;
	refine(obj, imgN, R, t);

	cv::Mat rvec(3, 1, CV_64F);
	Vec3d rv = rodriguesMatToVec(R);
	rvec.at<double>(0) = rv[0];
	rvec.at<double>(1) = rv[1];
	rvec.at<double>(2) = rv[2];
	cv::Mat tvec(3, 1, CV_64F);
	tvec.at<double>(0) = t[0];
	tvec.at<double>(1) = t[1];
	tvec.at<double>(2) = t[2];
	rvec.copyTo(_rvec);
	tvec.copyTo(_tvec);
	return true;
}

} /* namespace */

namespace cv {

void Rodrigues(InputArray _src, OutputArray _dst, OutputArray)
{
	Mat src = _src.getMat();
	if (src.empty())
		return;
	bool isVec = (src.total() == 3);
	int depth = src.depth();

	if (isVec) {
		double r[3];
		Mat s = src.reshape(1, 3);
		if (depth == CV_32F) {
			for (int i = 0; i < 3; ++i) r[i] = s.at<float>(i);
		} else {
			for (int i = 0; i < 3; ++i) r[i] = s.at<double>(i);
		}
		Matx33d R = rodriguesVecToMat(Vec3d(r[0], r[1], r[2]));
		Mat out(3, 3, depth);
		if (depth == CV_32F)
			for (int i = 0; i < 3; ++i)
				for (int j = 0; j < 3; ++j)
					out.at<float>(i, j) = (float)R(i, j);
		else
			for (int i = 0; i < 3; ++i)
				for (int j = 0; j < 3; ++j)
					out.at<double>(i, j) = R(i, j);
		out.copyTo(_dst);
	} else {
		Mat s = src;
		double R[3][3];
		if (depth == CV_32F)
			for (int i = 0; i < 3; ++i)
				for (int j = 0; j < 3; ++j)
					R[i][j] = s.at<float>(i, j);
		else
			for (int i = 0; i < 3; ++i)
				for (int j = 0; j < 3; ++j)
					R[i][j] = s.at<double>(i, j);
		Matx33d Rm;
		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 3; ++j)
				Rm(i, j) = R[i][j];
		Vec3d rv = rodriguesMatToVec(Rm);
		Mat out(3, 1, depth);
		if (depth == CV_32F)
			for (int i = 0; i < 3; ++i) out.at<float>(i) = (float)rv[i];
		else
			for (int i = 0; i < 3; ++i) out.at<double>(i) = rv[i];
		out.copyTo(_dst);
	}
}

void undistortPoints(InputArray _src, OutputArray _dst, InputArray Kin,
                     InputArray Din, InputArray Rin, InputArray Pin)
{
	Mat src = _src.getMat().clone();
	int n = src.checkVector(2);
	if (n <= 0)
		return;
	int depth = src.depth();

	Mat K = Kin.getMat();
	double fx, fy, cx, cy;
	if (K.depth() == CV_32F) {
		fx = K.at<float>(0, 0); fy = K.at<float>(1, 1);
		cx = K.at<float>(0, 2); cy = K.at<float>(1, 2);
	} else {
		fx = K.at<double>(0, 0); fy = K.at<double>(1, 1);
		cx = K.at<double>(0, 2); cy = K.at<double>(1, 2);
	}

	double k1 = 0, k2 = 0, p1 = 0, p2 = 0, k3 = 0;
	if (!Din.empty()) {
		Mat D = Din.getMat();
		int dn = (int)D.total();
		const float *df = (D.depth() == CV_32F) ? D.ptr<float>(0) : 0;
		const double *dd = (D.depth() == CV_64F) ? D.ptr<double>(0) : 0;
		double dv[5] = { 0, 0, 0, 0, 0 };
		for (int i = 0; i < dn && i < 5; ++i)
			dv[i] = df ? df[i] : dd[i];
		k1 = dv[0]; k2 = dv[1]; p1 = dv[2]; p2 = dv[3]; k3 = dv[4];
	}

	Mat R = Rin.empty() ? Mat() : Rin.getMat();
	Mat P = Pin.empty() ? Mat() : Pin.getMat();

	Mat dst(n, 1, CV_MAKETYPE(depth, 2));
	for (int i = 0; i < n; ++i) {
		double u, v;
		if (depth == CV_32F) {
			Vec2f val = src.at<Vec2f>(i);
			u = val[0]; v = val[1];
		} else {
			Vec2d val = src.at<Vec2d>(i);
			u = val[0]; v = val[1];
		}
		double x = (u - cx) / fx;
		double y = (v - cy) / fy;

		if (k1 != 0 || k2 != 0 || p1 != 0 || p2 != 0 || k3 != 0) {
			double x0 = x, y0 = y;
			for (int it = 0; it < 8; ++it) {
				double r2 = x0 * x0 + y0 * y0;
				double radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
				double dx = 2.0 * p1 * x0 * y0 + p2 * (r2 + 2.0 * x0 * x0);
				double dy = p1 * (r2 + 2.0 * y0 * y0) + 2.0 * p2 * x0 * y0;
				x0 = (x - dx) / radial;
				y0 = (y - dy) / radial;
			}
			x = x0; y = y0;
		}

		if (!R.empty()) {
			double X = x, Y = y, Z = 1.0;
			double xr = 0, yr = 0, zr = 0;
			for (int r = 0; r < 3; ++r) {
				double a = (R.depth() == CV_32F) ? R.at<float>(r, 0) : R.at<double>(r, 0);
				double b = (R.depth() == CV_32F) ? R.at<float>(r, 1) : R.at<double>(r, 1);
				double c = (R.depth() == CV_32F) ? R.at<float>(r, 2) : R.at<double>(r, 2);
				double val = a * X + b * Y + c * Z;
				if (r == 0) xr = val;
				else if (r == 1) yr = val;
				else zr = val;
			}
			x = xr / zr;
			y = yr / zr;
		}

		if (!P.empty()) {
			double fx2 = (P.depth() == CV_32F) ? P.at<float>(0, 0) : P.at<double>(0, 0);
			double fy2 = (P.depth() == CV_32F) ? P.at<float>(1, 1) : P.at<double>(1, 1);
			double cx2 = (P.depth() == CV_32F) ? P.at<float>(0, 2) : P.at<double>(0, 2);
			double cy2 = (P.depth() == CV_32F) ? P.at<float>(1, 2) : P.at<double>(1, 2);
			u = fx2 * x + cx2;
			v = fy2 * y + cy2;
		} else {
			u = x;
			v = y;
		}

		if (depth == CV_32F)
			dst.at<Vec2f>(i) = Vec2f((float)u, (float)v);
		else
			dst.at<Vec2d>(i) = Vec2d(u, v);
	}
	dst.copyTo(_dst);
}

bool solvePnP(InputArray objectPoints, InputArray imagePoints,
              InputArray cameraMatrix, InputArray distCoeffs,
              OutputArray rvec, OutputArray tvec,
              bool useExtrinsicGuess, int)
{
	std::vector<Point3d> obj;
	std::vector<Point2d> img;
	if (!extractObj(objectPoints, obj) || !extractImg(imagePoints, img))
		return false;
	if (obj.size() != img.size())
		return false;

	/* Normalise the image points using the intrinsics / distortion. */
	std::vector<Point2d> imgN;
	{
		cv::Mat src((int)img.size(), 1, CV_64FC2);
		for (size_t i = 0; i < img.size(); ++i)
			src.at<Vec2d>((int)i) = Vec2d(img[i].x, img[i].y);
		Mat dst;
		undistortPoints(src, dst, cameraMatrix, distCoeffs);
		imgN.resize(img.size());
		for (size_t i = 0; i < img.size(); ++i) {
			Vec2d v = dst.at<Vec2d>((int)i);
			imgN[i] = Point2d(v[0], v[1]);
		}
	}

	(void)useExtrinsicGuess;
	return solvePnPImpl(obj, imgN, rvec, tvec);
}

bool solvePnPRansac(InputArray objectPoints, InputArray imagePoints,
                    InputArray cameraMatrix, InputArray distCoeffs,
                    OutputArray rvec, OutputArray tvec,
                    bool useExtrinsicGuess, int iterationsCount,
                    float reprojectionError, double confidence,
                    OutputArray inliers, int flags)
{
	std::vector<Point3d> obj;
	std::vector<Point2d> img;
	if (!extractObj(objectPoints, obj) || !extractImg(imagePoints, img))
		return false;
	if (obj.size() != img.size())
		return false;
	int n = (int)obj.size();
	if (n < 6)
		return solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs,
		                rvec, tvec, useExtrinsicGuess, flags);

	Mat K = cameraMatrix.getMat();
	double fx = K.at<double>(0, 0), fy = K.at<double>(1, 1);
	double cx = K.at<double>(0, 2), cy = K.at<double>(1, 2);
	if (K.depth() == CV_32F) {
		fx = K.at<float>(0, 0); fy = K.at<float>(1, 1);
		cx = K.at<float>(0, 2); cy = K.at<float>(1, 2);
	}

	std::vector<Point2d> imgN(n);
	{
		cv::Mat src(n, 1, CV_64FC2);
		for (int i = 0; i < n; ++i)
			src.at<Vec2d>(i) = Vec2d(img[i].x, img[i].y);
		Mat dst;
		undistortPoints(src, dst, cameraMatrix, distCoeffs);
		for (int i = 0; i < n; ++i) {
			Vec2d v = dst.at<Vec2d>(i);
			imgN[i] = Point2d(v[0], v[1]);
		}
	}

	(void)confidence;

	int maxIter = iterationsCount > 0 ? iterationsCount : 100;
	double th = reprojectionError > 0 ? reprojectionError : 3.0;

	std::vector<int> bestInliers;
	Matx33d bestR;
	Vec3d bestT;
	int bestCount = 0;
	std::srand(0x5eed1234);

	for (int iter = 0; iter < maxIter; ++iter) {
		int idx[6];
		for (int k = 0; k < 6; ++k) {
			bool dup;
			do {
				idx[k] = std::rand() % n;
				dup = false;
				for (int j = 0; j < k; ++j)
					if (idx[j] == idx[k]) { dup = true; break; }
			} while (dup);
		}
		std::vector<Point3d> so(6);
		std::vector<Point2d> si(6);
		for (int k = 0; k < 6; ++k) {
			so[k] = obj[idx[k]];
			si[k] = imgN[idx[k]];
		}
		Matx33d R;
		Vec3d t;
		if (!dlt(so, si, R, t))
			continue;

		std::vector<int> inl;
		for (int i = 0; i < n; ++i) {
			if (pixelReprojectionError(obj[i], R, t, fx, fy, cx, cy, img[i]) < th)
				inl.push_back(i);
		}
		if ((int)inl.size() > bestCount) {
			bestCount = (int)inl.size();
			bestInliers = inl;
			bestR = R;
			bestT = t;
		}
	}

	if (bestCount < 6) {
		if (!inliers.empty())
			inliers.release();
		return false;
	}

	/* Refine on all inliers. */
	std::vector<Point3d> ro;
	std::vector<Point2d> ri;
	ro.reserve(bestInliers.size());
	ri.reserve(bestInliers.size());
	for (size_t i = 0; i < bestInliers.size(); ++i) {
		ro.push_back(obj[bestInliers[i]]);
		ri.push_back(imgN[bestInliers[i]]);
	}
	refine(ro, ri, bestR, bestT);

	/* Recompute the inlier set with the refined pose. */
	std::vector<int> finalInliers;
	for (int i = 0; i < n; ++i) {
		if (pixelReprojectionError(obj[i], bestR, bestT, fx, fy, cx, cy, img[i]) < th)
			finalInliers.push_back(i);
	}

	Vec3d rv = rodriguesMatToVec(bestR);
	Mat rvecOut(3, 1, CV_64F);
	rvecOut.at<double>(0) = rv[0];
	rvecOut.at<double>(1) = rv[1];
	rvecOut.at<double>(2) = rv[2];
	Mat tvecOut(3, 1, CV_64F);
	tvecOut.at<double>(0) = bestT[0];
	tvecOut.at<double>(1) = bestT[1];
	tvecOut.at<double>(2) = bestT[2];
	rvecOut.copyTo(rvec);
	tvecOut.copyTo(tvec);

	if (!inliers.empty()) {
		Mat inl(1, (int)finalInliers.size(), CV_32S);
		for (size_t i = 0; i < finalInliers.size(); ++i)
			inl.at<int>((int)i) = finalInliers[i];
		inl.copyTo(inliers);
	}
	return true;
}

} /* namespace cv */

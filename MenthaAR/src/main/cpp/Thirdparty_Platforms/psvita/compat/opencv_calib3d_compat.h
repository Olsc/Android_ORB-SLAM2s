#ifndef MENTHAAR_PSVITA_OPENCV_CALIB3D_COMPAT_H
#define MENTHAAR_PSVITA_OPENCV_CALIB3D_COMPAT_H

#include <opencv2/core.hpp>

#include <vector>

namespace cv {

enum SolvePnPMethod {
	SOLVEPNP_ITERATIVE   = 0,
	SOLVEPNP_EPNP        = 1,
	SOLVEPNP_P3P         = 2,
	SOLVEPNP_AP3P        = 3,
	SOLVEPNP_IPPE        = 4,
	SOLVEPNP_IPPE_SQUARE = 5,
	SOLVEPNP_SQPNP       = 6,
	SOLVEPNP_MAX_COUNT
};

CV_EXPORTS void Rodrigues(InputArray src, OutputArray dst,
                          OutputArray jacobian = noArray());

CV_EXPORTS void undistortPoints(InputArray src, OutputArray dst,
                                InputArray cameraMatrix, InputArray distCoeffs,
                                InputArray R = noArray(),
                                InputArray P = noArray());

CV_EXPORTS bool solvePnP(InputArray objectPoints, InputArray imagePoints,
                         InputArray cameraMatrix, InputArray distCoeffs,
                         OutputArray rvec, OutputArray tvec,
                         bool useExtrinsicGuess = false,
                         int flags = SOLVEPNP_ITERATIVE);

CV_EXPORTS bool solvePnPRansac(InputArray objectPoints, InputArray imagePoints,
                               InputArray cameraMatrix, InputArray distCoeffs,
                               OutputArray rvec, OutputArray tvec,
                               bool useExtrinsicGuess = false,
                               int iterationsCount = 100,
                               float reprojectionError = 8.0f,
                               double confidence = 0.99,
                               OutputArray inliers = noArray(),
                               int flags = SOLVEPNP_ITERATIVE);

} // namespace cv

#endif // MENTHAAR_PSVITA_OPENCV_CALIB3D_COMPAT_H
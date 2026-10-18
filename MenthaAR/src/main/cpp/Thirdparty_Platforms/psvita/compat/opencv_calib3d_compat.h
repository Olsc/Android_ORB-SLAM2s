/**
 * Copyright (C) 2025-2026 Olsc <OlscStudio@outlook.com>
 *
 * This file is part of the Android ORB-SLAM2s project (a fork of ORB-SLAM2).
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

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
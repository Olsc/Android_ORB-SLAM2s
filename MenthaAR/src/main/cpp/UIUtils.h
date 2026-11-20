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

/**
 * Created by Ads on 2017/1/15.
 * 由Olsc于2025/8/25开始进行修改
 */

// UI 工具：RANSAC 平面检测与 OpenCV 到 OpenGL 的矩阵转换

#ifndef UTILS_H
#define UTILS_H

#include "Common.h"
#include <opencv2/opencv.hpp>
#include "include/System.h"
#include "Plane.h"
#ifdef ANDROID
#include <GLES/gl.h>
#endif

// 使用RANSAC算法检测平面，失败返回NULL
Plane* detectPlane(const cv::Mat Tcw, const std::vector<ORB_SLAM2::MapPoint*> &vMPs, const int iterations);

// 带重力先验的水平面检测：upWorld 为 SLAM 世界系下指向天空的单位向量(3x1 float)。
// 按沿重力方向的高度聚类，选取点数足够且最低的一簇作为地面；条件不满足时返回 nullptr，由调用方回退到纯视觉 RANSAC
Plane* detectHorizontalPlane(const cv::Mat Tcw, const std::vector<ORB_SLAM2::MapPoint*> &vMPs, const cv::Mat &upWorld);

// 将OpenCV的Mat矩阵转换为OpenGL的列主序矩阵
void getColMajorMatrixFromMat(float M[],cv::Mat &img);

#endif // UTILS_H
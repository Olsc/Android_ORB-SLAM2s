/**
 * Created by Ads on 2017/1/15.
 * 由Olsc于2025/8/25开始进行修改
 */

// UI 工具：RANSAC 平面检测与 OpenCV→OpenGL 矩阵转换

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

// 将OpenCV的Mat矩阵转换为OpenGL的列主序矩阵
void getColMajorMatrixFromMat(float M[],cv::Mat &img);

#endif //ORB_SLAM2_AR_PLANE_H
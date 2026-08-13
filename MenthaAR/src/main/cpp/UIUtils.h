/**
 * Created by Ads on 2017/1/15.
 * 由Olsc于2025/8/25开始进行修改
 */

// UI工具函数模块：提供图像绘制、平面检测、投影矩阵计算等工具函数

#ifndef UTILS_H
#define UTILS_H

#include "Common.h"
#include <opencv2/opencv.hpp>
#include "include/System.h"
#include "Plane.h"
#ifdef ANDROID
#include <GLES/gl.h>
#endif

// 绘制当前帧跟踪到的特征点，蓝=新建、绿=匹配的已加载、红=未匹配的已加载
void drawTrackedPoints(const std::vector<cv::KeyPoint> &vKeys, const std::vector<ORB_SLAM2::MapPoint *> &vMPs,
                       cv::Mat &im, float cx = 0.0f, float cy = 0.0f);

// 绘制所有地图点，用于AR重定位时显示完整点云
void drawAllMapPoints(const cv::Mat &Tcw, const std::vector<ORB_SLAM2::MapPoint*> &allMapPoints,
                      cv::Mat &im, float fx, float fy, float cx, float cy, bool drawOnlyLoaded = true);

// 使用RANSAC算法检测平面，失败返回NULL
Plane* detectPlane(const cv::Mat Tcw, const std::vector<ORB_SLAM2::MapPoint*> &vMPs, const int iterations);

// 将OpenCV的Mat矩阵转换为OpenGL的列主序矩阵
void getColMajorMatrixFromMat(float M[],cv::Mat &img);

#endif //ORB_SLAM2_AR_PLANE_H
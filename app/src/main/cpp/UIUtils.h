/**
 * Created by Ads on 2017/1/15.
 * 由Olsc于2025/8/25开始进行修改
 */

/**
 * UI工具函数模块
 * 功能描述: 提供图像绘制、平面检测、投影矩阵计算等UI相关工具函数
 */

#ifndef UTILS_H
#define UTILS_H

#include "Common.h"
#include <opencv2/opencv.hpp>
#include "include/System.h"
#include "Plane.h"
#include <GLES/gl.h>

/**
 * 绘制当前帧跟踪到的特征点
 * 颜色编码: 蓝色-新建点, 绿色-成功匹配的已加载点, 红色-未匹配的已加载点
 * @param vKeys 关键点列表
 * @param vMPs 地图点列表
 * @param im 目标图像
 */
void drawTrackedPoints(const std::vector<cv::KeyPoint> &vKeys, const std::vector<ORB_SLAM2::MapPoint *> &vMPs, cv::Mat &im);

/**
 * 绘制所有地图点（用于AR重定位时显示完整点云）
 * @param Tcw 相机位姿矩阵 (世界到相机的变换)
 * @param allMapPoints 所有地图点列表
 * @param im 目标图像
 * @param fx 相机内参：X轴焦距
 * @param fy 相机内参：Y轴焦距
 * @param cx 相机内参：X轴主点
 * @param cy 相机内参：Y轴主点
 * @param drawOnlyLoaded 是否仅绘制从地图加载的点（默认true）
 */
void drawAllMapPoints(const cv::Mat &Tcw, const std::vector<ORB_SLAM2::MapPoint*> &allMapPoints, 
                      cv::Mat &im, float fx, float fy, float cx, float cy, bool drawOnlyLoaded = true);

/**
 * 使用RANSAC算法检测平面
 * @param Tcw 相机位姿矩阵
 * @param vMPs 用于平面拟合的地图点列表
 * @param iterations RANSAC迭代次数
 * @return 检测到的平面对象指针，失败返回NULL
 */
Plane* detectPlane(const cv::Mat Tcw, const std::vector<ORB_SLAM2::MapPoint*> &vMPs, const int iterations);

/**
 * 将OpenCV的Mat矩阵转换为OpenGL的列主序矩阵
 * @param M 输出的列主序矩阵（16个float）
 * @param img 输入的OpenCV Mat矩阵（4x4）
 */
void getColMajorMatrixFromMat(float M[],cv::Mat &img);

#endif //ORB_SLAM2_AR_PLANE_H

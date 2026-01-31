/**
 * Created by Ads on 2017/1/15.
 * 由Olsc于2025/8/25开始进行修改
 */

/**
 * 平面检测与表示模块
 * 功能描述: 从3D地图点中检测平面，用于AR物体放置
 */

#ifndef ORB_SLAM2_AR_PLANE_H
#define ORB_SLAM2_AR_PLANE_H

#include "Common.h"
#include <opencv2/opencv.hpp>
#include "include/System.h"

/**
 * 平面类
 * 表示3D空间中的一个平面，包含法向量、原点和坐标变换
 */
class Plane
{
public:
    /**
     * 从地图点集合构造平面
     * 使用SVD分解自动拟合平面
     * @param vMPs 定义平面的地图点集合
     * @param Tcw 相机位姿（用于确定法向量方向）
     */
    Plane(const std::vector<ORB_SLAM2::MapPoint*> &vMPs, const cv::Mat &Tcw);
    
    /**
     * 从法向量和原点构造平面
     * @param nx 法向量X分量
     * @param ny 法向量Y分量
     * @param nz 法向量Z分量
     * @param ox 原点X坐标
     * @param oy 原点Y坐标
     * @param oz 原点Z坐标
     */
    Plane(const float &nx, const float &ny, const float &nz, const float &ox, const float &oy, const float &oz);
    
    /**
     * SO(3)李代数的指数映射
     * 将旋转向量转换为旋转矩阵（Rodrigues公式）
     * @param x 旋转向量X分量
     * @param y 旋转向量Y分量
     * @param z 旋转向量Z分量
     * @return 旋转矩阵
     */
    cv::Mat ExpSO3(const float &x, const float &y, const float &z);
    
    /**
     * SO(3)李代数的指数映射（向量版本）
     * @param v 旋转向量（3x1）
     * @return 旋转矩阵
     */
    cv::Mat ExpSO3(const cv::Mat &v);
    
    /**
     * 根据地图点重新计算平面参数
     * 使用SVD分解更新法向量和原点
     */
    void Recompute();

    // 平面法向量（3x1矩阵）
    cv::Mat n;
    
    // 平面原点（3x1矩阵）
    cv::Mat o;
    
    // 平面绕法向量的任意旋转角度
    float rang;
    
    // 从世界坐标系到平面坐标系的变换矩阵（4x4）
    cv::Mat Tpw;
    
    // OpenGL格式的平面变换矩阵（列主序，16个float）
    float glTpw[16];
    
    // 定义平面的地图点集合
    std::vector<ORB_SLAM2::MapPoint*> mvMPs;
    
    // 首次观测平面时的相机位姿（用于确定法向量方向）
    cv::Mat mTcw;
    
    // 从相机中心指向平面原点的向量
    cv::Mat XC;
};

#endif // ORB_SLAM2_AR_PLANE_H

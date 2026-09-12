/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

/*
 * This project is based on ORB-SLAM2.
 *
 * The ORB-SLAM2 project was ported to the Android platform by Ads
 * under the GitHub account Martin20150405 in 2017.
 *
 * Starting from August 25, 2025, Olsc began modifying this project.
 * On the basis of the original project, functions such as map saving,
 * map loading, and relocalization were added.
 *
 * This project is distributed under the GNU General Public License
 * version 3, together with ORB-SLAM2.
 */

#ifndef CONVERTER_H
#define CONVERTER_H

#include<opencv2/core/core.hpp>
#include <cmath>

#include"../Thirdparty/Eigen/Dense"
#include"Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include"Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{

class Converter
{
public:
    static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

    static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);
    static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);

    static cv::Mat toCvMat(const g2o::SE3Quat &SE3);
    static cv::Mat toCvMat(const g2o::Sim3 &Sim3);
    static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);
    static cv::Mat toCvMat(const Eigen::Matrix3d &m);
    static cv::Mat toCvMat(const Eigen::Matrix<double,3,1> &m);
    static cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t);

    static Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector);
    static Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f &cvPoint);
    static Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3);

    static std::vector<float> toQuaternion(const cv::Mat &M);

    // 线性三角化：闭式代数 DLT 标量法（零开方操作，全标量对称矩阵克莱姆求解，经 Python 数学实测验证）
    static bool TriangulateWithCenters(const cv::Mat &P1, const cv::Mat &P2,
                                       const float /*Q1*/[3], const float /*Q2*/[3],
                                       float x1, float y1, float x2, float y2,
                                       cv::Mat &x3D)
    {
        const float p1_00=P1.at<float>(0,0), p1_01=P1.at<float>(0,1), p1_02=P1.at<float>(0,2), p1_03=P1.at<float>(0,3);
        const float p1_10=P1.at<float>(1,0), p1_11=P1.at<float>(1,1), p1_12=P1.at<float>(1,2), p1_13=P1.at<float>(1,3);
        const float p1_20=P1.at<float>(2,0), p1_21=P1.at<float>(2,1), p1_22=P1.at<float>(2,2), p1_23=P1.at<float>(2,3);

        const float p2_00=P2.at<float>(0,0), p2_01=P2.at<float>(0,1), p2_02=P2.at<float>(0,2), p2_03=P2.at<float>(0,3);
        const float p2_10=P2.at<float>(1,0), p2_11=P2.at<float>(1,1), p2_12=P2.at<float>(1,2), p2_13=P2.at<float>(1,3);
        const float p2_20=P2.at<float>(2,0), p2_21=P2.at<float>(2,1), p2_22=P2.at<float>(2,2), p2_23=P2.at<float>(2,3);

        // 构造 DLT 代数矩阵 A (4x4) 的四行系数
        const float a0_0 = x1*p1_20 - p1_00, a0_1 = x1*p1_21 - p1_01, a0_2 = x1*p1_22 - p1_02, a0_3 = x1*p1_23 - p1_03;
        const float a1_0 = y1*p1_20 - p1_10, a1_1 = y1*p1_21 - p1_11, a1_2 = y1*p1_22 - p1_12, a1_3 = y1*p1_23 - p1_13;
        const float a2_0 = x2*p2_20 - p2_00, a2_1 = x2*p2_21 - p2_01, a2_2 = x2*p2_22 - p2_02, a2_3 = x2*p2_23 - p2_03;
        const float a3_0 = y2*p2_20 - p2_10, a3_1 = y2*p2_21 - p2_11, a3_2 = y2*p2_22 - p2_12, a3_3 = y2*p2_23 - p2_13;

        // 对称半正定法方程系数矩阵 M = A[:, :3]^T * A[:, :3] (利用对称性仅算 6 项)
        const float m00 = a0_0*a0_0 + a1_0*a1_0 + a2_0*a2_0 + a3_0*a3_0;
        const float m01 = a0_0*a0_1 + a1_0*a1_1 + a2_0*a2_1 + a3_0*a3_1;
        const float m02 = a0_0*a0_2 + a1_0*a1_2 + a2_0*a2_2 + a3_0*a3_2;
        const float m11 = a0_1*a0_1 + a1_1*a1_1 + a2_1*a2_1 + a3_1*a3_1;
        const float m12 = a0_1*a0_2 + a1_1*a1_2 + a2_1*a2_2 + a3_1*a3_2;
        const float m22 = a0_2*a0_2 + a1_2*a1_2 + a2_2*a2_2 + a3_2*a3_2;

        // 常数项向量 B = -A[:, :3]^T * A[:, 3]
        const float b0 = -(a0_0*a0_3 + a1_0*a1_3 + a2_0*a2_3 + a3_0*a3_3);
        const float b1 = -(a0_1*a0_3 + a1_1*a1_3 + a2_1*a2_3 + a3_1*a3_3);
        const float b2 = -(a0_2*a0_3 + a1_2*a1_3 + a2_2*a2_3 + a3_2*a3_3);

        // 克莱姆法则闭式代数解（零浮点开方开销）
        const float c00 = m11*m22 - m12*m12;
        const float c01 = m02*m12 - m01*m22;
        const float c02 = m01*m12 - m02*m11;

        const float det = m00*c00 + m01*c01 + m02*c02;
        if(std::fabs(det) < 1e-12f)
            return false;

        const float invDet = 1.0f / det;
        const float c11 = m00*m22 - m02*m02;
        const float c12 = m01*m02 - m00*m12;
        const float c22 = m00*m11 - m01*m01;

        const float X = (c00*b0 + c01*b1 + c02*b2) * invDet;
        const float Y = (c01*b0 + c11*b1 + c12*b2) * invDet;
        const float Z = (c02*b0 + c12*b1 + c22*b2) * invDet;

        x3D = cv::Mat(3, 1, CV_32F);
        float* pData = x3D.ptr<float>();
        pData[0] = X;
        pData[1] = Y;
        pData[2] = Z;
        return true;
    }

    // 单次解算相机光心，辅助单次调用
    static bool ComputeCameraCenter(const cv::Mat &Pm, float out[3]) {
        const float a11=Pm.at<float>(0,0), a12=Pm.at<float>(0,1), a13=Pm.at<float>(0,2);
        const float a21=Pm.at<float>(1,0), a22=Pm.at<float>(1,1), a23=Pm.at<float>(1,2);
        const float a31=Pm.at<float>(2,0), a32=Pm.at<float>(2,1), a33=Pm.at<float>(2,2);
        const float b1=-Pm.at<float>(0,3), b2=-Pm.at<float>(1,3), b3=-Pm.at<float>(2,3);
        const float c11=a22*a33-a23*a32, c12=a21*a33-a23*a31, c13=a21*a32-a22*a31;
        const float det=a11*c11-a12*c12+a13*c13;
        if(std::fabs(det)<1e-12f) return false;
        out[0]=(b1*c11-a12*(b2*a33-a23*b3)+a13*(b2*a32-a22*b3))/det;
        out[1]=(a11*(b2*a33-a23*b3)-b1*c12+a13*(a21*b3-b2*a31))/det;
        out[2]=(a11*(a22*b3-b2*a32)-a12*(a21*b3-b2*a31)+b1*c13)/det;
        return true;
    }

    // 原接口兼容实现：单次求解光心后委托给 TriangulateWithCenters
    static bool TriangulateDLT(const cv::Mat &P1, const cv::Mat &P2,
                               float x1, float y1, float x2, float y2,
                               cv::Mat &x3D)
    {
        float Q1[3], Q2[3];
        if(!ComputeCameraCenter(P1, Q1) || !ComputeCameraCenter(P2, Q2))
            return false;
        return TriangulateWithCenters(P1, P2, Q1, Q2, x1, y1, x2, y2, x3D);
    }
};

}// namespace ORB_SLAM2

#endif // CONVERTER_H
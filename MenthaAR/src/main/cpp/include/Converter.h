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

    // 线性三角化：闭式中点法。光线方向 = DLT 行平面法向叉积（与 SVD 零空间
    // 同源），相机中心 = Cramer 解 P[:,0:3]·C = -P[:,3]，零分配。
    static bool TriangulateDLT(const cv::Mat &P1, const cv::Mat &P2,
                               float x1, float y1, float x2, float y2,
                               cv::Mat &x3D)
    {
        // ---- 光线方向：DLT 行平面法向叉积 ----
        // 平面对：na = x*P.row2 - P.row0，nb = y*P.row2 - P.row1；u = na.xyz × nb.xyz
        #define TRI_RAY(Pm, px, py, out) do { \
            const float _r00=Pm.at<float>(0,0), _r01=Pm.at<float>(0,1), _r02=Pm.at<float>(0,2); \
            const float _r10=Pm.at<float>(1,0), _r11=Pm.at<float>(1,1), _r12=Pm.at<float>(1,2); \
            const float _r20=Pm.at<float>(2,0), _r21=Pm.at<float>(2,1), _r22=Pm.at<float>(2,2); \
            const float _nax=px*_r20-_r00, _nay=px*_r21-_r01, _naz=px*_r22-_r02; \
            const float _nbx=py*_r20-_r10, _nby=py*_r21-_r11, _nbz=py*_r22-_r12; \
            out[0]=_nay*_nbz-_naz*_nby; \
            out[1]=_naz*_nbx-_nax*_nbz; \
            out[2]=_nax*_nby-_nay*_nbx; \
        } while(0)

        float u[3], v[3];
        TRI_RAY(P1, x1, y1, u);
        TRI_RAY(P2, x2, y2, v);
        #undef TRI_RAY

        const float unrm2 = u[0]*u[0]+u[1]*u[1]+u[2]*u[2];
        const float vnrm2 = v[0]*v[0]+v[1]*v[1]+v[2]*v[2];
        if(unrm2 < 1e-20f || vnrm2 < 1e-20f)
            return false;
        const float invu = 1.0f/std::sqrt(unrm2);
        const float invv = 1.0f/std::sqrt(vnrm2);
        u[0]*=invu; u[1]*=invu; u[2]*=invu;
        v[0]*=invv; v[1]*=invv; v[2]*=invv;

        // ---- 相机中心：Cramer 解 P[:,0:3]·C = -P[:,3]（对 [R|t] 与 K[R|t] 均成立）----
        #define TRI_CENTER(Pm, out) do { \
            const float a11=Pm.at<float>(0,0), a12=Pm.at<float>(0,1), a13=Pm.at<float>(0,2); \
            const float a21=Pm.at<float>(1,0), a22=Pm.at<float>(1,1), a23=Pm.at<float>(1,2); \
            const float a31=Pm.at<float>(2,0), a32=Pm.at<float>(2,1), a33=Pm.at<float>(2,2); \
            const float b1=-Pm.at<float>(0,3), b2=-Pm.at<float>(1,3), b3=-Pm.at<float>(2,3); \
            const float c11=a22*a33-a23*a32, c12=a21*a33-a23*a31, c13=a21*a32-a22*a31; \
            const float det=a11*c11-a12*c12+a13*c13; \
            if(std::fabs(det)<1e-12f) return false; \
            out[0]=(b1*c11-a12*(b2*a33-a23*b3)+a13*(b2*a32-a22*b3))/det; \
            out[1]=(a11*(b2*a33-a23*b3)-b1*c12+a13*(a21*b3-b2*a31))/det; \
            out[2]=(a11*(a22*b3-b2*a32)-a12*(a21*b3-b2*a31)+b1*c13)/det; \
        } while(0)

        float Q1[3], Q2[3];
        TRI_CENTER(P1, Q1);
        TRI_CENTER(P2, Q2);
        #undef TRI_CENTER

        // ---- 异面直线最近点对取中点 ----
        const float w0x = Q1[0]-Q2[0], w0y = Q1[1]-Q2[1], w0z = Q1[2]-Q2[2];
        const float A = u[0]*u[0]+u[1]*u[1]+u[2]*u[2];
        const float B = u[0]*v[0]+u[1]*v[1]+u[2]*v[2];
        const float C = v[0]*v[0]+v[1]*v[1]+v[2]*v[2];
        const float D = u[0]*w0x+u[1]*w0y+u[2]*w0z;
        const float E = v[0]*w0x+v[1]*w0y+v[2]*w0z;
        const float denom = A*C - B*B;
        if(std::fabs(denom) < 1e-12f)
            return false;   // 光线近平行（基线过短），与原 w==0 拒绝语义一致
        const float s = (B*E - C*D)/denom;
        const float t = (A*E - B*D)/denom;

        const float X1x=Q1[0]+s*u[0], X1y=Q1[1]+s*u[1], X1z=Q1[2]+s*u[2];
        const float X2x=Q2[0]+t*v[0], X2y=Q2[1]+t*v[1], X2z=Q2[2]+t*v[2];

        x3D = (cv::Mat_<float>(3,1) << 0.5f*(X1x+X2x), 0.5f*(X1y+X2y), 0.5f*(X1z+X2z));
        return true;
    }
};

}// namespace ORB_SLAM2

#endif // CONVERTER_H
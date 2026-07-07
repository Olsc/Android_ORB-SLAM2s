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

#include "Common.h"
#include "Sim3Solver.h"

#include <vector>
#include <cmath>
#include <opencv2/core/core.hpp>

#include "KeyFrame.h"
#include "ORBmatcher.h"
#include "Config.h"

#include "Thirdparty/DBoW2/DUtils/Random.h"

namespace ORB_SLAM2
{


Sim3Solver::Sim3Solver(KeyFrame *pKF1, KeyFrame *pKF2, const vector<MapPoint *> &vpMatched12):
    mnIterations(0), mnBestInliers(0)
{
    mpKF1 = pKF1;
    mpKF2 = pKF2;

    vector<MapPoint*> vpKeyFrameMP1 = pKF1->GetMapPointMatches();

    mN1 = vpMatched12.size();

    mvpMapPoints1.reserve(mN1);
    mvpMapPoints2.reserve(mN1);
    mvpMatches12 = vpMatched12;
    mvnIndices1.reserve(mN1);
    mvX3Dc1.reserve(mN1);
    mvX3Dc2.reserve(mN1);

    cv::Mat Rcw1 = pKF1->GetRotation();
    cv::Mat tcw1 = pKF1->GetTranslation();
    cv::Mat Rcw2 = pKF2->GetRotation();
    cv::Mat tcw2 = pKF2->GetTranslation();

    mvAllIndices.reserve(mN1);

    size_t idx=0;
    for(int i1=0; i1<mN1; i1++)
    {
        if(vpMatched12[i1])
        {
            MapPoint* pMP1 = vpKeyFrameMP1[i1];
            MapPoint* pMP2 = vpMatched12[i1];

            if(!pMP1)
                continue;

            if(pMP1->isBad() || pMP2->isBad())
                continue;

            int indexKF1 = pMP1->GetIndexInKeyFrame(pKF1);
            int indexKF2 = pMP2->GetIndexInKeyFrame(pKF2);

            if(indexKF1<0 || indexKF2<0)
                continue;

            const cv::KeyPoint &kp1 = pKF1->mvKeysUn[indexKF1];
            const cv::KeyPoint &kp2 = pKF2->mvKeysUn[indexKF2];

            const float sigmaSquare1 = pKF1->mvLevelSigma2[kp1.octave];
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];

            mvnMaxError1.push_back(SIM3_CHI2_TH*sigmaSquare1);
            mvnMaxError2.push_back(SIM3_CHI2_TH*sigmaSquare2);

            mvpMapPoints1.push_back(pMP1);
            mvpMapPoints2.push_back(pMP2);
            mvnIndices1.push_back(i1);

            cv::Mat X3D1w = pMP1->GetWorldPos();
            mvX3Dc1.push_back(Rcw1*X3D1w+tcw1);

            cv::Mat X3D2w = pMP2->GetWorldPos();
            mvX3Dc2.push_back(Rcw2*X3D2w+tcw2);

            mvAllIndices.push_back(idx);
            idx++;
        }
    }

    mK1 = pKF1->mK;
    mK2 = pKF2->mK;

    FromCameraToImage(mvX3Dc1,mvP1im1,mK1);
    FromCameraToImage(mvX3Dc2,mvP2im2,mK2);

    SetRansacParameters(SIM3_RANSAC_PROB, SIM3_RANSAC_MIN_INLIERS, SIM3_RANSAC_MAX_ITERS);
}

void Sim3Solver::SetRansacParameters(double probability, int minInliers, int maxIterations)
{
    mRansacProb = probability;
    mRansacMinInliers = minInliers;
    mRansacMaxIts = maxIterations;    

    N = mvpMapPoints1.size(); // 对应点的数量

    mvbInliersi.resize(N);

    // 根据对应点的数量调整参数
    float epsilon = (float)mRansacMinInliers/N;

    // 根据概率、epsilon 和最大迭代次数设置 RANSAC 迭代次数
    int nIterations;

    if(mRansacMinInliers==N)
        nIterations=1;
    else
        nIterations = ceil(log(1-mRansacProb)/log(1-pow(epsilon,3)));

    mRansacMaxIts = max(1,min(nIterations,mRansacMaxIts));

    mnIterations = 0;
}

cv::Mat Sim3Solver::iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers)
{
    bNoMore = false;
    vbInliers = vector<bool>(mN1,false);
    nInliers=0;

    if(N<mRansacMinInliers)
    {
        bNoMore = true;
        return cv::Mat();
    }

    vector<size_t> vAvailableIndices;

    cv::Mat P3Dc1i(3,3,CV_32F);
    cv::Mat P3Dc2i(3,3,CV_32F);

    int nCurrentIterations = 0;
    while(mnIterations<mRansacMaxIts && nCurrentIterations<nIterations)
    {
        nCurrentIterations++;
        mnIterations++;

        vAvailableIndices = mvAllIndices;

        // 获取最小点集
        for(short i = 0; i < 3; ++i)
        {
            int randi = rand() % vAvailableIndices.size();

            int idx = vAvailableIndices[randi];

            mvX3Dc1[idx].copyTo(P3Dc1i.col(i));
            mvX3Dc2[idx].copyTo(P3Dc2i.col(i));

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        ComputeSim3(P3Dc1i,P3Dc2i);

        CheckInliers();

        if(mnInliersi>=mnBestInliers)
        {
            mvbBestInliers = mvbInliersi;
            mnBestInliers = mnInliersi;
            mBestT12 = mT12i.clone();
            mBestRotation = mR12i.clone();
            mBestTranslation = mt12i.clone();
            mBestScale = ms12i;

            if(mnInliersi>mRansacMinInliers)
            {
                nInliers = mnInliersi;
                for(int i=0; i<N; i++)
                    if(mvbInliersi[i])
                        vbInliers[mvnIndices1[i]] = true;
                return mBestT12;
            }
        }
    }

    if(mnIterations>=mRansacMaxIts)
        bNoMore=true;

    return cv::Mat();
}

cv::Mat Sim3Solver::find(vector<bool> &vbInliers12, int &nInliers)
{
    bool bFlag;
    return iterate(mRansacMaxIts,bFlag,vbInliers12,nInliers);
}

void Sim3Solver::ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C)
{
    cv::reduce(P,C,1,CV_REDUCE_SUM);
    C = C/P.cols;

    for(int i=0; i<P.cols; i++)
    {
        Pr.col(i)=P.col(i)-C;
    }
}

void Sim3Solver::ComputeSim3(cv::Mat &P1, cv::Mat &P2)
{
    // 自定义实现：
    // Horn 1987，使用单位四元数的绝对方向闭式解

    // 步骤1：质心和相对坐标

    cv::Mat Pr1(P1.size(),P1.type()); // 相对于质心的坐标 (集合1)
    cv::Mat Pr2(P2.size(),P2.type()); // 相对于质心的坐标 (集合2)
    cv::Mat O1(3,1,Pr1.type()); // P1的质心
    cv::Mat O2(3,1,Pr2.type()); // P2的质心

    ComputeCentroid(P1,Pr1,O1);
    ComputeCentroid(P2,Pr2,O2);

    // 步骤2：计算M矩阵

    cv::Mat M = Pr2*Pr1.t();

    // 步骤3：计算N矩阵

    double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;

    cv::Mat N(4,4,P1.type());

    N11 = M.at<float>(0,0)+M.at<float>(1,1)+M.at<float>(2,2);
    N12 = M.at<float>(1,2)-M.at<float>(2,1);
    N13 = M.at<float>(2,0)-M.at<float>(0,2);
    N14 = M.at<float>(0,1)-M.at<float>(1,0);
    N22 = M.at<float>(0,0)-M.at<float>(1,1)-M.at<float>(2,2);
    N23 = M.at<float>(0,1)+M.at<float>(1,0);
    N24 = M.at<float>(2,0)+M.at<float>(0,2);
    N33 = -M.at<float>(0,0)+M.at<float>(1,1)-M.at<float>(2,2);
    N34 = M.at<float>(1,2)+M.at<float>(2,1);
    N44 = -M.at<float>(0,0)-M.at<float>(1,1)+M.at<float>(2,2);

    N = (cv::Mat_<float>(4,4) << N11, N12, N13, N14,
                                 N12, N22, N23, N24,
                                 N13, N23, N33, N34,
                                 N14, N24, N34, N44);


    // 步骤4：最大特征值的特征向量

    cv::Mat eval, evec;
    // 计算最大特征值对应特征向量，OpenCV返回按降序排列
    cv::eigen(N,eval,evec);
    // 直接用四元数构造旋转矩阵，避免Rodrigues计算与一次atan2、一次norm
    const float qw = evec.at<float>(0,0);
    const float qx = evec.at<float>(0,1);
    const float qy = evec.at<float>(0,2);
    const float qz = evec.at<float>(0,3);
    const float nq = std::sqrt(qw*qw+qx*qx+qy*qy+qz*qz)+1e-12f;
    const float w = qw/nq, x = qx/nq, y = qy/nq, z = qz/nq;
    mR12i.create(3,3,P1.type());
    const float xx = x*x, yy = y*y, zz = z*z;
    const float xy = x*y, xz = x*z, yz = y*z;
    const float wx = w*x, wy = w*y, wz = w*z;
    mR12i.at<float>(0,0) = 1.0f - 2.0f*(yy+zz);
    mR12i.at<float>(0,1) = 2.0f*(xy - wz);
    mR12i.at<float>(0,2) = 2.0f*(xz + wy);
    mR12i.at<float>(1,0) = 2.0f*(xy + wz);
    mR12i.at<float>(1,1) = 1.0f - 2.0f*(xx+zz);
    mR12i.at<float>(1,2) = 2.0f*(yz - wx);
    mR12i.at<float>(2,0) = 2.0f*(xz - wy);
    mR12i.at<float>(2,1) = 2.0f*(yz + wx);
    mR12i.at<float>(2,2) = 1.0f - 2.0f*(xx+yy);

    // 步骤5：旋转集合2

    cv::Mat P3 = mR12i*Pr2;

    // 步骤6：缩放
    // 使用dot()替代cv::pow和手动遍历，避免临时矩阵分配
    {
        double nom = Pr1.dot(P3);
        double den = P3.dot(P3);  // 等价于sum(P3.^2)
        ms12i = nom/den;
    }

    // 步骤7：平移

    mt12i.create(1,3,P1.type());
    mt12i = O1 - ms12i*mR12i*O2;

    // 步骤8：变换

    // 步骤8.1 T12
    mT12i = cv::Mat::eye(4,4,P1.type());

    cv::Mat sR = ms12i*mR12i;

    sR.copyTo(mT12i.rowRange(0,3).colRange(0,3));
    mt12i.copyTo(mT12i.rowRange(0,3).col(3));

    // 步骤8.2 T21

    mT21i = cv::Mat::eye(4,4,P1.type());

    cv::Mat sRinv = (1.0/ms12i)*mR12i.t();

    sRinv.copyTo(mT21i.rowRange(0,3).colRange(0,3));
    cv::Mat tinv = -sRinv*mt12i;
    tinv.copyTo(mT21i.rowRange(0,3).col(3));
}


void Sim3Solver::CheckInliers()
{
    vector<cv::Point2f> vP1im2, vP2im1;
    Project(mvX3Dc2,vP2im1,mT12i,mK1);
    Project(mvX3Dc1,vP1im2,mT21i,mK2);

    mnInliersi=0;

    for(size_t i=0; i<mvP1im1.size(); i++)
    {
        // 直接访问 Point2f 坐标，避免访问 Mat 开销
        const float dx1 = mvP1im1[i].x - vP2im1[i].x;
        const float dy1 = mvP1im1[i].y - vP2im1[i].y;
        const float err1 = dx1*dx1 + dy1*dy1;

        const float dx2 = vP1im2[i].x - mvP2im2[i].x;
        const float dy2 = vP1im2[i].y - mvP2im2[i].y;
        const float err2 = dx2*dx2 + dy2*dy2;

        if(err1<mvnMaxError1[i] && err2<mvnMaxError2[i])
        {
            mvbInliersi[i]=true;
            mnInliersi++;
        }
        else
            mvbInliersi[i]=false;
    }
}


cv::Mat Sim3Solver::GetEstimatedRotation()
{
    return mBestRotation.clone();
}

cv::Mat Sim3Solver::GetEstimatedTranslation()
{
    return mBestTranslation.clone();
}

float Sim3Solver::GetEstimatedScale()
{
    return mBestScale;
}

void Sim3Solver::Project(const vector<cv::Mat> &vP3Dw, vector<cv::Point2f> &vP2D, cv::Mat Tcw, cv::Mat K)
{
    // 提取矩阵元素为标量，避免循环内创建临时Mat对象
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    
    const float r00 = Rcw.at<float>(0,0); const float r01 = Rcw.at<float>(0,1); const float r02 = Rcw.at<float>(0,2);
    const float r10 = Rcw.at<float>(1,0); const float r11 = Rcw.at<float>(1,1); const float r12 = Rcw.at<float>(1,2);
    const float r20 = Rcw.at<float>(2,0); const float r21 = Rcw.at<float>(2,1); const float r22 = Rcw.at<float>(2,2);
    
    const float tx = tcw.at<float>(0);
    const float ty = tcw.at<float>(1);
    const float tz = tcw.at<float>(2);

    const float &fx = K.at<float>(0,0);
    const float &fy = K.at<float>(1,1);
    const float &cx = K.at<float>(0,2);
    const float &cy = K.at<float>(1,2);

    vP2D.clear();
    vP2D.reserve(vP3Dw.size());

    for(size_t i=0, iend=vP3Dw.size(); i<iend; i++)
    {
        const float X = vP3Dw[i].at<float>(0);
        const float Y = vP3Dw[i].at<float>(1);
        const float Z = vP3Dw[i].at<float>(2);
        
        const float Xc = r00*X + r01*Y + r02*Z + tx;
        const float Yc = r10*X + r11*Y + r12*Z + ty;
        const float Zc = r20*X + r21*Y + r22*Z + tz;
        
        const float invz = 1.0f/Zc;
        const float u = fx*Xc*invz+cx;
        const float v = fy*Yc*invz+cy;

        vP2D.push_back(cv::Point2f(u, v));
    }
}

void Sim3Solver::FromCameraToImage(const vector<cv::Mat> &vP3Dc, vector<cv::Point2f> &vP2D, cv::Mat K)
{
    const float &fx = K.at<float>(0,0);
    const float &fy = K.at<float>(1,1);
    const float &cx = K.at<float>(0,2);
    const float &cy = K.at<float>(1,2);

    vP2D.clear();
    vP2D.reserve(vP3Dc.size());

    for(size_t i=0, iend=vP3Dc.size(); i<iend; i++)
    {
        const float invz = 1.0f/(vP3Dc[i].at<float>(2));
        const float x = vP3Dc[i].at<float>(0)*invz;
        const float y = vP3Dc[i].at<float>(1)*invz;

        vP2D.push_back(cv::Point2f(fx*x+cx, fy*y+cy));
    }
}

} //namespace ORB_SLAM2

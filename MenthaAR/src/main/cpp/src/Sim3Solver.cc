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
#include <limits>
#include <opencv2/core/core.hpp>

#include "KeyFrame.h"
#include "ORBmatcher.h"
#include "Config.h"

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

    float Rcw1[9], tcw1[3], Rcw2[9], tcw2[3];
    pKF1->GetRotation(Rcw1);
    pKF1->GetTranslation(tcw1);
    pKF2->GetRotation(Rcw2);
    pKF2->GetTranslation(tcw2);

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

            // 相机坐标 Xc = Rcw*Xw + tcw（标量，不再为每个点分配 3x1 cv::Mat）
            cv::Point3f X1w, X2w;
            pMP1->GetWorldPos(X1w);
            pMP2->GetWorldPos(X2w);
            mvX3Dc1.push_back(cv::Point3f(Rcw1[0]*X1w.x + Rcw1[1]*X1w.y + Rcw1[2]*X1w.z + tcw1[0],
                                          Rcw1[3]*X1w.x + Rcw1[4]*X1w.y + Rcw1[5]*X1w.z + tcw1[1],
                                          Rcw1[6]*X1w.x + Rcw1[7]*X1w.y + Rcw1[8]*X1w.z + tcw1[2]));
            mvX3Dc2.push_back(cv::Point3f(Rcw2[0]*X2w.x + Rcw2[1]*X2w.y + Rcw2[2]*X2w.z + tcw2[0],
                                          Rcw2[3]*X2w.x + Rcw2[4]*X2w.y + Rcw2[5]*X2w.z + tcw2[1],
                                          Rcw2[6]*X2w.x + Rcw2[7]*X2w.y + Rcw2[8]*X2w.z + tcw2[2]));

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
    mvP2im1.resize(N);
    mvP1im2.resize(N);

    // 根据对应点的数量调整参数
    float epsilon = (float)mRansacMinInliers/N;

    // 根据概率、epsilon 和最大迭代次数设置 RANSAC 迭代次数
    int nIterations;

    if(mRansacMinInliers==N)
        nIterations=1;
    else
        nIterations = ceil(log(1-mRansacProb)/log(1-pow(epsilon,SIM3_RANSAC_MIN_SET)));

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

    size_t sample[SIM3_RANSAC_MIN_SET];
    double P1[3][SIM3_RANSAC_MIN_SET]; // 每列一个点
    double P2[3][SIM3_RANSAC_MIN_SET];

    int nCurrentIterations = 0;
    while(mnIterations<mRansacMaxIts && nCurrentIterations<nIterations)
    {
        nCurrentIterations++;
        mnIterations++;

        // 交换-恢复抽样：与“整份拷贝可用下标再删除”的实现产生相同样本，写入 O(3) 而非 O(N)
        SampleWithoutReplacement(mLcg, mvAllIndices, SIM3_RANSAC_MIN_SET, sample);
        for(int i = 0; i < SIM3_RANSAC_MIN_SET; ++i)
        {
            const cv::Point3f &a = mvX3Dc1[sample[i]];
            const cv::Point3f &b = mvX3Dc2[sample[i]];
            P1[0][i] = a.x; P1[1][i] = a.y; P1[2][i] = a.z;
            P2[0][i] = b.x; P2[1][i] = b.y; P2[2][i] = b.z;
        }

        ComputeSim3(P1,P2);

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

void Sim3Solver::ComputeSim3(const double P1[3][SIM3_RANSAC_MIN_SET], const double P2[3][SIM3_RANSAC_MIN_SET])
{
    // Horn 1987：单位四元数的绝对定向闭式解。全部在 double 中进行（原实现为 float 求和与 float 特征分解），
    // 结果以 float 写入 mR12i/mt12i/mT12i/mT21i 供外部使用，同时保留 double 版本给投影检验
    const int n = SIM3_RANSAC_MIN_SET;
    const double invN = 1.0 / n;

    // 步骤1：质心和相对坐标
    double O1[3] = {0,0,0}, O2[3] = {0,0,0};
    for(int i=0;i<n;i++) for(int r=0;r<3;r++) { O1[r] += P1[r][i]; O2[r] += P2[r][i]; }
    for(int r=0;r<3;r++) { O1[r] *= invN; O2[r] *= invN; }

    double Pr1[3][SIM3_RANSAC_MIN_SET], Pr2[3][SIM3_RANSAC_MIN_SET];
    for(int i=0;i<n;i++) for(int r=0;r<3;r++) { Pr1[r][i] = P1[r][i]-O1[r]; Pr2[r][i] = P2[r][i]-O2[r]; }

    // 步骤2：M = Pr2 * Pr1^T
    double M[3][3];
    for(int r=0;r<3;r++) for(int c=0;c<3;c++) { double s=0; for(int i=0;i<n;i++) s += Pr2[r][i]*Pr1[c][i]; M[r][c]=s; }

    // 步骤3：N 矩阵（4x4 对称）
    cv::Mat Nm(4,4,CV_64F);
    const double N11 = M[0][0]+M[1][1]+M[2][2];
    const double N12 = M[1][2]-M[2][1];
    const double N13 = M[2][0]-M[0][2];
    const double N14 = M[0][1]-M[1][0];
    const double N22 = M[0][0]-M[1][1]-M[2][2];
    const double N23 = M[0][1]+M[1][0];
    const double N24 = M[2][0]+M[0][2];
    const double N33 = -M[0][0]+M[1][1]-M[2][2];
    const double N34 = M[1][2]+M[2][1];
    const double N44 = -M[0][0]-M[1][1]+M[2][2];
    double* nm = Nm.ptr<double>();
    nm[0]=N11; nm[1]=N12; nm[2]=N13; nm[3]=N14;
    nm[4]=N12; nm[5]=N22; nm[6]=N23; nm[7]=N24;
    nm[8]=N13; nm[9]=N23; nm[10]=N33; nm[11]=N34;
    nm[12]=N14; nm[13]=N24; nm[14]=N34; nm[15]=N44;

    // 步骤4：最大特征值的特征向量（OpenCV 按降序返回）
    cv::Mat eval, evec;
    cv::eigen(Nm,eval,evec);
    const double qw0 = evec.at<double>(0,0);
    const double qx0 = evec.at<double>(0,1);
    const double qy0 = evec.at<double>(0,2);
    const double qz0 = evec.at<double>(0,3);
    const double nq = std::sqrt(qw0*qw0+qx0*qx0+qy0*qy0+qz0*qz0)+1e-300;
    const double w = qw0/nq, x = qx0/nq, y = qy0/nq, z = qz0/nq;
    const double xx = x*x, yy = y*y, zz = z*z;
    const double xy = x*y, xz = x*z, yz = y*z;
    const double wx = w*x, wy = w*y, wz = w*z;
    mR12d[0][0] = 1.0 - 2.0*(yy+zz); mR12d[0][1] = 2.0*(xy - wz);       mR12d[0][2] = 2.0*(xz + wy);
    mR12d[1][0] = 2.0*(xy + wz);     mR12d[1][1] = 1.0 - 2.0*(xx+zz);   mR12d[1][2] = 2.0*(yz - wx);
    mR12d[2][0] = 2.0*(xz - wy);     mR12d[2][1] = 2.0*(yz + wx);       mR12d[2][2] = 1.0 - 2.0*(xx+yy);

    // 步骤5：旋转集合2  P3 = R12 * Pr2
    double P3[3][SIM3_RANSAC_MIN_SET];
    for(int i=0;i<n;i++) for(int r=0;r<3;r++)
        P3[r][i] = mR12d[r][0]*Pr2[0][i] + mR12d[r][1]*Pr2[1][i] + mR12d[r][2]*Pr2[2][i];

    // 步骤6：尺度 s = <Pr1,P3> / <P3,P3>
    {
        double nom = 0, den = 0;
        for(int i=0;i<n;i++) for(int r=0;r<3;r++) { nom += Pr1[r][i]*P3[r][i]; den += P3[r][i]*P3[r][i]; }
        ms12d = nom/den;
    }

    // 步骤7：平移 t = O1 - s*R12*O2
    for(int r=0;r<3;r++)
        mt12d[r] = O1[r] - ms12d*(mR12d[r][0]*O2[0] + mR12d[r][1]*O2[1] + mR12d[r][2]*O2[2]);

    // 步骤8：T21 = [R^T/s | -R^T t / s]
    const double invS = 1.0/ms12d;
    for(int r=0;r<3;r++) for(int c=0;c<3;c++) mR21d[r][c] = mR12d[c][r]*invS;
    for(int r=0;r<3;r++) mt21d[r] = -(mR21d[r][0]*mt12d[0] + mR21d[r][1]*mt12d[1] + mR21d[r][2]*mt12d[2]);

    // 对外的 float 版本
    ms12i = (float)ms12d;
    mR12i.create(3,3,CV_32F);
    mt12i.create(3,1,CV_32F);
    for(int r=0;r<3;r++) { for(int c=0;c<3;c++) mR12i.at<float>(r,c) = (float)mR12d[r][c]; mt12i.at<float>(r) = (float)mt12d[r]; }
    mT12i = cv::Mat::eye(4,4,CV_32F);
    mT21i = cv::Mat::eye(4,4,CV_32F);
    for(int r=0;r<3;r++)
    {
        for(int c=0;c<3;c++)
        {
            mT12i.at<float>(r,c) = (float)(ms12d*mR12d[r][c]);
            mT21i.at<float>(r,c) = (float)mR21d[r][c];
        }
        mT12i.at<float>(r,3) = (float)mt12d[r];
        mT21i.at<float>(r,3) = (float)mt21d[r];
    }
}

void Sim3Solver::CheckInliers()
{
    // 2 → 1：用 s*R12, t12 把 Xc2 变到相机 1 再投影；1 → 2 用 T21
    Project(mvX3Dc2, mvP2im1, mR12d, ms12d, mt12d, mK1);
    Project(mvX3Dc1, mvP1im2, mR21d, 1.0, mt21d, mK2);

    mnInliersi=0;

    for(size_t i=0; i<mvP1im1.size(); i++)
    {
        // 任一侧在相机后方（Project 置为 NaN）时比较为假，自然判为外点
        const float dx1 = mvP1im1[i].x - mvP2im1[i].x;
        const float dy1 = mvP1im1[i].y - mvP2im1[i].y;
        const float err1 = dx1*dx1 + dy1*dy1;

        const float dx2 = mvP1im2[i].x - mvP2im2[i].x;
        const float dy2 = mvP1im2[i].y - mvP2im2[i].y;
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

void Sim3Solver::Project(const vector<cv::Point3f> &vP3Dc, vector<cv::Point2f> &vP2D,
                         const double R[3][3], const double s, const double t[3], const cv::Mat &K)
{
    const double fx = K.at<float>(0,0);
    const double fy = K.at<float>(1,1);
    const double cx = K.at<float>(0,2);
    const double cy = K.at<float>(1,2);
    const float nanf = std::numeric_limits<float>::quiet_NaN();

    vP2D.resize(vP3Dc.size());

    for(size_t i=0, iend=vP3Dc.size(); i<iend; i++)
    {
        const double X = vP3Dc[i].x, Y = vP3Dc[i].y, Z = vP3Dc[i].z;

        const double Xc = s*(R[0][0]*X + R[0][1]*Y + R[0][2]*Z) + t[0];
        const double Yc = s*(R[1][0]*X + R[1][1]*Y + R[1][2]*Z) + t[1];
        const double Zc = s*(R[2][0]*X + R[2][1]*Y + R[2][2]*Z) + t[2];

        // 相机后方的点与前方镜像点投影到同一像素，原实现不判断深度符号
        if(Zc <= 0.0)
        {
            vP2D[i] = cv::Point2f(nanf, nanf);
            continue;
        }

        const double invz = 1.0/Zc;
        vP2D[i] = cv::Point2f((float)(fx*Xc*invz+cx), (float)(fy*Yc*invz+cy));
    }
}

void Sim3Solver::FromCameraToImage(const vector<cv::Point3f> &vP3Dc, vector<cv::Point2f> &vP2D, const cv::Mat &K)
{
    const float &fx = K.at<float>(0,0);
    const float &fy = K.at<float>(1,1);
    const float &cx = K.at<float>(0,2);
    const float &cy = K.at<float>(1,2);

    vP2D.clear();
    vP2D.reserve(vP3Dc.size());

    for(size_t i=0, iend=vP3Dc.size(); i<iend; i++)
    {
        const float invz = 1.0f/vP3Dc[i].z;
        const float x = vP3Dc[i].x*invz;
        const float y = vP3Dc[i].y*invz;

        vP2D.push_back(cv::Point2f(fx*x+cx, fy*y+cy));
    }
}

} //namespace ORB_SLAM2
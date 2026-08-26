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

#include "Initializer.h"

#include "Thirdparty/DBoW2/DUtils/Random.h"

#include "Optimizer.h"
#include "ORBmatcher.h"
#include "Config.h"
#include "Converter.h"
#include "Random.h"

#include <limits>

#include<thread>

namespace ORB_SLAM2
{

Initializer::Initializer(const Frame &ReferenceFrame, float sigma, int iterations)
{
    mK = ReferenceFrame.mK.clone();

    mvKeys1 = ReferenceFrame.mvKeysUn;

    mSigma = sigma;
    mSigma2 = sigma*sigma;
    mMaxIterations = iterations;
}

bool Initializer::Initialize(const Frame &CurrentFrame, const vector<int> &vMatches12, cv::Mat &R21, cv::Mat &t21,
                             vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated)
{
    // 填充包含当前关键点和参考帧匹配的结构
    // 参考帧：1，当前帧：2
    mvKeys2 = CurrentFrame.mvKeysUn;

    mvMatches12.clear();
    mvMatches12.reserve(mvKeys2.size());
    mvbMatched1.resize(mvKeys1.size());
    for(size_t i=0, iend=vMatches12.size();i<iend; i++)
    {
        if(vMatches12[i]>=0)
        {
            mvMatches12.push_back(make_pair(i,vMatches12[i]));
            mvbMatched1[i]=true;
        }
        else
            mvbMatched1[i]=false;
    }

    const int N = mvMatches12.size();

    // 最小集选择的索引
    vector<size_t> vAllIndices;
    vAllIndices.reserve(N);
    vector<size_t> vAvailableIndices;

    for(int i=0; i<N; i++)
    {
        vAllIndices.push_back(i);
    }

    // 为每次 RANSAC 迭代生成 8 个点的集合
    mvSets = vector< vector<size_t> >(mMaxIterations,vector<size_t>(INITIALIZER_RANSAC_MIN_SET,0));

    LCG lcg(0);

    for(int it=0; it<mMaxIterations; it++)
    {
        vAvailableIndices = vAllIndices;

        // 选择最小集
        for(size_t j=0; j<INITIALIZER_RANSAC_MIN_SET; j++)
        {
            int randi = lcg.randomInt(0, vAvailableIndices.size()-1);
            int idx = vAvailableIndices[randi];

            mvSets[it][j] = idx;

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }
    }

    // 启动线程并行计算基础矩阵和单应性矩阵
    vector<bool> vbMatchesInliersH, vbMatchesInliersF;
    float SH, SF;
    cv::Mat H, F;

    thread threadH(&Initializer::FindHomography,this,ref(vbMatchesInliersH), ref(SH), ref(H));
    thread threadF(&Initializer::FindFundamental,this,ref(vbMatchesInliersF), ref(SF), ref(F));

    // 等待两个线程完成
    threadH.join();
    threadF.join();

    // 计算得分比率
    float RH = SH/(SH+SF);

    // 根据比率 (0.40-0.45) 尝试从单应性矩阵或基础矩阵重建
    if(RH>INITIALIZER_H_SCORE_RATIO)
        return ReconstructH(vbMatchesInliersH,H,mK,R21,t21,vP3D,vbTriangulated,INITIALIZER_MIN_PARALLAX,INITIALIZER_MIN_TRIANGULATED);
    else //if(pF_HF>0.6)
        return ReconstructF(vbMatchesInliersF,F,mK,R21,t21,vP3D,vbTriangulated,INITIALIZER_MIN_PARALLAX,INITIALIZER_MIN_TRIANGULATED);

    return false;
}

void Initializer::FindHomography(vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21)
{
    const int N = mvMatches12.size();

    // 归一化坐标
    vector<cv::Point2f> vPn1, vPn2;
    cv::Mat T1, T2;
    Normalize(mvKeys1,vPn1, T1);
    Normalize(mvKeys2,vPn2, T2);
    cv::Mat T2inv = T2.inv();

    // 最佳结果变量
    score = 0.0;
    vbMatchesInliers = vector<bool>(N,false);

    // 迭代变量
    vector<cv::Point2f> vPn1i(INITIALIZER_RANSAC_MIN_SET);
    vector<cv::Point2f> vPn2i(INITIALIZER_RANSAC_MIN_SET);
    cv::Mat H21i, H12i;
    vector<bool> vbCurrentInliers(N,false);
    float currentScore;

    // 自适应提前终止：按 N=log(1-p)/log(1-w^s) 估算所需迭代数，高内点率时减少迭代，
    // 置信概率 INITIALIZER_RANSAC_PROB 不变
    const double ransacProb = INITIALIZER_RANSAC_PROB;
    const int minSetSize = INITIALIZER_RANSAC_MIN_SET;
    int nInlierBest = 0;

    // 执行所有 RANSAC 迭代并保存得分最高的解
    for(int it=0; it<mMaxIterations; it++)
    {
        // 选择最小集
        for(size_t j=0; j<INITIALIZER_RANSAC_MIN_SET; j++)
        {
            int idx = mvSets[it][j];

            vPn1i[j] = vPn1[mvMatches12[idx].first];
            vPn2i[j] = vPn2[mvMatches12[idx].second];
        }

        cv::Mat Hn = ComputeH21(vPn1i,vPn2i);
        H21i = T2inv*Hn*T1;
        H12i = H21i.inv();

        currentScore = CheckHomography(H21i, H12i, vbCurrentInliers, mSigma);

        if(currentScore>score)
        {
            H21 = H21i.clone();
            vbMatchesInliers = vbCurrentInliers;
            score = currentScore;
            nInlierBest = 0;
            for(int i=0; i<N; ++i) if(vbMatchesInliers[i]) nInlierBest++;
        }

        // 自适应提前终止：内点率足够高时按标准公式估算剩余需求
        if(it >= INITIALIZER_ADAPTIVE_START_ITER && nInlierBest > 0)
        {
            const double w = (double)nInlierBest / (double)N;
            double wPow = 1.0;
            for(int k=0; k<minSetSize; ++k) wPow *= w;
            if(wPow > 1e-9)
            {
                const double nNeeded = std::log(1.0-ransacProb) / std::log(1.0-wPow);
                if(std::isfinite(nNeeded) && (double)it >= nNeeded * INITIALIZER_ADAPTIVE_SAFETY_FACTOR)
                    break;
            }
        }
    }
}

void Initializer::FindFundamental(vector<bool> &vbMatchesInliers, float &score, cv::Mat &F21)
{
    const int N = vbMatchesInliers.size();

    // 归一化坐标
    vector<cv::Point2f> vPn1, vPn2;
    cv::Mat T1, T2;
    Normalize(mvKeys1,vPn1, T1);
    Normalize(mvKeys2,vPn2, T2);
    cv::Mat T2t = T2.t();

    // 最佳结果变量
    score = 0.0;
    vbMatchesInliers = vector<bool>(N,false);

    // 迭代变量
    vector<cv::Point2f> vPn1i(INITIALIZER_RANSAC_MIN_SET);
    vector<cv::Point2f> vPn2i(INITIALIZER_RANSAC_MIN_SET);
    cv::Mat F21i;
    vector<bool> vbCurrentInliers(N,false);
    float currentScore;

    // 自适应提前终止（同 FindHomography，置信度不变）
    const double ransacProb = INITIALIZER_RANSAC_PROB;
    const int minSetSize = INITIALIZER_RANSAC_MIN_SET;
    int nInlierBest = 0;

    // 执行所有 RANSAC 迭代并保存得分最高的解
    for(int it=0; it<mMaxIterations; it++)
    {
        // 选择最小集
        for(size_t j=0; j<INITIALIZER_RANSAC_MIN_SET; j++)
        {
            int idx = mvSets[it][j];

            vPn1i[j] = vPn1[mvMatches12[idx].first];
            vPn2i[j] = vPn2[mvMatches12[idx].second];
        }

        cv::Mat Fn = ComputeF21(vPn1i,vPn2i);

        F21i = T2t*Fn*T1;

        currentScore = CheckFundamental(F21i, vbCurrentInliers, mSigma);

        if(currentScore>score)
        {
            F21 = F21i.clone();
            vbMatchesInliers = vbCurrentInliers;
            score = currentScore;
            nInlierBest = 0;
            for(int i=0; i<N; ++i) if(vbMatchesInliers[i]) nInlierBest++;
        }

        if(it >= INITIALIZER_ADAPTIVE_START_ITER && nInlierBest > 0)
        {
            const double w = (double)nInlierBest / (double)N;
            double wPow = 1.0;
            for(int k=0; k<minSetSize; ++k) wPow *= w;
            if(wPow > 1e-9)
            {
                const double nNeeded = std::log(1.0-ransacProb) / std::log(1.0-wPow);
                if(std::isfinite(nNeeded) && (double)it >= nNeeded * INITIALIZER_ADAPTIVE_SAFETY_FACTOR)
                    break;
            }
        }
    }
}

cv::Mat Initializer::ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2)
{
    const int N = vP1.size();

    cv::Mat A(2*N, 9, CV_32F);

    for(int i=0; i<N; i++)
    {
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

        A.at<float>(2*i,0) = 0.0f;
        A.at<float>(2*i,1) = 0.0f;
        A.at<float>(2*i,2) = 0.0f;
        A.at<float>(2*i,3) = -u1;
        A.at<float>(2*i,4) = -v1;
        A.at<float>(2*i,5) = -1.0f;
        A.at<float>(2*i,6) = v2*u1;
        A.at<float>(2*i,7) = v2*v1;
        A.at<float>(2*i,8) = v2;

        A.at<float>(2*i+1,0) = u1;
        A.at<float>(2*i+1,1) = v1;
        A.at<float>(2*i+1,2) = 1.0f;
        A.at<float>(2*i+1,3) = 0.0f;
        A.at<float>(2*i+1,4) = 0.0f;
        A.at<float>(2*i+1,5) = 0.0f;
        A.at<float>(2*i+1,6) = -u2*u1;
        A.at<float>(2*i+1,7) = -u2*v1;
        A.at<float>(2*i+1,8) = -u2;
    }

    cv::Mat u, w, vt;
    // A 为 16x9 超定阵，vt 本就是完整 9x9；FULL_UV 只会白白把 u 扩成 16x16
    cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A);

    return vt.row(8).reshape(0, 3).clone();
}

cv::Mat Initializer::ComputeF21(const vector<cv::Point2f> &vP1,const vector<cv::Point2f> &vP2)
{
    const int N = vP1.size();

    cv::Mat A(N, 9, CV_32F);

    for(int i=0; i<N; i++)
    {
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

        A.at<float>(i,0) = u2*u1;
        A.at<float>(i,1) = u2*v1;
        A.at<float>(i,2) = u2;
        A.at<float>(i,3) = v2*u1;
        A.at<float>(i,4) = v2*v1;
        A.at<float>(i,5) = v2;
        A.at<float>(i,6) = u1;
        A.at<float>(i,7) = v1;
        A.at<float>(i,8) = 1.0f;
    }

    cv::Mat u, w, vt;
    cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    cv::Mat Fpre = vt.row(8).reshape(0, 3);

    cv::SVDecomp(Fpre, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    w.at<float>(2) = 0.0f;

    return u * cv::Mat::diag(w) * vt;
}

float Initializer::CheckHomography(const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma)
{   
    const int N = mvMatches12.size();

    const float h11 = H21.at<float>(0,0);
    const float h12 = H21.at<float>(0,1);
    const float h13 = H21.at<float>(0,2);
    const float h21 = H21.at<float>(1,0);
    const float h22 = H21.at<float>(1,1);
    const float h23 = H21.at<float>(1,2);
    const float h31 = H21.at<float>(2,0);
    const float h32 = H21.at<float>(2,1);
    const float h33 = H21.at<float>(2,2);

    const float h11inv = H12.at<float>(0,0);
    const float h12inv = H12.at<float>(0,1);
    const float h13inv = H12.at<float>(0,2);
    const float h21inv = H12.at<float>(1,0);
    const float h22inv = H12.at<float>(1,1);
    const float h23inv = H12.at<float>(1,2);
    const float h31inv = H12.at<float>(2,0);
    const float h32inv = H12.at<float>(2,1);
    const float h33inv = H12.at<float>(2,2);

    vbMatchesInliers.resize(N);

    float score = 0;

    const float th = OPTIMIZER_CHI2_TH_2D;
    const float thSigma2 = th * sigma * sigma;
    const float invSigmaSquare = 1.0f/(sigma*sigma);

    for(int i=0; i<N; i++)
    {
        bool bIn = true;

        const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
        const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];

        const float u1 = kp1.pt.x;
        const float v1 = kp1.pt.y;
        const float u2 = kp2.pt.x;
        const float v2 = kp2.pt.y;

        // x2in1 = H12*x2

        const float w2in1inv = 1.0/(h31inv*u2+h32inv*v2+h33inv);
        const float u2in1 = (h11inv*u2+h12inv*v2+h13inv)*w2in1inv;
        const float v2in1 = (h21inv*u2+h22inv*v2+h23inv)*w2in1inv;

        const float squareDist1 = (u1-u2in1)*(u1-u2in1)+(v1-v2in1)*(v1-v2in1);

        // 等价于: squareDist1*invSigmaSquare > th ⇔ squareDist1 > th*sigma²
        if(squareDist1 > thSigma2)
            bIn = false;
        else
            score += th - squareDist1*invSigmaSquare;

        // x1in2 = H21*x1

        const float w1in2inv = 1.0/(h31*u1+h32*v1+h33);
        const float u1in2 = (h11*u1+h12*v1+h13)*w1in2inv;
        const float v1in2 = (h21*u1+h22*v1+h23)*w1in2inv;

        const float squareDist2 = (u2-u1in2)*(u2-u1in2)+(v2-v1in2)*(v2-v1in2);

        if(squareDist2 > thSigma2)
            bIn = false;
        else
            score += th - squareDist2*invSigmaSquare;

        if(bIn)
            vbMatchesInliers[i]=true;
        else
            vbMatchesInliers[i]=false;
    }

    return score;
}

float Initializer::CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma)
{
    const int N = mvMatches12.size();

    const float f11 = F21.at<float>(0,0);
    const float f12 = F21.at<float>(0,1);
    const float f13 = F21.at<float>(0,2);
    const float f21 = F21.at<float>(1,0);
    const float f22 = F21.at<float>(1,1);
    const float f23 = F21.at<float>(1,2);
    const float f31 = F21.at<float>(2,0);
    const float f32 = F21.at<float>(2,1);
    const float f33 = F21.at<float>(2,2);

    vbMatchesInliers.resize(N);

    float score = 0;

    const float th = OPTIMIZER_CHI2_TH_1D;
    const float thScore = OPTIMIZER_CHI2_TH_2D;

    const float thSigma2 = th * sigma * sigma;
    const float invSigmaSquare = 1.0f/(sigma*sigma);

    for(int i=0; i<N; i++)
    {
        bool bIn = true;

        const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
        const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];

        const float u1 = kp1.pt.x;
        const float v1 = kp1.pt.y;
        const float u2 = kp2.pt.x;
        const float v2 = kp2.pt.y;

        // l2=F21x1=(a2,b2,c2)

        const float a2 = f11*u1+f12*v1+f13;
        const float b2 = f21*u1+f22*v1+f23;
        const float c2 = f31*u1+f32*v1+f33;

        const float num2 = a2*u2+b2*v2+c2;

        const float squareDist1 = num2*num2/(a2*a2+b2*b2);

        // 等价于: squareDist1*invSigmaSquare > th ⇔ squareDist1 > th*sigma²
        if(squareDist1 > thSigma2)
            bIn = false;
        else
            score += thScore - squareDist1*invSigmaSquare;

        // l1 =x2tF21=(a1,b1,c1)

        const float a1 = f11*u2+f21*v2+f31;
        const float b1 = f12*u2+f22*v2+f32;
        const float c1 = f13*u2+f23*v2+f33;

        const float num1 = a1*u1+b1*v1+c1;

        const float squareDist2 = num1*num1/(a1*a1+b1*b1);

        if(squareDist2 > thSigma2)
            bIn = false;
        else
            score += thScore - squareDist2*invSigmaSquare;

        if(bIn)
            vbMatchesInliers[i]=true;
        else
            vbMatchesInliers[i]=false;
    }

    return score;
}

bool Initializer::ReconstructF(vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                            cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
{
    int N=0;
    for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
        if(vbMatchesInliers[i])
            N++;

    const float minGoodRatio = (N < INITIALIZER_GOOD_RATIO_SMALL_N) ? INITIALIZER_GOOD_RATIO_SMALL : INITIALIZER_GOOD_RATIO_LARGE;
    const int minTri = std::min(minTriangulated, std::max(INITIALIZER_MIN_TRI_HARD, N/2));

    // 从基础矩阵计算本质矩阵
    cv::Mat E21 = K.t()*F21*K;

    cv::Mat R1, R2, t;

    // 恢复4种运动假设
    DecomposeE(E21,R1,R2,t);  

    cv::Mat t1=t;
    cv::Mat t2=-t;

    // 使用4种假设重构并检查
    vector<cv::Point3f> vP3D1, vP3D2, vP3D3, vP3D4;
    vector<bool> vbTriangulated1,vbTriangulated2,vbTriangulated3, vbTriangulated4;
    float parallax1,parallax2, parallax3, parallax4;

    int nGood1 = CheckRT(R1,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D1, INITIALIZER_REPROJ_TH_FACTOR*mSigma2, vbTriangulated1, parallax1);
    int nGood2 = CheckRT(R2,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D2, INITIALIZER_REPROJ_TH_FACTOR*mSigma2, vbTriangulated2, parallax2);
    int nGood3 = CheckRT(R1,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D3, INITIALIZER_REPROJ_TH_FACTOR*mSigma2, vbTriangulated3, parallax3);
    int nGood4 = CheckRT(R2,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D4, INITIALIZER_REPROJ_TH_FACTOR*mSigma2, vbTriangulated4, parallax4);

    int maxGood = max(nGood1,max(nGood2,max(nGood3,nGood4)));

    R21 = cv::Mat();
    t21 = cv::Mat();

    int nMinGood = max(static_cast<int>(minGoodRatio*N),minTri);

    int nsimilar = 0;
    if(nGood1>INITIALIZER_NGOOD_SIMILAR_RATIO*maxGood)
        nsimilar++;
    if(nGood2>INITIALIZER_NGOOD_SIMILAR_RATIO*maxGood)
        nsimilar++;
    if(nGood3>INITIALIZER_NGOOD_SIMILAR_RATIO*maxGood)
        nsimilar++;
    if(nGood4>INITIALIZER_NGOOD_SIMILAR_RATIO*maxGood)
        nsimilar++;

    // 如果没有明显的优胜者或三角测量点不足，则拒绝初始化
    if(maxGood<nMinGood || nsimilar>1)
    {
        return false;
    }

    // 如果最佳重建有足够的视差初始化
    if(maxGood==nGood1)
    {
        if(parallax1>minParallax)
        {
            vP3D = vP3D1;
            vbTriangulated = vbTriangulated1;

            R1.copyTo(R21);
            t1.copyTo(t21);
            return true;
        }
    }else if(maxGood==nGood2)
    {
        if(parallax2>minParallax)
        {
            vP3D = vP3D2;
            vbTriangulated = vbTriangulated2;

            R2.copyTo(R21);
            t1.copyTo(t21);
            return true;
        }
    }else if(maxGood==nGood3)
    {
        if(parallax3>minParallax)
        {
            vP3D = vP3D3;
            vbTriangulated = vbTriangulated3;

            R1.copyTo(R21);
            t2.copyTo(t21);
            return true;
        }
    }else if(maxGood==nGood4)
    {
        if(parallax4>minParallax)
        {
            vP3D = vP3D4;
            vbTriangulated = vbTriangulated4;

            R2.copyTo(R21);
            t2.copyTo(t21);
            return true;
        }
    }

    return false;
}

bool Initializer::ReconstructH(vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
{
    int N=0;
    for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
        if(vbMatchesInliers[i])
            N++;

    const float minGoodRatio = (N < INITIALIZER_GOOD_RATIO_SMALL_N) ? INITIALIZER_GOOD_RATIO_SMALL : INITIALIZER_GOOD_RATIO_LARGE;
    const int minTri = std::min(minTriangulated, std::max(INITIALIZER_MIN_TRI_HARD, N/2));

    // 使用Faugeras (1988) 平面场景方法恢复8种运动假设

    cv::Mat invK = K.inv();
    cv::Mat A = invK*H21*K;

    cv::Mat U,w,Vt,V;
    cv::SVD::compute(A,w,U,Vt,cv::SVD::FULL_UV);
    V=Vt.t();

    float s = cv::determinant(U)*cv::determinant(Vt);

    float d1 = w.at<float>(0);
    float d2 = w.at<float>(1);
    float d3 = w.at<float>(2);

    if(d1/d2<INITIALIZER_SINGULAR_RATIO || d2/d3<INITIALIZER_SINGULAR_RATIO)
    {
        return false;
    }

    // 预计算平方值和公共子表达式，避免重复乘法
    const float d1sq = d1*d1;
    const float d2sq = d2*d2;
    const float d3sq = d3*d3;
    const float d1sq_m_d3sq = d1sq - d3sq;  // d1²-d3² 在多处使用

    vector<cv::Mat> vR, vt, vn;
    vR.reserve(8);
    vt.reserve(8);
    vn.reserve(8);

    //n'=[x1 0 x3] 4种可能性 e1=e3=1, e1=1 e3=-1, e1=-1 e3=1, e1=e3=-1
    float aux1 = sqrt((d1sq-d2sq)/d1sq_m_d3sq);
    float aux3 = sqrt((d2sq-d3sq)/d1sq_m_d3sq);
    float x1[] = {aux1,aux1,-aux1,-aux1};
    float x3[] = {aux3,-aux3,aux3,-aux3};

    //情况 d'=d2
    float aux_stheta = sqrt((d1sq-d2sq)*(d2sq-d3sq))/((d1+d3)*d2);

    float ctheta = (d2sq+d1*d3)/((d1+d3)*d2);
    float stheta[] = {aux_stheta, -aux_stheta, -aux_stheta, aux_stheta};

    for(int i=0; i<4; i++)
    {
        cv::Mat Rp=cv::Mat::eye(3,3,CV_32F);
        Rp.at<float>(0,0)=ctheta;
        Rp.at<float>(0,2)=-stheta[i];
        Rp.at<float>(2,0)=stheta[i];
        Rp.at<float>(2,2)=ctheta;

        cv::Mat R = s*U*Rp*Vt;
        vR.push_back(R);

        cv::Mat tp(3,1,CV_32F);
        tp.at<float>(0)=x1[i];
        tp.at<float>(1)=0;
        tp.at<float>(2)=-x3[i];
        tp*=d1-d3;

        cv::Mat t = U*tp;
        vt.push_back(t/cv::norm(t));

        cv::Mat np(3,1,CV_32F);
        np.at<float>(0)=x1[i];
        np.at<float>(1)=0;
        np.at<float>(2)=x3[i];

        cv::Mat n = V*np;
        if(n.at<float>(2)<0)
            n=-n;
        vn.push_back(n);
    }

    //情况 d'=-d2
    float aux_sphi = sqrt((d1sq-d2sq)*(d2sq-d3sq))/((d1-d3)*d2);

    float cphi = (d1*d3-d2sq)/((d1-d3)*d2);
    float sphi[] = {aux_sphi, -aux_sphi, -aux_sphi, aux_sphi};

    for(int i=0; i<4; i++)
    {
        cv::Mat Rp=cv::Mat::eye(3,3,CV_32F);
        Rp.at<float>(0,0)=cphi;
        Rp.at<float>(0,2)=sphi[i];
        Rp.at<float>(1,1)=-1;
        Rp.at<float>(2,0)=sphi[i];
        Rp.at<float>(2,2)=-cphi;

        cv::Mat R = s*U*Rp*Vt;
        vR.push_back(R);

        cv::Mat tp(3,1,CV_32F);
        tp.at<float>(0)=x1[i];
        tp.at<float>(1)=0;
        tp.at<float>(2)=x3[i];
        tp*=d1+d3;

        cv::Mat t = U*tp;
        vt.push_back(t/cv::norm(t));

        cv::Mat np(3,1,CV_32F);
        np.at<float>(0)=x1[i];
        np.at<float>(1)=0;
        np.at<float>(2)=x3[i];

        cv::Mat n = V*np;
        if(n.at<float>(2)<0)
            n=-n;
        vn.push_back(n);
    }

    int bestGood = 0;
    int secondBestGood = 0;    
    int bestSolutionIdx = -1;
    float bestParallax = -1;
    vector<cv::Point3f> bestP3D;
    vector<bool> bestTriangulated;

    // 我们重构所有假设并通过三角测量点和视差进行检查，而不是应用Faugeras论文中提出的可见性约束（对于低视差点可能失败）
    for(size_t i=0; i<8; i++)
    {
        float parallaxi;
        vector<cv::Point3f> vP3Di;
        vector<bool> vbTriangulatedi;
        int nGood = CheckRT(vR[i],vt[i],mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K,vP3Di, INITIALIZER_REPROJ_TH_FACTOR*mSigma2, vbTriangulatedi, parallaxi);

        if(nGood>bestGood)
        {
            secondBestGood = bestGood;
            bestGood = nGood;
            bestSolutionIdx = i;
            bestParallax = parallaxi;
            bestP3D = vP3Di;
            bestTriangulated = vbTriangulatedi;
        }
        else if(nGood>secondBestGood)
        {
            secondBestGood = nGood;
        }
    }

    if(secondBestGood<INITIALIZER_BEST_RATIO*bestGood && bestParallax>=minParallax && bestGood>minTri && bestGood>minGoodRatio*N)
    {
        vR[bestSolutionIdx].copyTo(R21);
        vt[bestSolutionIdx].copyTo(t21);
        vP3D = bestP3D;
        vbTriangulated = bestTriangulated;

        return true;
    }

    return false;
}

void Initializer::Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D)
{
    // 线性三角化（公共 DLT 实现，见 Converter::TriangulateDLT）
    if (!Converter::TriangulateDLT(P1, P2, kp1.pt.x, kp1.pt.y, kp2.pt.x, kp2.pt.y, x3D))
    {
        // w==0 退化：填充 inf，与原「x3D.rowRange(0,3)/w（w→0）」行为等价，
        // 由 CheckRT 的 isfinite 检查兜底拒绝该点。
        const float inf = std::numeric_limits<float>::infinity();
        x3D = (cv::Mat_<float>(3,1) << inf, inf, inf);
    }
}

void Initializer::Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T)
{
    float meanX = 0;
    float meanY = 0;
    const int N = vKeys.size();

    vNormalizedPoints.resize(N);

    for(int i=0; i<N; i++)
    {
        meanX += vKeys[i].pt.x;
        meanY += vKeys[i].pt.y;
    }

    const float invN = 1.0f / N;
    meanX *= invN;
    meanY *= invN;

    float meanDevX = 0;
    float meanDevY = 0;

    for(int i=0; i<N; i++)
    {
        float dx = vKeys[i].pt.x - meanX;
        float dy = vKeys[i].pt.y - meanY;

        vNormalizedPoints[i].x = dx;
        vNormalizedPoints[i].y = dy;

        // 避免函数调用 fabs
        meanDevX += (dx > 0) ? dx : -dx;
        meanDevY += (dy > 0) ? dy : -dy;
    }

    meanDevX *= invN;
    meanDevY *= invN;

    float sX = 1.0f/meanDevX;
    float sY = 1.0f/meanDevY;

    for(int i=0; i<N; i++)
    {
        vNormalizedPoints[i].x *= sX;
        vNormalizedPoints[i].y *= sY;
    }

    T = cv::Mat::eye(3,3,CV_32F);
    T.at<float>(0,0) = sX;
    T.at<float>(1,1) = sY;
    T.at<float>(0,2) = -meanX*sX;
    T.at<float>(1,2) = -meanY*sY;
}

int Initializer::CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                       const vector<Match> &vMatches12, vector<bool> &vbMatchesInliers,
                       const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax)
{
    // 校准参数
    const float fx = K.at<float>(0,0);
    const float fy = K.at<float>(1,1);
    const float cx = K.at<float>(0,2);
    const float cy = K.at<float>(1,2);

    vbGood = vector<bool>(vKeys1.size(),false);
    vP3D.resize(vKeys1.size());

    vector<float> vCosParallax;
    vCosParallax.reserve(vKeys1.size());

    // 相机1投影矩阵K[I|0]
    cv::Mat P1(3,4,CV_32F,cv::Scalar(0));
    K.copyTo(P1.rowRange(0,3).colRange(0,3));

    cv::Mat O1 = cv::Mat::zeros(3,1,CV_32F);

    // 相机2投影矩阵K[R|t]
    cv::Mat P2(3,4,CV_32F);
    R.copyTo(P2.rowRange(0,3).colRange(0,3));
    t.copyTo(P2.rowRange(0,3).col(3));
    P2 = K*P2;

    cv::Mat O2 = -R.t()*t;

    int nGood=0;

    for(size_t i=0, iend=vMatches12.size();i<iend;i++)
    {
        if(!vbMatchesInliers[i])
            continue;

        const cv::KeyPoint &kp1 = vKeys1[vMatches12[i].first];
        const cv::KeyPoint &kp2 = vKeys2[vMatches12[i].second];
        cv::Mat p3dC1;

        Triangulate(kp1,kp2,P1,P2,p3dC1);

        if(!isfinite(p3dC1.at<float>(0)) || !isfinite(p3dC1.at<float>(1)) || !isfinite(p3dC1.at<float>(2)))
        {
            vbGood[vMatches12[i].first]=false;
            continue;
        }

        // 检查视差
        cv::Mat normal1 = p3dC1 - O1;
        cv::Mat normal2 = p3dC1 - O2;
        // 内联计算向量范数和点积，避免多次cv::norm调用
        const float n1x = normal1.at<float>(0), n1y = normal1.at<float>(1), n1z = normal1.at<float>(2);
        const float n2x = normal2.at<float>(0), n2y = normal2.at<float>(1), n2z = normal2.at<float>(2);
        const float dist1Sq = n1x*n1x + n1y*n1y + n1z*n1z;
        const float dist2Sq = n2x*n2x + n2y*n2y + n2z*n2z;
        const float dotProd = n1x*n2x + n1y*n2y + n1z*n2z;

        float cosParallax = dotProd / sqrt(dist1Sq * dist2Sq);
        float dist1 = sqrt(dist1Sq);

        // 检查第一个相机前方的深度（仅当有足够的视差时，因为"无限远"点容易变为负深度）
        if(p3dC1.at<float>(2)<=0 && cosParallax<INITIALIZER_PARALLAX_COS_TH)
            continue;

        // 检查第二个相机前方的深度（仅当有足够的视差时，因为"无限远"点容易变为负深度）
        cv::Mat p3dC2 = R*p3dC1+t;

        if(p3dC2.at<float>(2)<=0 && cosParallax<INITIALIZER_PARALLAX_COS_TH)
            continue;

        // 检查第一幅图像中的重投影误差
        float im1x, im1y;
        float invZ1 = 1.0/p3dC1.at<float>(2);
        im1x = fx*p3dC1.at<float>(0)*invZ1+cx;
        im1y = fy*p3dC1.at<float>(1)*invZ1+cy;

        float squareError1 = (im1x-kp1.pt.x)*(im1x-kp1.pt.x)+(im1y-kp1.pt.y)*(im1y-kp1.pt.y);

        if(squareError1>th2)
            continue;

        // 检查第二幅图像中的重投影误差
        float im2x, im2y;
        float invZ2 = 1.0/p3dC2.at<float>(2);
        im2x = fx*p3dC2.at<float>(0)*invZ2+cx;
        im2y = fy*p3dC2.at<float>(1)*invZ2+cy;

        float squareError2 = (im2x-kp2.pt.x)*(im2x-kp2.pt.x)+(im2y-kp2.pt.y)*(im2y-kp2.pt.y);

        if(squareError2>th2)
            continue;

        vCosParallax.push_back(cosParallax);
        vP3D[vMatches12[i].first] = cv::Point3f(p3dC1.at<float>(0),p3dC1.at<float>(1),p3dC1.at<float>(2));
        nGood++;

        if(cosParallax<INITIALIZER_PARALLAX_COS_TH)
            vbGood[vMatches12[i].first]=true;
    }

    if(nGood>0)
    {
        sort(vCosParallax.begin(),vCosParallax.end());

        size_t idx = min(INITIALIZER_PARALLAX_PERCENTILE,int(vCosParallax.size()-1));
        parallax = acos(vCosParallax[idx])*180/CV_PI;
    }
    else
        parallax=0;

    return nGood;
}

void Initializer::DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t)
{
    cv::Mat u,w,vt;
    cv::SVD::compute(E,w,u,vt);

    u.col(2).copyTo(t);
    t=t/cv::norm(t);

    cv::Mat W(3,3,CV_32F,cv::Scalar(0));
    W.at<float>(0,1)=-1;
    W.at<float>(1,0)=1;
    W.at<float>(2,2)=1;

    R1 = u*W*vt;
    if(cv::determinant(R1)<0)
        R1=-R1;

    R2 = u*W.t()*vt;
    if(cv::determinant(R2)<0)
        R2=-R2;
}

} //namespace ORB_SLAM2
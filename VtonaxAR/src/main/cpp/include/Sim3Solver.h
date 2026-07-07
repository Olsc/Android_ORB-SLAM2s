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

#ifndef SIM3SOLVER_H
#define SIM3SOLVER_H

#include <opencv2/opencv.hpp>
#include <vector>

#include "KeyFrame.h"

namespace ORB_SLAM2
{

class Sim3Solver
{
public:

    Sim3Solver(KeyFrame* pKF1, KeyFrame* pKF2, const std::vector<MapPoint*> &vpMatched12);

    void SetRansacParameters(double probability = 0.99, int minInliers = 6 , int maxIterations = 300);

    cv::Mat find(std::vector<bool> &vbInliers12, int &nInliers);

    cv::Mat iterate(int nIterations, bool &bNoMore, std::vector<bool> &vbInliers, int &nInliers);

    cv::Mat GetEstimatedRotation();
    cv::Mat GetEstimatedTranslation();
    float GetEstimatedScale();

protected:

    void ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C);

    void ComputeSim3(cv::Mat &P1, cv::Mat &P2);

    void CheckInliers();

    void Project(const std::vector<cv::Mat> &vP3Dw, std::vector<cv::Point2f> &vP2D, cv::Mat Tcw, cv::Mat K);
    void FromCameraToImage(const std::vector<cv::Mat> &vP3Dc, std::vector<cv::Point2f> &vP2D, cv::Mat K);

protected:

    // 关键帧和匹配
    KeyFrame* mpKF1;
    KeyFrame* mpKF2;

    std::vector<cv::Mat> mvX3Dc1;
    std::vector<cv::Mat> mvX3Dc2;
    std::vector<MapPoint*> mvpMapPoints1;
    std::vector<MapPoint*> mvpMapPoints2;
    std::vector<MapPoint*> mvpMatches12;
    std::vector<size_t> mvnIndices1;
    std::vector<size_t> mvSigmaSquare1;
    std::vector<size_t> mvSigmaSquare2;
    std::vector<size_t> mvnMaxError1;
    std::vector<size_t> mvnMaxError2;

    int N;
    int mN1;

    // 当前估计
    cv::Mat mR12i;
    cv::Mat mt12i;
    float ms12i;
    cv::Mat mT12i;
    cv::Mat mT21i;
    std::vector<bool> mvbInliersi;
    int mnInliersi;

    // 当前Ransac状态
    int mnIterations;
    std::vector<bool> mvbBestInliers;
    int mnBestInliers;
    cv::Mat mBestT12;
    cv::Mat mBestRotation;
    cv::Mat mBestTranslation;
    float mBestScale;

    // 随机选择的索引
    std::vector<size_t> mvAllIndices;

    // 投影
    std::vector<cv::Point2f> mvP1im1;
    std::vector<cv::Point2f> mvP2im2;

    // RANSAC概率
    double mRansacProb;

    // RANSAC最小内点数
    int mRansacMinInliers;

    // RANSAC最大迭代次数
    int mRansacMaxIts;

    // 内点/外点阈值。e = dist(Pi,T_ij*Pj)^2 < 5.991*mSigma2
    float mTh;
    float mSigma2;

    // 标定参数
    cv::Mat mK1;
    cv::Mat mK2;
};

} //namespace ORB_SLAM2

#endif // SIM3SOLVER_H

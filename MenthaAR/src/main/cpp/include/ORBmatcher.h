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

#ifndef ORBMATCHER_H
#define ORBMATCHER_H

#include<vector>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"MapPoint.h"
#include"KeyFrame.h"
#include"Frame.h"
#include"Config.h"

namespace ORB_SLAM2
{

#include "HBSTTypes.h"
#include <utility>

using namespace std;

class ORBmatcher
{    
public:

    ORBmatcher(float nnratio=ORB_MATCHER_DEFAULT_NN_RATIO, bool checkOri=true);

    // 计算两个ORB描述子之间的汉明距离
    static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);
    static int DescriptorDistance(const uint8_t* pa, const uint8_t* pb);

    // 基于 HBST 的帧间特征匹配，支持比率测试
    int SearchByHBST(Frame &F1, Frame &F2, std::vector<cv::Point2f> &vbPrevMatched);

    // 基于 HBST 的关键帧间特征匹配
    int SearchByHBST(KeyFrame *pKF1, KeyFrame *pKF2, std::vector<MapPoint*> &vpMatches12);

    // 使用 HBST 替代 BoW 进行快速特征匹配
    // 用于重定位和回环检测
    int SearchByHBST(KeyFrame *pKF, Frame &F, std::vector<MapPoint*> &vpMapPointMatches);

    // 从特征点描述子构造 HBST 的 Matchable 格式
    static HBSTTree::MatchableVector getMatchables(const cv::Mat &descriptors, const std::vector<size_t>& objects);

    // 在帧关键点和投影地图点之间搜索匹配，返回匹配数量
    // 用于跟踪局部地图(跟踪)
    int SearchByProjection(Frame &F, const std::vector<MapPoint*> &vpMapPoints, const float th=ORB_MATCHER_DEFAULT_PROJ_TH);

    // 将上一帧中跟踪的地图点投影到当前帧并搜索匹配
    // 用于从前一帧跟踪(跟踪)
    int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono);

    // 将关键帧中看到的地图点投影到帧中并搜索匹配
    // 用于重定位(跟踪)
    int SearchByProjection(Frame &CurrentFrame, KeyFrame* pKF, const std::set<MapPoint*> &sAlreadyFound, const float th, const int ORBdist);

    // 使用相似变换投影地图点并搜索匹配
    // 用于回环检测(闭环)
    int SearchByProjection(KeyFrame* pKF, cv::Mat Scw, const std::vector<MapPoint*> &vpPoints, std::vector<MapPoint*> &vpMatched, int th);

    // 地图初始化的匹配(仅用于单目情况)
    int SearchForInitialization(Frame &F1, Frame &F2, std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12, int windowSize=ORB_MATCHER_INIT_WINDOW);

    // 匹配以三角化新的地图点，检查对极约束
    int SearchForTriangulation(KeyFrame *pKF1, KeyFrame* pKF2, cv::Mat F12,
                               std::vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo);

    // 在KF1和KF2之间搜索地图点匹配，通过Sim3变换[s12*R12|t12]
    // 在双目和RGB-D情况下，s12=1
    int SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches12, const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th);

    // 将地图点投影到关键帧并搜索重复的地图点
    int Fuse(KeyFrame* pKF, const vector<MapPoint *> &vpMapPoints, const float th=ORB_MATCHER_DEFAULT_FUSE_TH);

    // 使用给定的Sim3将地图点投影到关键帧并搜索重复的地图点
    int Fuse(KeyFrame* pKF, cv::Mat Scw, const std::vector<MapPoint*> &vpPoints, float th, vector<MapPoint *> &vpReplacePoint);

public:

    static const int TH_LOW;
    static const int TH_HIGH;
    static const int HISTO_LENGTH;

protected:

    bool CheckDistEpipolarLine(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &F12, const KeyFrame *pKF);

    float RadiusByViewingCos(const float &viewCos);

    void ComputeThreeMaxima(std::vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);

    float mfNNratio;
    bool mbCheckOrientation;
};

}// namespace ORB_SLAM2

#endif // ORBMATCHER_H

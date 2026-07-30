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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"

#include<opencv2/core/core.hpp>
#include<mutex>
#include<atomic>

namespace ORB_SLAM2
{

class KeyFrame;
class Map;
class Frame;

class MapPoint
{
public:
    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);
    MapPoint(const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF);
    // 用于加载地图点的轻量级构造函数，无观测值
    MapPoint(const cv::Mat &Pos, Map* pMap);

    void SetWorldPos(const cv::Mat &Pos);
    cv::Mat GetWorldPos();
    void GetWorldPos(cv::Point3f& out);

    cv::Mat GetNormal();
    void GetNormal(cv::Point3f& out);
    KeyFrame* GetReferenceKeyFrame();

    std::map<KeyFrame*,size_t> GetObservations();
    void ShareObservations(std::map<KeyFrame*, int>& counter, unsigned long excludeId = -1);
    int GetRedundantObservationsCount(KeyFrame* pKF, int scaleLevel);
    int Observations() const;

    void AddObservation(KeyFrame* pKF,size_t idx);
    void EraseObservation(KeyFrame* pKF);

    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(MapPoint* pMP);    
    MapPoint* GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();
    inline void SetDescriptor(const cv::Mat &desc){
        if(desc.empty()) return;
        std::lock_guard<std::mutex> lock(mMutexFeatures);
        mDescriptor = desc.clone();
    }

    void UpdateNormalAndDepth();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    int PredictScale(const float &currentDist, KeyFrame*pKF);
    int PredictScale(const float &currentDist, Frame* pF);
    inline void SetNormalAndDepth(const cv::Mat &normal, float minD, float maxD){
        std::lock_guard<std::mutex> lock(mMutexFeatures);
        mNormalVector = normal.clone();
        mfMinDistance = minD;
        mfMaxDistance = maxD;
    }

public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    std::atomic<int> nObs;

    // 地图ID，用于多地图支持
    int mnMapId;

    void SetMapId(int id);
    int GetMapId();
    // 标记从持久化地图加载的点用于可视化
    bool mbFromLoadedMap = false;
    // 标记在当前帧中匹配的点（临时）
    bool mbMatchedInCurrentFrame = false;

    // 跟踪使用的变量
    float mTrackProjX;
    float mTrackProjY;
    float mTrackProjXR;
    bool mbTrackInView;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // 局部建图使用的变量
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // 回环闭合使用的变量
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;    
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;



protected:    

     // 绝对坐标位置
     cv::Mat mWorldPos;

     // 观测该点的关键帧及在关键帧中的索引
     std::map<KeyFrame*,size_t> mObservations;

     // 平均观测方向
     cv::Mat mNormalVector;

     // 用于快速匹配的最佳描述子
     cv::Mat mDescriptor;

     // 参考关键帧
     KeyFrame* mpRefKF;

     // 跟踪计数器
     int mnVisible;
     int mnFound;

     // 坏点标志（我们目前不从内存中删除地图点）
    std::atomic<bool> mbBad;
    MapPoint* mpReplaced;

     // 尺度不变性距离
     float mfMinDistance;
     float mfMaxDistance;

     Map* mpMap;

     mutable std::mutex mMutexPos;
     mutable std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM2

#endif // MAPPOINT_H

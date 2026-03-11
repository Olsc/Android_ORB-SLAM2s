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

#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"FrameDrawer.h"
#include"Map.h"
#include"LocalMapping.h"
#include"LoopClosing.h"
#include"Frame.h"
#include "ORBVocabulary.h"
#include"KeyFrameDatabase.h"
#include"ORBextractor.h"
#include "Initializer.h"
#include "System.h"

#include <deque>

#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <chrono>

namespace ORB_SLAM2
{

class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;

class Tracking
{  

public:
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);

    // 预处理输入并调用Track()。提取特征并执行单目处理。
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetMap(Map* pMap);

    // 加载新设置
    // 焦距应该相似，否则在投影点时尺度预测会失败
    // TODO: 修改MapPoint::PredictScale以考虑焦距
    void ChangeCalibration(const string &strSettingPath);

    // 如果您已停用局部建图并且只想定位相机，请使用此函数。
    void InformOnlyTracking(const bool &flag);

    // 后台全局重定位线程控制
    void StartGlobalRelocThread();
    void StopGlobalRelocThread();

    // 对齐工具
    bool HasMapAlignment() const { return mbHaveMapAlign; }
    void ClearMapAlignment();
    void ClearRelocCache();
    cv::Mat GetMapAlignedPose(const cv::Mat &TcwSlam);
    bool HasLoadedMapData() const;  // 检查是否已加载地图数据
    // 读取对齐置信度（供UI显示）
    float GetAlignConfidence() const {
        std::unique_lock<std::mutex> lk(mMutexReloc);
        return mAlignConfidence;
    }
    // 背景匹配分数（0-1），即使未完成PnP也能反映进入目标区域的可能性
    float GetRelocMatchScore() const { return mRelocMatchScore.load(); }
    
    // 平滑对齐更新：减少单帧抖动
    void UpdateAlignmentSmooth(const cv::Mat &T_new, int inliers, float confidence, double ts);
    
    // 为加载的地图点构建/重建缓存的参考描述符
    void BuildLoadedRefCache();

    // 后台重定位的运行时配置
    void SetRelocConfig(int topKWords, int maxCandidates, int matchChunk, int bgSleepUs,
                        int maxBindInliers, int maxProjBinds);

    // 获取当前激活的地图ID
    int GetCurrentMapId() const { return mnCurrentMapId; }






    // 跟踪状态
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    // 当前激活的地图ID
    std::atomic<int> mnCurrentMapId;

    // 跟踪模式
    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // 输入传感器
    int mSensor;

    // 当前帧
    Frame mCurrentFrame;
    cv::Mat mImGray;
    cv::Mat mLastImGray;

    // 初始化变量（单目）
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // 用于在执行结束时恢复完整相机轨迹的列表。
    // 基本上我们存储每帧的参考关键帧及其相对变换
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // 如果局部建图被停用并且我们仅执行定位，则为true
    bool mbOnlyTracking;

    // 重置函数
    void Reset();

    // 仅清除跟踪状态而不清除地图
    void ClearTrackingState();

protected:

    // 主跟踪函数。它与输入传感器无关。
    void Track();

    // 单目的地图初始化
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();

    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    // 与实时SLAM解耦的重型全局匹配循环
    void GlobalRelocLoop();
    // 从快照投影加载的地图点并绑定到当前帧
    void BindLoadedMapPointsUsingSnapshots();

    // 在仅执行定位的情况下，当没有与地图中的点匹配时，此标志为true。
    // 如果与时间点有足够的匹配，跟踪仍将继续。
    // 在这种情况下，我们正在做视觉里程计。系统将尝试进行重定位以恢复
    // 到地图的"零漂移"定位。
    bool mbVO;

    //其他线程指针
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    //ORB
    ORBextractor* mpORBextractorLeft;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // 初始化（仅用于单目）
    Initializer* mpInitializer;
    double mLastInitAttemptTime; // 上次尝试初始化的时间戳，用于限频

    //局部地图
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    // 系统
    System* mpSystem;
    
    //绘制器
    FrameDrawer* mpFrameDrawer;

    //地图
    Map* mpMap;

    //校准矩阵
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;

    //新关键帧规则（根据fps）
    int mMinFrames;
    int mMaxFrames;

    // 当前帧中的匹配数
    int mnMatchesInliers;

    //上一帧、关键帧和重定位信息
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    //运动模型
    cv::Mat mVelocity;

    //颜色顺序（true RGB，false BGR，如果是灰度图则忽略）
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;

    // 后台全局重定位状态
    std::thread* mptGlobalReloc = nullptr;
    std::atomic<bool> mbRelocThreadStop{false};
    mutable std::mutex mMutexReloc;
    std::condition_variable mCvReloc; // 生命周期事件驱动的唤醒，避免使用延时
    cv::Mat mLastDesc; // 描述符快照
    std::vector<cv::KeyPoint> mLastKeysUn; // 关键点快照
    int mLastN = 0;
    double mLastTimestamp = 0.0;
    cv::Mat mLastTcwSlam; // 当前SLAM位姿快照
    // 快照版本号（生产-消费），用于无延时唤醒
    std::atomic<unsigned long long> mSnapSeqProduced{0ULL};
    std::atomic<unsigned long long> mSnapSeqConsumed{0ULL};

    // 从SLAM世界到加载地图世界的稳定对齐
    cv::Mat mT_map_from_slam; // 4x4
    bool mbHaveMapAlign = false;
    float mAlignConfidence = 0.0f;
    double mLastAlignTs = 0.0;
    std::atomic<float> mRelocMatchScore{0.0f};
    
    // 平滑对齐更新机制：使用EMA（指数移动平均）减少抖动，性能优化版本
    cv::Mat mSmoothedT_map_from_slam;  // 平滑后的对齐变换
    int mAlignUpdateCount = 0;  // 对齐更新计数
    int mAlignSkipCounter = 0;  // 跳帧计数器，用于降低更新频率

    // 为加载的地图点缓存的参考描述符（用于后台匹配）
    cv::Mat mRefDesc; // 描述符行
    std::vector<MapPoint*> mRefIdxToMP;
    size_t mRefCachedMPCount = 0;
    double mRefLastBuildTs = 0.0;
    std::atomic<bool> mRefBuilding{false};
    // 倒排索引：视觉词id -> mRefIdxToMP中的索引
    std::unordered_map<unsigned int, std::vector<int>> mRefInverted;
    // 加载地图点的不可变快照以避免竞争
    struct RefMPSnapshot {
        cv::Point3f Pw;
        float minD;
        float maxD;
        int mapId = 0;
    };
    std::vector<RefMPSnapshot> mRefSnapshots;
    
    // 简单的3D网格索引，用于加速空间查询
    struct LoadedMapGrid {
        float minX=0, maxX=0, minY=0, maxY=0, minZ=0, maxZ=0;
        float cellSize = 10.0f; // 默认10米
        int nCols=0, nRows=0, nSlices=0;
        std::vector<std::vector<int>> cells;
        
        void Clear() { cells.clear(); }
        // 构建网格
        void Build(const std::vector<RefMPSnapshot>& snaps, float size = 10.0f);
        // 获取包围盒内的候选点 (原始版本,返回矩形区域)
        void GetCandidatesInBBox(const cv::Point3f& center, float radius, std::vector<int>& outIndices) const;
        // 获取包围盒内的候选点 (优化版本,精确圆形过滤)
        void GetCandidatesInSphere(const cv::Point3f& center, float radius, 
                                  const std::vector<RefMPSnapshot>& snaps,
                                  std::vector<int>& outIndices) const;
    };
    LoadedMapGrid mRefGrid;

    // 失败滞后
    int mConsecutiveFail = 0;
    int mConsecutiveLostFrames = 0;

    // ===== 异步重定位结果缓冲区（仅对齐）=====
    struct RelocAlignResult{
        cv::Mat T_map_from_slam; // 4x4
        int inliers = 0;
        float confidence = 0.0f;
        double ts = 0.0;
        unsigned long long seq = 0ULL;
        int mapId = 0;
    };
    mutable std::mutex mMutexRelocBuf;
    RelocAlignResult mRelocBuf; // 由BG线程产生
    unsigned long long mRelocSeqProduced = 0ULL;
    unsigned long long mRelocSeqConsumed = 0ULL;

    void PublishRelocAlignment(const cv::Mat &TmapFromSlam, int inliers, float conf, double ts, int mapId);
    bool TryConsumeRelocAlignment(RelocAlignResult &out);

    // ===== 配置旋钮 =====
    int mCfgTopKWords = 6;
    int mCfgMaxCandidates = 8000;
    int mCfgMatchChunk = 2000;
    int mCfgBgSleepUs = 20000; // 兼容旧接口，不再依赖延时
    int mCfgMaxBindInliers = 120;
    int mCfgMaxProjBinds = 200;
    // 重定位冷却以避免在明显不匹配时进行重型PnP
    int mRelocCooldownFrames = 0;

    int mPendingMapId = -1;
    int mPendingMapCount = 0;
    RelocAlignResult mPendingAlign;
    int mMapSwitchCooldownFrames = 0;
    int mNoCurMapLoadedInliersFrames = 0;
    int mLastAcceptedAlignInliers = 0;
    
    // 最近一次跟踪的内点数（原子变量，供后台线程读取）
    std::atomic<int> mLastTrackingInliers{0};
    // 最近一次跟踪状态（供后台线程判断是否需要运行）
    std::atomic<bool> mTrackingOK{false};
    // 后台线程上次运行时间戳
    std::chrono::steady_clock::time_point mLastBgRunTime;
    // 连续重定位成功帧计数
    int mRelocSuccessFrames = 0;
    // 临时对齐变换矩阵（尚未确认）
    cv::Mat mPendingT_map_from_slam;
    float mPendingConfidence = 0.0f;
    int mPendingInliers = 0;
    
    // 重试计数器，防止GlobalRelocLoop死循环
    int mRefCacheRetryCount = 0;
    static constexpr int MAX_REF_CACHE_RETRIES = 10;
};

} //namespace ORB_SLAM2

#endif // TRACKING_H

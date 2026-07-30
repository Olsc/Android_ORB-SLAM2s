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

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"

#include <mutex>
#include <condition_variable>
#include <atomic>


namespace ORB_SLAM2
{

class Tracking;
class LoopClosing;
class Map;

class LocalMapping
{
public:
    LocalMapping(Map* pMap);

    void SetLoopCloser(LoopClosing* pLoopCloser);

    void SetTracker(Tracking* pTracker);
    void SetMap(Map* pMap);
    void ClearQueues();

    // 主函数
    void Run();

    void InsertKeyFrame(KeyFrame* pKF);

    // 线程同步
    void RequestStop();
    void CancelStopRequest();

    // 等待 LM 进入 Stopped 状态，最多等 timeoutMs 毫秒。返回时调用者仍需 isStopped() 确认。
    // 替代外部对 isStopped() 的盲轮询（usleep 轮询）。
    void WaitForStopped(int timeoutMs);

    void RequestReset();
    void WaitForResetComplete();
    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

    void InterruptBA();

    void RequestFinish();
    bool isFinished();

    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

protected:

    bool CheckNewKeyFrames();
    void ProcessNewKeyFrame();
    void CreateNewMapPoints();

    void MapPointCulling();
    void SearchInNeighbors();

    void KeyFrameCulling();
    void CheckLimits();

    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;

    bool mbResetComplete = false;
    std::mutex mMutexResetComplete;
    std::condition_variable mCvResetComplete;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Map* mpMap;

    LoopClosing* mpLoopCloser;
    Tracking* mpTracker;

    std::list<KeyFrame*> mlNewKeyFrames;

    KeyFrame* mpCurrentKeyFrame;

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    std::atomic<bool> mbAbortBA;

    std::atomic<bool> mbStopped;
    std::atomic<bool> mbStopRequested;
    std::atomic<bool> mbNotStop;
    std::mutex mMutexStop;

    std::atomic<bool> mbAcceptKeyFrames;
    std::mutex mMutexAccept;

    // 事件驱动唤醒：InsertKeyFrame/RequestStop/Release/RequestFinish/RequestReset
    std::mutex mMutexEvent;
    std::condition_variable mCvEvent;
};

} //namespace ORB_SLAM2

#endif // LOCALMAPPING_H

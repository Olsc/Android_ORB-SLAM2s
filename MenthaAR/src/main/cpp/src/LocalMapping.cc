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

#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "Config.h"
#include "Converter.h"

#include<mutex>
#include<chrono>
#include<algorithm>
#include "MenthaProfiler.h" // 性能分析器

namespace ORB_SLAM2
{

LocalMapping::LocalMapping(Map *pMap):
    mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap)
{
    mbAbortBA.store(false);
    mbStopped.store(false);
    mbStopRequested.store(false);
    mbNotStop.store(false);
    mbAcceptKeyFrames.store(true);
}

void LocalMapping::SetLoopCloser(LoopClosing* pLoopCloser)
{
    mpLoopCloser = pLoopCloser;
}

void LocalMapping::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LocalMapping::SetMap(Map* pMap)
{
    mpMap = pMap;
}

void LocalMapping::ClearQueues()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.clear();
    mlpRecentAddedMapPoints.clear();
}

void LocalMapping::Run()
{
    VT_PROFILE_FUNCTION();
    mbFinished = false;

    while(1)
    {
        // 跟踪线程将看到局部建图线程正忙
        SetAcceptKeyFrames(false);

        // 检查队列中是否有关键帧
        if(CheckNewKeyFrames())
        {
            {
                VT_PROFILE_SCOPE("LocalMapping::ProcessNewKeyFrame");
                // BoW 转换并插入地图
                ProcessNewKeyFrame();
            }

            {
                VT_PROFILE_SCOPE("LocalMapping::MapPointCulling");
                // 检查最近的地图点
                MapPointCulling();
            }

            {
                VT_PROFILE_SCOPE("LocalMapping::CreateNewMapPoints");
                // 三角化新的地图点
                CreateNewMapPoints();
            }

            if(!CheckNewKeyFrames())
            {
                VT_PROFILE_SCOPE("LocalMapping::SearchInNeighbors");
                // 在邻近关键帧中寻找更多匹配并融合重复点
                SearchInNeighbors();
            }

            mbAbortBA.store(false);

            if(!CheckNewKeyFrames() && !stopRequested())
            {
                // 局部 BA
                if(mpMap->KeyFramesInMap()>=LOCAL_BA_MIN_KEYFRAMES)
                {
                    VT_PROFILE_SCOPE("LocalMapping::LocalBundleAdjustment");
                    Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame, reinterpret_cast<bool*>(&mbAbortBA), mpMap);
                }

                {
                    VT_PROFILE_SCOPE("LocalMapping::KeyFrameCulling");
                    // 检查冗余的局部关键帧
                    KeyFrameCulling();
                }

                {
                    VT_PROFILE_SCOPE("LocalMapping::CheckLimits");
                    // 检查地图限制（统一管理）
                    CheckLimits();
                }
            }

            {
                VT_PROFILE_SCOPE("LocalMapping::InsertLoopKF");
                mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
            }
        }
        else if(Stop())
        {
            // 安全停止区域
            VT_PROFILE_SCOPE("LocalMapping::Stopped");
            {
                std::unique_lock<std::mutex> lock(mMutexEvent);
                while(isStopped() && !CheckFinish())
                {
                    // 阻塞等待 Release() 唤醒
                    if(mCvEvent.wait_for(lock, std::chrono::milliseconds(LOCAL_MAPPING_STOP_WAIT_TIMEOUT_MS))
                            == std::cv_status::timeout)
                    {
                        // 超时了还没人 Release — 自动恢复，避免 LM 永久卡死
                        unique_lock<mutex> stopLock(mMutexStop);
                        mbStopped.store(false);
                        mbStopRequested.store(false);
                        // 继续主循环，跟踪线程会看到 LM 已恢复正常
                    }
                }
            }
            if(CheckFinish())
                break;
        }

        ResetIfRequested();

        // 跟踪线程将看到局部建图线程正忙
        SetAcceptKeyFrames(true);

        if(CheckFinish())
            break;

        // 等待事件（新 KF/Stop/Finish/Reset），有事件立即唤醒，最多等 3ms
        {
            std::unique_lock<std::mutex> lock(mMutexEvent);
            mCvEvent.wait_for(lock, std::chrono::milliseconds(LOCAL_MAPPING_EVENT_WAIT_MS));
        }
    }

    SetFinish();
}

void LocalMapping::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    mbAbortBA.store(true);
    mCvEvent.notify_one();
}

bool LocalMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}

void LocalMapping::ProcessNewKeyFrame()
{
    {
        std::unique_lock<std::mutex> lock(mMutexNewKFs);
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
    }

    // 将地图点关联到新关键帧，并更新法线和描述子
    const vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

    for(size_t i=0; i<vpMapPointMatches.size(); i++)
    {
        MapPoint* pMP = vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))
                {
                    pMP->AddObservation(mpCurrentKeyFrame, i);
                    pMP->UpdateNormalAndDepth();
                    pMP->ComputeDistinctiveDescriptors();
                }
                else // 这种情况只发生于跟踪线程插入的新双目点
                {
                    mlpRecentAddedMapPoints.push_back(pMP);
                }
            }
        }
    }    

    // 更新共视步图中的链接
    mpCurrentKeyFrame->UpdateConnections();

    // 将关键帧插入地图
    mpMap->AddKeyFrame(mpCurrentKeyFrame);
}

void LocalMapping::MapPointCulling()
{
    // 检查最近添加的地图点
    list<MapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

    int nThObs = MAPPOINT_MIN_OBSERVATIONS_MONO;
    const int cnThObs = nThObs;

    while(lit!=mlpRecentAddedMapPoints.end())
    {
        MapPoint* pMP = *lit;
        if(pMP->isBad())
        {
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        // 跳过加载的地图点，不要删除它们
        else if(pMP->mbFromLoadedMap)
        {
            lit = mlpRecentAddedMapPoints.erase(lit); // 从"最近添加"列表移除，但不标记为bad
        }
        else if(pMP->GetFoundRatio()<MAPPOINT_MIN_FOUND_RATIO )
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=MAPPOINT_CULL_KF_GAP_CHECK && pMP->Observations()<=cnThObs)
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=MAPPOINT_CULL_KF_GAP_REMOVE)
            lit = mlpRecentAddedMapPoints.erase(lit);
        else
            lit++;
    }
}

void LocalMapping::CreateNewMapPoints()
{
    // 在共视图中检索邻近关键帧
    int nn = LOCAL_MAPPING_TRIANGULATION_NEIGHBORS;
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

    ORBmatcher matcher(ORB_MATCHER_NNRATIO_TRIANGULATION,false);

    cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation();
    cv::Mat Rwc1 = Rcw1.t();
    cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();
    cv::Mat Tcw1(3,4,CV_32F);
    Rcw1.copyTo(Tcw1.colRange(0,3));
    tcw1.copyTo(Tcw1.col(3));
    cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();

    const float &fx1 = mpCurrentKeyFrame->fx;
    const float &fy1 = mpCurrentKeyFrame->fy;
    const float &cx1 = mpCurrentKeyFrame->cx;
    const float &cy1 = mpCurrentKeyFrame->cy;
    const float &invfx1 = mpCurrentKeyFrame->invfx;
    const float &invfy1 = mpCurrentKeyFrame->invfy;

    const float ratioFactor = LOCAL_MAPPING_TRIANGULATION_RATIO_FACTOR*mpCurrentKeyFrame->mfScaleFactor;

    int nnew=0;

    // 使用极线约束搜索匹配并三角化
    for(size_t i=0; i<vpNeighKFs.size(); i++)
    {
        if(i>0 && CheckNewKeyFrames())
            return;

        KeyFrame* pKF2 = vpNeighKFs[i];

        // 首先检查基线是否太短
        cv::Mat Ow2 = pKF2->GetCameraCenter();
        cv::Mat vBaseline = Ow2-Ow1;
        const float baseline = cv::norm(vBaseline);

        {
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(TRIANGULATION_DEPTH_PERCENTILE);
            const float ratioBaselineDepth = baseline/medianDepthKF2;

            if(ratioBaselineDepth<LOCAL_MAPPING_TRIANGULATION_BASELINE_RATIO)
                continue;
        }

        // 计算基础矩阵
        cv::Mat F12 = ComputeF12(mpCurrentKeyFrame,pKF2);

        // 搜索满足极线约束的匹配
        vector<pair<size_t,size_t> > vMatchedIndices;
        matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,F12,vMatchedIndices,false);

        cv::Mat Rcw2 = pKF2->GetRotation();
        cv::Mat Rwc2 = Rcw2.t();
        cv::Mat tcw2 = pKF2->GetTranslation();
        cv::Mat Tcw2(3,4,CV_32F);
        Rcw2.copyTo(Tcw2.colRange(0,3));
        tcw2.copyTo(Tcw2.col(3));

        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;
        const float &invfx2 = pKF2->invfx;
        const float &invfy2 = pKF2->invfy;

        // 对每个匹配进行三角化
        const int nmatches = vMatchedIndices.size();

        cv::Mat w,u,vt;

        for(int ikp=0; ikp<nmatches; ikp++)
        {
            const int &idx1 = vMatchedIndices[ikp].first;
            const int &idx2 = vMatchedIndices[ikp].second;

            const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeysUn[idx1];
            // 单目模式不需要双目信息
            const float kp1_ur = -1.0f;
            bool bStereo1 = false;

            const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
            const float kp2_ur = -1.0f;
            bool bStereo2 = false;

            // 检查光线之间的视差（标量替换优化 cv::Mat 分配）
            const float xn1x = (kp1.pt.x-cx1)*invfx1;
            const float xn1y = (kp1.pt.y-cy1)*invfy1;
            // xn2 = [(kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1]
            const float xn2x = (kp2.pt.x-cx2)*invfx2;
            const float xn2y = (kp2.pt.y-cy2)*invfy2;
            // ray1 = Rwc1 * [xn1x, xn1y, 1]  (z 分量为常数 1，直接取 Rwc 第三列)
            const float r1x = Rwc1.at<float>(0,0)*xn1x + Rwc1.at<float>(0,1)*xn1y + Rwc1.at<float>(0,2);
            const float r1y = Rwc1.at<float>(1,0)*xn1x + Rwc1.at<float>(1,1)*xn1y + Rwc1.at<float>(1,2);
            const float r1z = Rwc1.at<float>(2,0)*xn1x + Rwc1.at<float>(2,1)*xn1y + Rwc1.at<float>(2,2);
            // ray2 = Rwc2 * [xn2x, xn2y, 1]
            const float r2x = Rwc2.at<float>(0,0)*xn2x + Rwc2.at<float>(0,1)*xn2y + Rwc2.at<float>(0,2);
            const float r2y = Rwc2.at<float>(1,0)*xn2x + Rwc2.at<float>(1,1)*xn2y + Rwc2.at<float>(1,2);
            const float r2z = Rwc2.at<float>(2,0)*xn2x + Rwc2.at<float>(2,1)*xn2y + Rwc2.at<float>(2,2);
            const float dotProduct = r1x*r2x + r1y*r2y + r1z*r2z;
            const float norm1Sq = r1x*r1x + r1y*r1y + r1z*r1z;
            const float norm2Sq = r2x*r2x + r2y*r2y + r2z*r2z;
            const float cosParallaxRays = dotProduct / sqrt(norm1Sq * norm2Sq);

            float cosParallaxStereo = cosParallaxRays+1;
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;

            cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);

            cv::Mat x3D;
            if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<LOCAL_MAPPING_TRIANGULATION_PARALLAX_TH))
            {
                // 线性三角化（公共 DLT 实现，见 Converter::TriangulateDLT；w==0 时返回 false）
                if (!Converter::TriangulateDLT(Tcw1, Tcw2, xn1x, xn1y, xn2x, xn2y, x3D))
                    continue;

            }
            else
                continue; // 没有双目信息且视差非常小

            cv::Mat x3Dt = x3D.t();

            // 检查三角化点是否在相机前方
            float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
            if(z1<=0)
                continue;

            float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
            if(z2<=0)
                continue;

            // 检查第一个关键帧中的重投影误差（交叉相乘消除 1.0/z1 浮点除法，100% 数学等价）
            const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
            const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
            const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);

            {
                float dx1 = fx1*x1 + (cx1 - kp1.pt.x)*z1;
                float dy1 = fy1*y1 + (cy1 - kp1.pt.y)*z1;
                if((dx1*dx1 + dy1*dy1) > OPTIMIZER_CHI2_TH_2D * sigmaSquare1 * z1 * z1)
                    continue;
            }

            // 检查第二个关键帧中的重投影误差（交叉相乘消除 1.0/z2 浮点除法，100% 数学等价）
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
            const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
            const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
            {
                float dx2 = fx2*x2 + (cx2 - kp2.pt.x)*z2;
                float dy2 = fy2*y2 + (cy2 - kp2.pt.y)*z2;
                if((dx2*dx2 + dy2*dy2) > OPTIMIZER_CHI2_TH_2D * sigmaSquare2 * z2 * z2)
                    continue;
            }

            // 检查尺度一致性
            // 使用平方距离进行快速零值检查，避免不必要的sqrt
            cv::Mat normal1 = x3D-Ow1;
            cv::Mat normal2 = x3D-Ow2;
            const float n1x = normal1.at<float>(0), n1y = normal1.at<float>(1), n1z = normal1.at<float>(2);
            const float n2x = normal2.at<float>(0), n2y = normal2.at<float>(1), n2z = normal2.at<float>(2);
            const float dist1Sq = n1x*n1x + n1y*n1y + n1z*n1z;
            const float dist2Sq = n2x*n2x + n2y*n2y + n2z*n2z;

            if(dist1Sq < 1e-12f || dist2Sq < 1e-12f)
                continue;

            // 只在需要时计算实际距离
            float dist1 = sqrt(dist1Sq);
            float dist2 = sqrt(dist2Sq);

            const float ratioDist = dist2/dist1;
            const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                continue;

            // 三角化成功
            MapPoint* pMP = new MapPoint(x3D,mpCurrentKeyFrame,mpMap);

            pMP->AddObservation(mpCurrentKeyFrame,idx1);            
            pMP->AddObservation(pKF2,idx2);

            mpCurrentKeyFrame->AddMapPoint(pMP,idx1);
            pKF2->AddMapPoint(pMP,idx2);

            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();

            mpMap->AddMapPoint(pMP);
            mlpRecentAddedMapPoints.push_back(pMP);

            nnew++;
        }
    }
}

void LocalMapping::SearchInNeighbors()
{
    // 检索邻近关键帧
    int nn = LOCAL_MAPPING_NEIGHBOR_KFS;
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
    vector<KeyFrame*> vpTargetKFs;
    for(vector<KeyFrame*>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
            continue;
        vpTargetKFs.push_back(pKFi);
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;

        // 扩展到一些二级邻居
        const vector<KeyFrame*> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(LOCAL_MAPPING_SECOND_NEIGHBOR_KFS);
        for(vector<KeyFrame*>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
        {
            KeyFrame* pKFi2 = *vit2;
            if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->mnId || pKFi2->mnId==mpCurrentKeyFrame->mnId)
                continue;
            vpTargetKFs.push_back(pKFi2);
            pKFi2->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;
        }
    }

    // 通过从当前关键帧投影到目标关键帧来搜索匹配
    ORBmatcher matcher;
    vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(vector<KeyFrame*>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;

        matcher.Fuse(pKFi,vpMapPointMatches);
    }

    // 通过从目标关键帧投影到当前关键帧来搜索匹配
    vector<MapPoint*> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

    for(vector<KeyFrame*>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
    {
        KeyFrame* pKFi = *vitKF;

        vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();

        for(vector<MapPoint*>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
        {
            MapPoint* pMP = *vitMP;
            if(!pMP)
                continue;
            if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
                continue;
            pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
            vpFuseCandidates.push_back(pMP);
        }
    }

    matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates);

    // 更新点
    vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        MapPoint* pMP=vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                pMP->ComputeDistinctiveDescriptors();
                pMP->UpdateNormalAndDepth();
            }
        }
    }

    // 更新共视步图中的连接
    mpCurrentKeyFrame->UpdateConnections();
}

cv::Mat LocalMapping::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
{
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    cv::Mat t12x = SkewSymmetricMatrix(t12);

    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;

    return K1.t().inv()*t12x*R12*K2.inv();
}

void LocalMapping::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested.store(true);
    mbAbortBA.store(true);
    mCvEvent.notify_one();
}

void LocalMapping::CancelStopRequest()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested.store(false);
    mbStopped.store(false);
}

bool LocalMapping::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested.load() && !mbNotStop.load())
    {
        mbStopped.store(true);
        // 通知 WaitForStopped 的调用方（LoopClosing）状态已变化
        mCvEvent.notify_one();
        return true;
    }

    return false;
}

bool LocalMapping::isStopped()
{
    return mbStopped.load();
}

void LocalMapping::WaitForStopped(int timeoutMs)
{
    if (isStopped()) return;
    std::unique_lock<std::mutex> lock(mMutexEvent);
    mCvEvent.wait_for(lock, std::chrono::milliseconds(timeoutMs));
}

bool LocalMapping::stopRequested()
{
    return mbStopRequested.load();
}

void LocalMapping::Release()
{
    // 锁顺序 mMutexStop → mMutexFinish: 与 Stopped 循环的
    // isStopped(mMutexStop) → CheckFinish(mMutexFinish) 保持一致
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if(mbFinished)
        return;
    mbStopped.store(false);
    mbStopRequested.store(false);
    list<KeyFrame*> lKFs;
    {
        unique_lock<mutex> lock3(mMutexNewKFs);
        lKFs.swap(mlNewKeyFrames);
    }
    for(list<KeyFrame*>::iterator lit = lKFs.begin(), lend=lKFs.end(); lit!=lend; lit++)
    {
        if(*lit)
            (*lit)->SetBadFlag();
    }
    // 通知 Stopped 循环退出等待（mCvEvent.wait 在 mMutexStop 下阻塞时，
    // notify 在 mMutexStop/mMutexFinish 释放后发送，确保数据可见性）
    mCvEvent.notify_one();
}

bool LocalMapping::AcceptKeyFrames()
{
    if(mbAcceptKeyFrames.load())
        return true;

    // 即使建图线程正忙，如果队列中积压的关键帧较少（少于3帧），也允许继续插入，以极大地提升跟踪稳定性，避免运动卡顿
    unique_lock<mutex> lockQueue(mMutexNewKFs);
    return mlNewKeyFrames.size() < LOCAL_MAPPING_MAX_QUEUED_KFS;
}

void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    mbAcceptKeyFrames.store(flag);
}

bool LocalMapping::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped.load())
        return false;

    mbNotStop.store(flag);

    return true;
}

void LocalMapping::InterruptBA()
{
    mbAbortBA.store(true);
}

void LocalMapping::KeyFrameCulling()
{
    // 检查冗余关键帧：超过 REDUNDANCY_THRESHOLD 的地图点被≥3个其他KF观测则视为冗余
    vector<KeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    // 每次最多处理 KEYFRAME_CULLING_MAX_KFS 个关键帧，防止单次耗时过久阻塞跟踪线程
    const int KEYFRAME_CULLING_MAX_KFS = KEYFRAME_CULL_BATCH_SIZE;
    int nProcessed = 0;

    // 第一阶段：无主锁下的只读候选冗余帧搜集 (Read-Only Pass)
    vector<KeyFrame*> vpRedundantKFs;
    vpRedundantKFs.reserve(KEYFRAME_CULLING_MAX_KFS);

    for(vector<KeyFrame*>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        if(++nProcessed > KEYFRAME_CULLING_MAX_KFS)
            break;

        // 每次循环检查是否被中断（Reset/Stop请求），防止长时间阻塞
        if(mbAbortBA)
            break;

        KeyFrame* pKF = *vit;
        if(!pKF || pKF->isBad() || pKF->mnId==0)
            continue;
        const vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

        int nObs = KEYFRAME_REDUNDANCY_OBS_THRESHOLD;
        const int thObs=nObs;

        // 首先统计有效的 MapPoints 数量，以便在检测到足够多的非冗余观测后提前退出
        int nMPs = 0;
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP && !pMP->isBad())
                nMPs++;
        }

        if(nMPs == 0) continue;

        // 非冗余观测的最大允许数量。一旦非冗余观测数超过此上限，该帧绝无可能满足冗余标准
        const int maxNonRedundant = nMPs * (1.0f - KEYFRAME_REDUNDANCY_THRESHOLD);

        int nRedundantObservations=0;
        int nNonRedundantObservations=0;
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP && !pMP->isBad())
            {
                bool bRedundant = false;
                if(pMP->Observations()>thObs)
                {
                    const int &scaleLevel = pKF->mvKeysUn[i].octave;
                    int nObs = pMP->GetRedundantObservationsCount(pKF, scaleLevel);
                    if(nObs>=thObs)
                    {
                        nRedundantObservations++;
                        bRedundant = true;
                    }
                }

                if(!bRedundant)
                {
                    nNonRedundantObservations++;
                    // 如果非冗余点数已超限，则可断定该关键帧非冗余，提前剪枝，避免后续大量锁开销
                    if(nNonRedundantObservations > maxNonRedundant)
                    {
                        break;
                    }
                }
            }
        }

        if(nRedundantObservations > KEYFRAME_REDUNDANCY_THRESHOLD*nMPs)
        {
            vpRedundantKFs.push_back(pKF);
        }
    }

    // 第二阶段：每次只删除 1 个冗余关键帧（其余留待下一轮 KeyFrameCulling）。
    // SetBadFlag 会在锁外级联 EraseObservation → ComputeDistinctiveDescriptors（O(n²)），
    // 一次删除多个 KF 会与 Tracking 争抢数千次 MapPoint 锁，造成单次数秒冻结；
    // 改为每轮删 1 个，把开销摊薄到多轮，不改变算法语义、不降精度。
    for(size_t i=0; i<vpRedundantKFs.size(); i++)
    {
        KeyFrame* pKF = vpRedundantKFs[i];
        if(pKF && !pKF->isBad())
        {
            pKF->SetBadFlag();
            break;  // 每轮只删 1 个冗余 KF，摊薄 SetBadFlag 级联开销
        }
    }
}

cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
}

void LocalMapping::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
        mbAbortBA.store(true); // 立即中断正在进行的BA，确保Reset能被快速处理
    }
    mCvEvent.notify_one();
}

void LocalMapping::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlNewKeyFrames.clear();
        mlpRecentAddedMapPoints.clear();
        mbResetRequested=false;
        {
            unique_lock<mutex> completeLock(mMutexResetComplete);
            mbResetComplete = true;
        }
        mCvResetComplete.notify_one();
    }
}

void LocalMapping::WaitForResetComplete()
{
    unique_lock<mutex> lock(mMutexResetComplete);
    mCvResetComplete.wait(lock, [this]{ return mbResetComplete; });
    mbResetComplete = false;
}

void LocalMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
    mCvEvent.notify_one();
}

bool LocalMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LocalMapping::SetFinish()
{
    // 锁顺序 mMutexStop → mMutexFinish：与 Release() 一致
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    mbStopped = true;
    mbFinished = true;
}

bool LocalMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void LocalMapping::CheckLimits()
{
    // 1. 关键帧限制检查
    long unsigned int nKFs = mpMap->KeyFramesInMap();
    if(nKFs > MAX_KEYFRAMES)
    {
        // 获取所有关键帧
        vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

        // 按 ID 排序（最旧的在前）
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        int nToErase = nKFs - MAX_KEYFRAMES + KEYFRAME_CULL_BATCH_SIZE;
        int nErased = 0;

        // 获取局部关键帧（当前的邻居）以保护它们
        set<KeyFrame*> spLocalKFs;
        vector<KeyFrame*> vpLocalKFs = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();
        for(size_t i=0; i<vpLocalKFs.size(); i++) spLocalKFs.insert(vpLocalKFs[i]);
        spLocalKFs.insert(mpCurrentKeyFrame);

        // 遍历并移除安全候选者
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];

            // 保护规则：
            if(pKF->mnId == 0) continue; // 不要删除第一个关键帧（原点）
            if(spLocalKFs.count(pKF)) continue; // 不要删除局部关键帧（跟踪需要）

            // 标记为 bad（这将触发从地图中删除并清理观测）
            pKF->SetBadFlag();
            nErased++;

            if(nErased >= nToErase) break;
        }
    }

    // 2. 地图点限制检查（统一管理）
    long unsigned int nMPs = mpMap->MapPointsInMap();
    if(nMPs > MAX_MAPPOINTS)
    {
        vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();

        // 使用 nth_element 代替 sort，将最旧的 nToEraseMP 个点放到前面
        int nToEraseMP = nMPs - MAX_MAPPOINTS + MAPPOINT_CULL_BATCH_SIZE;
        if(nToEraseMP > (int)vpMPs.size()) nToEraseMP = vpMPs.size();

        std::nth_element(vpMPs.begin(), vpMPs.begin() + nToEraseMP, vpMPs.end(), [](MapPoint* a, MapPoint* b){
            return a->mnId < b->mnId;
        });
        int nErasedMP = 0;

        // 获取当前帧观测到的地图点以保护它们
        set<MapPoint*> spLocalMPs;
        if(mpCurrentKeyFrame) {
             vector<MapPoint*> currentMPs = mpCurrentKeyFrame->GetMapPointMatches();
             for(auto mp : currentMPs) if(mp) spLocalMPs.insert(mp);
        }

        for(size_t i=0; i<vpMPs.size(); i++)
        {
             MapPoint* pMP = vpMPs[i];
             if(!pMP || pMP->isBad()) continue;
             if(pMP->mbFromLoadedMap) continue; // 保护加载的地图点（绿点）

             // 保护当前关键帧看到的点
             if(spLocalMPs.count(pMP)) continue;

             // 保护最近看到的点（在最近 N 帧内）
             if(pMP->mnLastFrameSeen >= mpCurrentKeyFrame->mnFrameId - LOCAL_MAPPING_CULL_PROTECT_FRAMES) continue; 

             pMP->SetBadFlag();
             nErasedMP++;

             if(nErasedMP >= nToEraseMP) break;
        }
    }
}

} //namespace ORB_SLAM2

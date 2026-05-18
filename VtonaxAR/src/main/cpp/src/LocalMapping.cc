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

#include<mutex>
#include<algorithm>
#include "VtonaxProfiler.h" // 性能分析器

namespace ORB_SLAM2
{

LocalMapping::LocalMapping(Map *pMap):
    mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true)
{
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
            // BoW 转换并插入地图
            ProcessNewKeyFrame();

            // 检查最近的地图点
            MapPointCulling();

            // 三角化新的地图点
            CreateNewMapPoints();

            if(!CheckNewKeyFrames())
            {
                // 在邻近关键帧中寻找更多匹配并融合重复点
                SearchInNeighbors();
            }

            mbAbortBA = false;

            if(!CheckNewKeyFrames() && !stopRequested())
            {
                // 局部 BA
                if(mpMap->KeyFramesInMap()>2)
                    Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame,&mbAbortBA, mpMap);

                // 检查冗余的局部关键帧
                KeyFrameCulling();
                
                // 检查地图限制（统一管理）
                CheckLimits();
            }

            mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
        }
        else if(Stop())
        {
            // 安全停止区域
            while(isStopped() && !CheckFinish())
            {
                usleep(3000);
            }
            if(CheckFinish())
                break;
        }

        ResetIfRequested();

        // 跟踪线程将看到局部建图线程正忙
        SetAcceptKeyFrames(true);

        if(CheckFinish())
            break;

        usleep(3000);
    }

    SetFinish();
}

void LocalMapping::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    mbAbortBA=true;
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

    // 计算词袋结构
    mpCurrentKeyFrame->ComputeBoW();

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
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=2 && pMP->Observations()<=cnThObs)
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=3)
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

    ORBmatcher matcher(0.6,false);

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

        // if(!mbMonocular)
        // {
        //     if(baseline<pKF2->mb)
        //     continue;
        // }
        // else
        {
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
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

            // 检查光线之间的视差
            cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0);
            cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0);

            cv::Mat ray1 = Rwc1*xn1;
            cv::Mat ray2 = Rwc2*xn2;
            // 内联计算向量范数平方，避免两次cv::norm调用
            const float r1x = ray1.at<float>(0), r1y = ray1.at<float>(1), r1z = ray1.at<float>(2);
            const float r2x = ray2.at<float>(0), r2y = ray2.at<float>(1), r2z = ray2.at<float>(2);
            const float dotProduct = r1x*r2x + r1y*r2y + r1z*r2z;
            const float norm1Sq = r1x*r1x + r1y*r1y + r1z*r1z;
            const float norm2Sq = r2x*r2x + r2y*r2y + r2z*r2z;
            const float cosParallaxRays = dotProduct / sqrt(norm1Sq * norm2Sq);

            float cosParallaxStereo = cosParallaxRays+1;
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;

            // 单目模式下跳过双目视差计算
            // if(bStereo1)
            //     cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));
            // else if(bStereo2)
            //     cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));

            cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);

            cv::Mat x3D;
            if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<LOCAL_MAPPING_TRIANGULATION_PARALLAX_TH))
            {
                // 线性三角化方法
                cv::Mat A(4,4,CV_32F);
                A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
                A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
                A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
                A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);

                cv::Mat w,u,vt;
                cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

                x3D = vt.row(3).t();

                if(x3D.at<float>(3)==0)
                    continue;

                // 欧几里得坐标
                x3D = x3D.rowRange(0,3)/x3D.at<float>(3);

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

            // 检查第一个关键帧中的重投影误差
            const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
            const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
            const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
            const float invz1 = 1.0/z1;

            {
                float u1 = fx1*x1*invz1+cx1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                if((errX1*errX1+errY1*errY1)>OPTIMIZER_CHI2_TH_2D*sigmaSquare1)
                    continue;
            }

            // 检查第二个关键帧中的重投影误差
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
            const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
            const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
            const float invz2 = 1.0/z2;
            {
                float u2 = fx2*x2*invz2+cx2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                if((errX2*errX2+errY2*errY2)>OPTIMIZER_CHI2_TH_2D*sigmaSquare2)
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

            /*if(fabs(ratioDist-ratioOctave)>ratioFactor)
                continue;*/
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
    mbStopRequested = true;
    unique_lock<mutex> lock2(mMutexNewKFs);
    mbAbortBA = true;
}

void LocalMapping::CancelStopRequest()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = false;
}

bool LocalMapping::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        return true;
    }

    return false;
}

bool LocalMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool LocalMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

void LocalMapping::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if(mbFinished)
        return;
    mbStopped = false;
    mbStopRequested = false;
    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
        delete *lit;
    mlNewKeyFrames.clear();
}

bool LocalMapping::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

bool LocalMapping::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped)
        return false;

    mbNotStop = flag;

    return true;
}

void LocalMapping::InterruptBA()
{
    mbAbortBA = true;
}

void LocalMapping::KeyFrameCulling()
{
    // 检查冗余关键帧（仅限局部关键帧）
    // 如果一个关键帧观测到的90%的地图点至少被其他3个关键帧（在相同或更精细的尺度上）观测到，则认为该关键帧是冗余的
    // 我们只考虑近距离的双目点
    vector<KeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    for(vector<KeyFrame*>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        KeyFrame* pKF = *vit;
        if(pKF->mnId==0)
            continue;
        const vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

        int nObs = KEYFRAME_REDUNDANCY_OBS_THRESHOLD;
        const int thObs=nObs;
        int nRedundantObservations=0;
        int nMPs=0;
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    // 单目模式下跳过深度检查
                    // if(!mbMonocular)
                    // {
                    //     if(pKF->mvDepth[i]>pKF->mThDepth || pKF->mvDepth[i]<0)
                    //         continue;
                    // }

                    nMPs++;
                    if(pMP->Observations()>thObs)
                    {
                        const int &scaleLevel = pKF->mvKeysUn[i].octave;
                        const std::map<KeyFrame*, size_t> observations = pMP->GetObservations();
                        int nObs=0;
                        for(std::map<KeyFrame*, size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            KeyFrame* pKFi = mit->first;
                            if(pKFi==pKF)
                                continue;
                            const int &scaleLeveli = pKFi->mvKeysUn[mit->second].octave;

                            if(scaleLeveli<=scaleLevel+1)
                            {
                                nObs++;
                                if(nObs>=thObs)
                                    break;
                            }
                        }
                        if(nObs>=thObs)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }  

        if(nRedundantObservations>KEYFRAME_REDUNDANCY_THRESHOLD*nMPs)
            pKF->SetBadFlag();
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
        mbAbortBA = true; // 立即中断正在进行的BA，确保Reset能被快速处理
    }

    // 移除阻塞的自旋锁，让主线程立刻返回
    // while(1)
    // {
    //     {
    //         unique_lock<mutex> lock2(mMutexReset);
    //         if(!mbResetRequested)
    //             break;
    //     }
    //     usleep(3000);
    // }
}

void LocalMapping::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlNewKeyFrames.clear();
        mlpRecentAddedMapPoints.clear();
        mbResetRequested=false;
    }
}

void LocalMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LocalMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LocalMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
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

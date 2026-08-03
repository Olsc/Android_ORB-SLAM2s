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

#include "MapPoint.h"
#include "ORBmatcher.h"
#include "Config.h"

#include<mutex>

namespace ORB_SLAM2
{

long unsigned int MapPoint::nNextId=0;

MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map* pMap):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap), mnMapId(-1)
{
    Pos.copyTo(mWorldPos);
    mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    // MapPoints 可以从 Tracking 和 Local Mapping 创建。此互斥锁避免 ID 冲突。
    std::unique_lock<std::mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

MapPoint::MapPoint(const cv::Mat &Pos, Map* pMap, Frame* pFrame, const int &idxF):
    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
    mnBALocalForKF(0), mnFuseCandidateForKF(0),mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1),
    mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    cv::Mat Ow = pFrame->GetCameraCenter();
    cv::Mat PC = Pos - Ow;

    // 内联计算距离和法向量归一化，避免多次cv::norm调用
    const float pcx = PC.at<float>(0);
    const float pcy = PC.at<float>(1);
    const float pcz = PC.at<float>(2);
    const float distSq = pcx*pcx + pcy*pcy + pcz*pcz;
    const float dist = sqrt(distSq);
    const float invDist = (dist > 1e-10f) ? (1.0f / dist) : 0.0f;

    mNormalVector = (cv::Mat_<float>(3,1) << pcx * invDist, pcy * invDist, pcz * invDist);

    const int level = pFrame->mvKeysUn[idxF].octave;
    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    mfMaxDistance = dist*levelScaleFactor;
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

    // MapPoints 可以从 Tracking 和 Local Mapping 创建。此互斥锁避免 ID 冲突。
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

MapPoint::MapPoint(const cv::Mat &Pos, Map* pMap):
    mnFirstKFid(-1), mnFirstFrame(-1), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
    mnBALocalForKF(0), mnFuseCandidateForKF(0),mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1),
    mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap), mnMapId(-1)
{
    Pos.copyTo(mWorldPos);
    // 初始化一个默认法向，避免视锥角度检查失败
    mNormalVector = (cv::Mat_<float>(3,1) << 0,0,1);
    // 给予宽松的尺度范围，保证视野检查可通过
    mfMinDistance = MAPPOINT_DEFAULT_MIN_DIST;
    mfMaxDistance = MAPPOINT_DEFAULT_MAX_DIST;

    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

void MapPoint::SetWorldPos(const cv::Mat &Pos)
{
    std::unique_lock<std::mutex> lock(mMutexPos);
    Pos.copyTo(mWorldPos);
}

cv::Mat MapPoint::GetWorldPos()
{
    std::unique_lock<std::mutex> lock(mMutexPos);
    return mWorldPos.clone();
}

void MapPoint::GetWorldPos(cv::Point3f& out)
{
    std::unique_lock<std::mutex> lock(mMutexPos);
    out.x = mWorldPos.at<float>(0);
    out.y = mWorldPos.at<float>(1);
    out.z = mWorldPos.at<float>(2);
}

cv::Mat MapPoint::GetNormal()
{
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector.clone();
}

void MapPoint::GetNormal(cv::Point3f& out)
{
    std::unique_lock<std::mutex> lock(mMutexPos);
    out.x = mNormalVector.at<float>(0);
    out.y = mNormalVector.at<float>(1);
    out.z = mNormalVector.at<float>(2);
}

KeyFrame* MapPoint::GetReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mpRefKF;
}

void MapPoint::AddObservation(KeyFrame* pKF, size_t idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return;
    mObservations[pKF]=idx;

        nObs++;

    // 如果还没有参考关键帧，使用首次观测的关键帧作为参考
    if(mpRefKF==nullptr) mpRefKF = pKF;
}

void MapPoint::EraseObservation(KeyFrame* pKF)
{
    bool bBad=false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
        {
            int idx = mObservations[pKF];
                nObs--;

            mObservations.erase(pKF);

            if(mpRefKF==pKF)
                mpRefKF=mObservations.begin()->first;

            // 如果只有2个或更少的观测，则丢弃该点
            if(nObs<=MAPPOINT_MIN_OBS_FOR_BAD)
                bBad=true;
        }
    }

    if(bBad)
        SetBadFlag();
}

map<KeyFrame*, size_t> MapPoint::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

void MapPoint::ShareObservations(std::map<KeyFrame*, int>& counter, unsigned long excludeId)
{
    unique_lock<mutex> lock(mMutexFeatures);
    for(std::map<KeyFrame*, size_t>::const_iterator mit=mObservations.begin(), mend=mObservations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        if(pKF->mnId == excludeId || pKF->isBad())
            continue;
        counter[pKF]++;
    }
}

int MapPoint::GetRedundantObservationsCount(KeyFrame* pKF, int scaleLevel)
{
    unique_lock<mutex> lock(mMutexFeatures);
    int count = 0;
    for(std::map<KeyFrame*, size_t>::const_iterator mit=mObservations.begin(), mend=mObservations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKFi = mit->first;
        if(pKFi==pKF || pKFi->isBad())
            continue;
        if(mit->second >= pKFi->mvKeysUn.size())
            continue;
        const int &scaleLeveli = pKFi->mvKeysUn[mit->second].octave;
        if(scaleLeveli<=scaleLevel+1)
        {
            count++;
        }
    }
    return count;
}

int MapPoint::Observations() const
{
    // nObs 已改为原子变量，无需加锁，零锁开销
    return nObs.load(std::memory_order_relaxed);
}

void MapPoint::SetBadFlag()
{
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;
        obs = mObservations;
        mObservations.clear();
    }
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        pKF->EraseMapPointMatch(mit->second);
    }

    mpMap->EraseMapPoint(this);
}

MapPoint* MapPoint::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}

void MapPoint::Replace(MapPoint* pMP)
{
    if(pMP->mnId==this->mnId)
        return;

    int nvisible, nfound;
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP;
    }

    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // 替换关键帧中的测量值
        KeyFrame* pKF = mit->first;

        if(!pMP->IsInKeyFrame(pKF))
        {
            pKF->ReplaceMapPointMatch(mit->second, pMP);
            pMP->AddObservation(pKF,mit->second);
        }
        else
        {
            pKF->EraseMapPointMatch(mit->second);
        }
    }
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();

    mpMap->EraseMapPoint(this);
}

bool MapPoint::isBad()
{
    // mbBad已改为原子变量，无需加锁，极大减少高频调用的锁开销，使用 memory_order_relaxed 以获得最佳性能
    return mbBad.load(std::memory_order_relaxed);
}

void MapPoint::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;
}

void MapPoint::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;
}

float MapPoint::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}

void MapPoint::ComputeDistinctiveDescriptors()
{
    // 检索所有观测到的描述符
    vector<cv::Mat> vDescriptors;

    map<KeyFrame*,size_t> observations;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if(mbBad)
            return;
        observations=mObservations;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        if(!pKF->isBad())
            vDescriptors.push_back(pKF->mDescriptors.row(mit->second));
    }

    if(vDescriptors.empty())
        return;

    const size_t N = vDescriptors.size();

    // 快速路径：1 个或 2 个观测时无需计算距离矩阵
    if (N <= 2)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        mDescriptor = vDescriptors[0].clone();
        return;
    }

    // N > 2 时，使用栈内存缓冲区（限制最大 64 个观测）
    const size_t N_max = std::min(N, (size_t)64);
    int distsMat[64][64];

    for(size_t i = 0; i < N_max; ++i)
    {
        distsMat[i][i] = 0;
        for(size_t j = i + 1; j < N_max; ++j)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i], vDescriptors[j]);
            distsMat[i][j] = distij;
            distsMat[j][i] = distij;
        }
    }

    int BestMedian = INT_MAX;
    int BestIdx = 0;
    int rowDists[64];

    for(size_t i = 0; i < N_max; ++i)
    {
        std::memcpy(rowDists, distsMat[i], N_max * sizeof(int));
        size_t medianIdx = (N_max - 1) / 2;
        std::nth_element(rowDists, rowDists + medianIdx, rowDists + N_max);
        int median = rowDists[medianIdx];

        if(median < BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures);
        mDescriptor = vDescriptors[BestIdx].clone();
    }
}

cv::Mat MapPoint::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

void MapPoint::UpdateNormalAndDepth()
{
    map<KeyFrame*,size_t> observations;
    KeyFrame* pRefKF;
    cv::Mat Pos;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if(mbBad)
            return;
        observations=mObservations;
        pRefKF=mpRefKF;
        Pos = mWorldPos.clone();
    }

    if(observations.empty())
        return;

    // 兼容加载点：若参考关键帧为空或无效，则选取任一可用观测关键帧
    if(!pRefKF || pRefKF->isBad())
    {
        auto it = observations.begin();
        if(it==observations.end()) return;
        pRefKF = it->first;
        if(!pRefKF) return;
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            mpRefKF = pRefKF;
        }
    }

    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;
    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        cv::Mat Owi = pKF->GetCameraCenter();
        cv::Mat normali = Pos - Owi;

        // 内联计算向量范数，避免cv::norm调用
        const float nx = normali.at<float>(0);
        const float ny = normali.at<float>(1);
        const float nz = normali.at<float>(2);
        const float normSq = nx*nx + ny*ny + nz*nz;
        if(normSq > 1e-12f) {
            const float invNorm = 1.0f / sqrt(normSq);
            normal.at<float>(0) += nx * invNorm;
            normal.at<float>(1) += ny * invNorm;
            normal.at<float>(2) += nz * invNorm;
        }
        n++;
    }

    cv::Mat PC = Pos - pRefKF->GetCameraCenter();

    // 内联计算距离，避免cv::norm调用
    const float pcx = PC.at<float>(0);
    const float pcy = PC.at<float>(1);
    const float pcz = PC.at<float>(2);
    const float dist = sqrt(pcx*pcx + pcy*pcy + pcz*pcz);

    const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels;

    {
        unique_lock<mutex> lock3(mMutexPos);
        mfMaxDistance = dist*levelScaleFactor;
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
        mNormalVector = normal/n;
    }
}

float MapPoint::GetMinDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return MAPPOINT_MIN_DIST_INVARIANCE_FACTOR*mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return MAPPOINT_MAX_DIST_INVARIANCE_FACTOR*mfMaxDistance;
}

int MapPoint::PredictScale(const float &currentDist, KeyFrame* pKF)
{
    float maxDistance;
    {
        unique_lock<mutex> lock(mMutexPos);
        maxDistance = mfMaxDistance;
    }

    int nScale = 0;
    const std::vector<float>& mvScaleFactors = pKF->mvScaleFactors;
    const int nLevels = pKF->mnScaleLevels;
    for (; nScale < nLevels; ++nScale)
    {
        if (maxDistance <= currentDist * mvScaleFactors[nScale])
            break;
    }
    if (nScale >= nLevels)
        nScale = nLevels - 1;

    return nScale;
}

int MapPoint::PredictScale(const float &currentDist, Frame* pF)
{
    float maxDistance;
    {
        unique_lock<mutex> lock(mMutexPos);
        maxDistance = mfMaxDistance;
    }

    int nScale = 0;
    const std::vector<float>& mvScaleFactors = pF->mvScaleFactors;
    const int nLevels = pF->mnScaleLevels;
    for (; nScale < nLevels; ++nScale)
    {
        if (maxDistance <= currentDist * mvScaleFactors[nScale])
            break;
    }
    if (nScale >= nLevels)
        nScale = nLevels - 1;

    return nScale;
}

void MapPoint::SetMapId(int id)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnMapId = id;
}

int MapPoint::GetMapId()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mnMapId;
}

} //namespace ORB_SLAM2

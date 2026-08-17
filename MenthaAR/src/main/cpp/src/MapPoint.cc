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
    // 栈版读取相机中心（Frame 无锁），PC = Pos - Ow 标量化
    cv::Point3f Ow;
    pFrame->GetCameraCenter(Ow);

    // 内联计算距离和法向量归一化，避免多次cv::norm调用
    const float pcx = Pos.at<float>(0) - Ow.x;
    const float pcy = Pos.at<float>(1) - Ow.y;
    const float pcz = Pos.at<float>(2) - Ow.z;
    const float distSq = pcx*pcx + pcy*pcy + pcz*pcz;
    const float dist = sqrt(distSq);
    const float invDist = (dist > 1e-10f) ? (1.0f / dist) : 0.0f;

    mNormalVector = (cv::Mat_<float>(3,1) << pcx * invDist, pcy * invDist, pcz * invDist);

    const int level = pFrame->mvKeysUn[idxF].octave;
    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    mfMaxDistance = dist*levelScaleFactor;
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

    // 描述子改为 shared_ptr + 原子读
    std::atomic_store(&mDescriptor, std::make_shared<const cv::Mat>(pFrame->mDescriptors.row(idxF).clone()));
    mfMinDistInvariance.store(MAPPOINT_MIN_DIST_INVARIANCE_FACTOR*mfMinDistance, std::memory_order_relaxed);
    mfMaxDistInvariance.store(MAPPOINT_MAX_DIST_INVARIANCE_FACTOR*mfMaxDistance, std::memory_order_relaxed);

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
    mfMinDistInvariance.store(MAPPOINT_MIN_DIST_INVARIANCE_FACTOR*mfMinDistance, std::memory_order_relaxed);
    mfMaxDistInvariance.store(MAPPOINT_MAX_DIST_INVARIANCE_FACTOR*mfMaxDistance, std::memory_order_relaxed);

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
                mpRefKF = mObservations.empty() ? static_cast<KeyFrame*>(NULL) : mObservations.begin()->first;

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

void MapPoint::ShareObservations(std::unordered_map<KeyFrame*, int>& counter, unsigned long excludeId)
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
    if (nObs.load(std::memory_order_relaxed) < 3)
        return 0;

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
        if(scaleLeveli<=scaleLevel+MAPPOINT_SCALE_LEVEL_TOL)
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
    // 目标点已坏则不合并（否则观测会被并入死点丢失）
    if(pMP->isBad())
        return;

    int nvisible, nfound;
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        nvisible = mnVisible.load(std::memory_order_relaxed);
        nfound = mnFound.load(std::memory_order_relaxed);
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
    // 原子计数（热路径每帧数百次调用）
    mnVisible.fetch_add(n, std::memory_order_relaxed);
}

void MapPoint::IncreaseFound(int n)
{
    mnFound.fetch_add(n, std::memory_order_relaxed);
}

float MapPoint::GetFoundRatio()
{
    // mnVisible 至少为 1（构造函数初始化），无需除零保护
    return static_cast<float>(mnFound.load(std::memory_order_relaxed)) /
           mnVisible.load(std::memory_order_relaxed);
}

void MapPoint::ComputeDistinctiveDescriptors()
{
    // 检索所有观测到的描述符
    map<KeyFrame*,size_t> observations;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if(mbBad)
            return;
        observations=mObservations;
    }

    if(observations.empty())
        return;

    const uint8_t* descPtrs[MAPPOINT_DESC_MAX_OBS];
    cv::Mat descMats[MAPPOINT_DESC_MAX_OBS];
    size_t nDescs = 0;

    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        if(!pKF->isBad())
        {
            if (nDescs < MAPPOINT_DESC_MAX_OBS)
            {
                descPtrs[nDescs] = pKF->mDescriptors.ptr<uint8_t>(mit->second);
                descMats[nDescs] = pKF->mDescriptors.row(mit->second);
                nDescs++;
            }
        }
    }

    if(nDescs == 0)
        return;

    const size_t N = nDescs;

    // 快速路径：1 个或 2 个观测时无需计算距离矩阵
    if (N <= 2)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        std::atomic_store(&mDescriptor, std::make_shared<const cv::Mat>(descMats[0].clone()));
        return;
    }

    int distsMat[MAPPOINT_DESC_MAX_OBS][MAPPOINT_DESC_MAX_OBS];

    for(size_t i = 0; i < N; ++i)
    {
        distsMat[i][i] = 0;
        for(size_t j = i + 1; j < N; ++j)
        {
            int distij = ORBmatcher::DescriptorDistance(descPtrs[i], descPtrs[j]);
            distsMat[i][j] = distij;
            distsMat[j][i] = distij;
        }
    }

    int BestMedian = INT_MAX;
    int BestIdx = 0;
    int rowDists[MAPPOINT_DESC_MAX_OBS];

    for(size_t i = 0; i < N; ++i)
    {
        std::memcpy(rowDists, distsMat[i], N * sizeof(int));
        size_t medianIdx = (N - 1) / 2;
        std::nth_element(rowDists, rowDists + medianIdx, rowDists + N);
        int median = rowDists[medianIdx];

        if(median < BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures);
        std::atomic_store(&mDescriptor, std::make_shared<const cv::Mat>(descMats[BestIdx].clone()));
    }
}

cv::Mat MapPoint::GetDescriptor()
{
    // 原子读取 shared_ptr 后 clone
    std::shared_ptr<const cv::Mat> d = std::atomic_load(&mDescriptor);
    if(d)
        return d->clone();
    return cv::Mat();
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
    // Pos 的标量副本，避免循环内 Mat 运算
    const float posx = Pos.at<float>(0);
    const float posy = Pos.at<float>(1);
    const float posz = Pos.at<float>(2);
    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        // 栈版读取相机中心
        cv::Point3f Owi;
        pKF->GetCameraCenter(Owi);

        // normali = Pos - Owi（标量）
        const float nx = posx - Owi.x;
        const float ny = posy - Owi.y;
        const float nz = posz - Owi.z;
        const float normSq = nx*nx + ny*ny + nz*nz;
        if(normSq > 1e-12f) {
            const float invNorm = 1.0f / sqrt(normSq);
            normal.at<float>(0) += nx * invNorm;
            normal.at<float>(1) += ny * invNorm;
            normal.at<float>(2) += nz * invNorm;
        }
        n++;
    }

    cv::Point3f pRefOwi;
    pRefKF->GetCameraCenter(pRefOwi);

    // PC = Pos - pRefKF 相机中心（标量）
    const float pcx = posx - pRefOwi.x;
    const float pcy = posy - pRefOwi.y;
    const float pcz = posz - pRefOwi.z;
    const float dist = sqrt(pcx*pcx + pcy*pcy + pcz*pcz);

    const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels;

    {
        unique_lock<mutex> lock3(mMutexPos);
        mfMaxDistance = dist*levelScaleFactor;
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
        mNormalVector = normal/n;
        // 同步预计算不变性边界
        mfMinDistInvariance.store(MAPPOINT_MIN_DIST_INVARIANCE_FACTOR*mfMinDistance, std::memory_order_relaxed);
        mfMaxDistInvariance.store(MAPPOINT_MAX_DIST_INVARIANCE_FACTOR*mfMaxDistance, std::memory_order_relaxed);
    }
}

float MapPoint::GetMinDistanceInvariance()
{
    // 原子缓存
    return mfMinDistInvariance.load(std::memory_order_relaxed);
}

float MapPoint::GetMaxDistanceInvariance()
{
    // 原子缓存
    return mfMaxDistInvariance.load(std::memory_order_relaxed);
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
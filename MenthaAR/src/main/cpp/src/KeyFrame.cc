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

#include "KeyFrame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include "Config.h"
#include <mutex>
#include <algorithm>
#include "Common.h"

using namespace std;

namespace ORB_SLAM2
{

long unsigned int KeyFrame::nNextId=0;

KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB):
    mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
    fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
    mbf(F.mbf), mb(F.mb), N(F.N), mvKeys(F.mvKeys),    mvKeysUn(F.mvKeysUn), mDescriptors(F.mDescriptors.clone()),
    mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
    mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
    mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
    mnMaxY(F.mnMaxY), mK(F.mK.clone()), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
    mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
    mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb/2), mpMap(pMap), mpTree(F.mpTree)
{
    mnId=nNextId++;

    mGrid.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++)
            mGrid[i][j] = F.mGrid[i][j];
    }

    SetPose(F.mTcw);    
}

void KeyFrame::SetPose(const cv::Mat &Tcw_)
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    
    // 验证输入位姿的有效性，防止崩溃
    if(Tcw_.empty() || Tcw_.rows < 4 || Tcw_.cols < 4){
        // 位姿无效时设为单位矩阵避免崩溃
        Tcw = cv::Mat::eye(4,4,CV_32F);
        // 继续后续处理，使用单位矩阵
    } else {
        Tcw_.copyTo(Tcw);
    }
    
    // 确保Tcw是有效的4x4矩阵
    if(Tcw.empty() || Tcw.rows < 4 || Tcw.cols < 4){
        LOGE("关键帧::设置位姿: 复制后Tcw仍然无效，使用单位矩阵");
        Tcw = cv::Mat::eye(4,4,CV_32F);
    }
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat Rwc = Rcw.t();
    Ow = -Rwc*tcw;

    Twc = cv::Mat::eye(4,4,Tcw.type());
    Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    Ow.copyTo(Twc.rowRange(0,3).col(3));

    Ow.copyTo(Twc.rowRange(0,3).col(3));
}

cv::Mat KeyFrame::GetPose()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return Tcw.clone();
}

cv::Mat KeyFrame::GetPoseInverse()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return Twc.clone();
}

cv::Mat KeyFrame::GetCameraCenter()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return Ow.clone();
}

void KeyFrame::GetCameraCenter(cv::Point3f& out)
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    out.x = Ow.at<float>(0);
    out.y = Ow.at<float>(1);
    out.z = Ow.at<float>(2);
}

cv::Mat KeyFrame::GetRotation()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).colRange(0,3).clone();
}

cv::Mat KeyFrame::GetTranslation()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).col(3).clone();
}

void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight)
{
    {
        std::unique_lock<std::mutex> lock(mMutexConnections);
        if(!mConnectedKeyFrameWeights.count(pKF))
            mConnectedKeyFrameWeights[pKF]=weight;
        else if(mConnectedKeyFrameWeights[pKF]!=weight)
            mConnectedKeyFrameWeights[pKF]=weight;
        else
            return;
    }

    UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles()
{
    std::unique_lock<std::mutex> lock(mMutexConnections);
    std::vector<std::pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());
    for(std::map<KeyFrame*,int>::const_iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; ++mit)
       vPairs.push_back(std::make_pair(mit->second,mit->first));

    std::sort(vPairs.rbegin(), vPairs.rend());

    mvpOrderedConnectedKeyFrames.resize(vPairs.size());
    mvOrderedWeights.resize(vPairs.size());
    for(size_t i=0, iend=vPairs.size(); i<iend; ++i)
    {
        mvpOrderedConnectedKeyFrames[i] = vPairs[i].second;
        mvOrderedWeights[i] = vPairs[i].first;
    }
}

std::set<KeyFrame*> KeyFrame::GetConnectedKeyFrames()
{
    std::unique_lock<std::mutex> lock(mMutexConnections);
    std::set<KeyFrame*> s;
    for(std::map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
        s.insert(mit->first);
    return s;
}

std::vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames()
{
    std::unique_lock<std::mutex> lock(mMutexConnections);
    return mvpOrderedConnectedKeyFrames;
}

std::vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
    std::unique_lock<std::mutex> lock(mMutexConnections);
    if((int)mvpOrderedConnectedKeyFrames.size()<N)
        return mvpOrderedConnectedKeyFrames;
    else
        return std::vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);

}

vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight(const int &w)
{
    unique_lock<mutex> lock(mMutexConnections);

    if(mvpOrderedConnectedKeyFrames.empty())
        return vector<KeyFrame*>();

    vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,KeyFrame::weightComp);
    if(it==mvOrderedWeights.end())
        return vector<KeyFrame*>();
    else
    {
        int n = it-mvOrderedWeights.begin();
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);
    }
}

int KeyFrame::GetWeight(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexConnections);
    if(mConnectedKeyFrameWeights.count(pKF))
        return mConnectedKeyFrameWeights[pKF];
    else
        return 0;
}

void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(idx < mvpMapPoints.size())
        mvpMapPoints[idx]=pMP;
}

void KeyFrame::EraseMapPointMatch(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(idx < mvpMapPoints.size())
        mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
}

void KeyFrame::EraseMapPointMatch(MapPoint* pMP)
{
    int idx = pMP->GetIndexInKeyFrame(this);
    if(idx>=0 && static_cast<size_t>(idx)<mvpMapPoints.size())
        mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
}

void KeyFrame::ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP)
{
    if(idx < mvpMapPoints.size())
        mvpMapPoints[idx]=pMP;
}

set<MapPoint*> KeyFrame::GetMapPoints()
{
    unique_lock<mutex> lock(mMutexFeatures);
    set<MapPoint*> s;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        MapPoint* pMP = mvpMapPoints[i];
        if(!pMP->isBad())
            s.insert(pMP);
    }
    return s;
}

int KeyFrame::TrackedMapPoints(const int &minObs)
{
    unique_lock<mutex> lock(mMutexFeatures);

    int nPoints=0;
    const bool bCheckObs = minObs>0;
    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = mvpMapPoints[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(bCheckObs)
                {
                    if(mvpMapPoints[i]->Observations()>=minObs)
                        nPoints++;
                }
                else
                    nPoints++;
            }
        }
    }

    return nPoints;
}

vector<MapPoint*> KeyFrame::GetMapPointMatches()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints;
}

MapPoint* KeyFrame::GetMapPoint(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(idx >= mvpMapPoints.size())
        return nullptr;
    return mvpMapPoints[idx];
}

void KeyFrame::UpdateConnections()
{
    map<KeyFrame*,int> KFcounter;

    vector<MapPoint*> vpMP;

    {
        unique_lock<mutex> lockMPs(mMutexFeatures);
        vpMP = mvpMapPoints;
    }

    // 对于关键帧中的所有地图点，检查它们在哪些其他关键帧中被看到
    // 增加这些关键帧的计数器
    for(vector<MapPoint*>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;

        if(!pMP)
            continue;

        if(pMP->isBad())
            continue;

        pMP->ShareObservations(KFcounter, mnId);
    }

    // 这不应该发生
    if(KFcounter.empty())
        return;

    // 如果计数器大于阈值，则添加连接
    // 如果没有关键帧计数器超过阈值，则添加计数器最大的那个
    int nmax=0;
    KeyFrame* pKFmax=NULL;
    int th = KEYFRAME_CONNECTION_TH;

    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(KFcounter.size());
    for(map<KeyFrame*,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
    {
        if(mit->second>nmax)
        {
            nmax=mit->second;
            pKFmax=mit->first;
        }
        if(mit->second>=th)
        {
            vPairs.push_back(make_pair(mit->second,mit->first));
            (mit->first)->AddConnection(this,mit->second);
        }
    }

    if(vPairs.empty())
    {
        vPairs.push_back(make_pair(nmax,pKFmax));
        pKFmax->AddConnection(this,nmax);
    }

    std::sort(vPairs.rbegin(), vPairs.rend());

    vector<KeyFrame*> vOrderedKFs(vPairs.size());
    vector<int> vOrderedWs(vPairs.size());
    for(size_t i=0, iend=vPairs.size(); i<iend; ++i)
    {
        vOrderedKFs[i] = vPairs[i].second;
        vOrderedWs[i] = vPairs[i].first;
    }

    {
        unique_lock<mutex> lockCon(mMutexConnections);

        mConnectedKeyFrameWeights = KFcounter;
        mvpOrderedConnectedKeyFrames = std::move(vOrderedKFs);
        mvOrderedWeights = std::move(vOrderedWs);

        if(mbFirstConnection && mnId!=0)
        {
            mpParent = mvpOrderedConnectedKeyFrames.front();
            mpParent->AddChild(this);
            mbFirstConnection = false;
        }

    }
}

void KeyFrame::AddChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.insert(pKF);
}

void KeyFrame::EraseChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.erase(pKF);
}

void KeyFrame::ChangeParent(KeyFrame *pKF)
{
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        mpParent = pKF;
    }
    pKF->AddChild(this);
}

set<KeyFrame*> KeyFrame::GetChilds()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens;
}

KeyFrame* KeyFrame::GetParent()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mpParent;
}

bool KeyFrame::hasChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens.count(pKF);
}

void KeyFrame::AddLoopEdge(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    mspLoopEdges.insert(pKF);
}

set<KeyFrame*> KeyFrame::GetLoopEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspLoopEdges;
}

void KeyFrame::SetNotErase()
{
    unique_lock<mutex> lock(mMutexConnections);
    mbNotErase = true;
}

void KeyFrame::SetErase()
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mspLoopEdges.empty())
        {
            mbNotErase = false;
        }
    }

    if(mbToBeErased)
    {
        SetBadFlag();
    }
}

void KeyFrame::SetBadFlag()
{   
    // 避免在持有锁的情况下修改被迭代的容器
    // 1. 拷贝连接关系
    map<KeyFrame*,int> connectedWeightsCopy;
    vector<MapPoint*> mapPointsCopy;
    
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mnId==0)
            return;
        else if(mbNotErase)
        {
            mbToBeErased = true;
            return;
        }
        connectedWeightsCopy = mConnectedKeyFrameWeights;
        mConnectedKeyFrameWeights.clear(); // 提前清空，防止后续再次访问
    }

    {
        unique_lock<mutex> lockFeatures(mMutexFeatures);
        mapPointsCopy = mvpMapPoints;
        mvpMapPoints.assign(N, static_cast<MapPoint*>(NULL));
    }

    // 2. 在锁外进行所有的 Erase 操作，避免死锁和迭代器失效
    for(map<KeyFrame*,int>::iterator mit = connectedWeightsCopy.begin(), mend=connectedWeightsCopy.end(); mit!=mend; mit++)
    {
        mit->first->EraseConnection(this);
    }

    for(size_t i=0; i<mapPointsCopy.size(); i++)
    {
        if(mapPointsCopy[i])
            mapPointsCopy[i]->EraseObservation(this);
    }

    set<KeyFrame*> mspChildrensCopy;
    KeyFrame* pParentCopy = nullptr;

    {
        unique_lock<mutex> lock(mMutexConnections);
        
        mvpOrderedConnectedKeyFrames.clear();
        mspChildrensCopy = mspChildrens;
        pParentCopy = mpParent;
        
        mspChildrens.clear();
    }

    // 更新生成树 (在锁外安全执行，彻底根治与 ChangeParent 的死锁)
    set<KeyFrame*> sParentCandidates;
    sParentCandidates.insert(pParentCopy);

    // 在每次迭代中，为子节点分配一个父节点（共视权重最高的那对）
    // 将该子节点作为其余节点的新父节点候选
    while(!mspChildrensCopy.empty())
    {
        bool bContinue = false;

        int max = -1;
        KeyFrame* pC = nullptr;
        KeyFrame* pP = nullptr;

        for(set<KeyFrame*>::iterator sit=mspChildrensCopy.begin(), send=mspChildrensCopy.end(); sit!=send; sit++)
        {
            KeyFrame* pKF = *sit;
            if(pKF->isBad())
                continue;

            // 检查父节点候选是否连接到关键帧
            vector<KeyFrame*> vpConnected = pKF->GetVectorCovisibleKeyFrames();
            for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
            {
                for(set<KeyFrame*>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                {
                    if(vpConnected[i]->mnId == (*spcit)->mnId)
                    {
                        int w = pKF->GetWeight(vpConnected[i]);
                        if(w>max)
                        {
                            pC = pKF;
                            pP = vpConnected[i];
                            max = w;
                            bContinue = true;
                        }
                    }
                }
            }
        }

        if(bContinue)
        {
            pC->ChangeParent(pP);
            sParentCandidates.insert(pC);
            mspChildrensCopy.erase(pC);
        }
        else
            break;
    }

    // 如果子节点与任何父节点候选没有共视链接，则将其分配给此KF的原始父节点
    if(!mspChildrensCopy.empty())
    {
        for(set<KeyFrame*>::iterator sit=mspChildrensCopy.begin(); sit!=mspChildrensCopy.end(); sit++)
        {
            (*sit)->ChangeParent(pParentCopy);
        }
    }

    if(pParentCopy)
    {
        pParentCopy->EraseChild(this);
    }

    {
        unique_lock<mutex> lock(mMutexConnections);
        if(pParentCopy)
            mTcp = Tcw * pParentCopy->GetPoseInverse();
        mbBad = true;
    }


    mpMap->EraseKeyFrame(this);
    mpKeyFrameDB->erase(this);
}

bool KeyFrame::isBad()
{
    // mbBad已改为原子变量，无需加锁
    return mbBad;
}

void KeyFrame::EraseConnection(KeyFrame* pKF)
{
    bool bUpdate = false;
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mConnectedKeyFrameWeights.count(pKF))
        {
            mConnectedKeyFrameWeights.erase(pKF);
            bUpdate=true;
        }
    }

    if(bUpdate)
        UpdateBestCovisibles();
}

std::vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const
{
    std::vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=mnGridCols)
        return vIndices;

    const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=mnGridRows)
        return vIndices;

    const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const std::vector<size_t>& vCell = mGrid[ix][iy];
            const float rSq = r*r;
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;
                const float distSq = distx*distx + disty*disty;

                if(distSq < rSq)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool KeyFrame::IsInImage(const float &x, const float &y) const
{
    return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
}

cv::Mat KeyFrame::UnprojectStereo(int i)
{
    // 单目模式不支持双目反投影
    return cv::Mat();
}

float KeyFrame::ComputeSceneMedianDepth(const int q)
{
    std::vector<MapPoint*> vpMapPoints;
    cv::Mat Tcw_;
    {
        std::unique_lock<std::mutex> lock(mMutexFeatures, std::defer_lock);
        std::unique_lock<std::mutex> lock2(mMutexPose, std::defer_lock);
        
        // 同时锁定以避免死锁
        std::lock(lock, lock2);
        
        vpMapPoints = mvpMapPoints;
        Tcw_ = Tcw.clone();
    }

    std::vector<float> vDepths;
    vDepths.reserve(N);
    cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);
    Rcw2 = Rcw2.t();
    float zcw = Tcw_.at<float>(2,3);
    for(int i=0; i<N; i++)
    {
        if(vpMapPoints[i])
        {
            MapPoint* pMP = vpMapPoints[i];
            cv::Mat x3Dw = pMP->GetWorldPos();
            float z = Rcw2.dot(x3Dw)+zcw;
            vDepths.push_back(z);
        }
    }

    std::sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];
}

std::shared_ptr<HBSTTree> KeyFrame::GetHBSTTree() {
    unique_lock<mutex> lock(mMutexFeatures);
    if (!mpTree && !mDescriptors.empty()) {
        mpTree = std::make_shared<HBSTTree>();
        std::vector<size_t> objects(N);
        for(int i=0; i<N; i++) objects[i] = i;
        HBSTTree::MatchableVector matchables = HBSTTree::getMatchables(mDescriptors, objects, mnId);
        mpTree->add(matchables);
    }
    return mpTree;
}

} //namespace ORB_SLAM2

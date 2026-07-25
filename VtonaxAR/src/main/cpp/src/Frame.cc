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

#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <thread>
#include "Config.h"
#include <algorithm>

using namespace std;

namespace ORB_SLAM2
{

long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

Frame::Frame()
    : mpTree(nullptr)
{}

// 复制构造函数
Frame::Frame(const Frame &frame)
    :mpORBextractorLeft(frame.mpORBextractorLeft),
     mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
     mbf(frame.mbf), mb(frame.mb), N(frame.N), mvKeys(frame.mvKeys),
     mvKeysUn(frame.mvKeysUn),
     mDescriptors(frame.mDescriptors.clone()),
     mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
     mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
     mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
     mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
     mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2),
     mpTree(frame.mpTree)
{
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=frame.mGrid[i][j];

    if(!frame.mTcw.empty())
        SetPose(frame.mTcw);
}

Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBextractorLeft(extractor),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mpTree(nullptr)
{
    // 帧 ID
    mnId=nNextId++;

    // 尺度层级信息
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB提取
    ExtractORB(0,imGray);

    // logTime();
    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    // 单目模式下，mvKeysUn与mvKeys相同（不需要去失真处理）
    mvKeysUn = mvKeys;

    mvpMapPoints = std::vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = std::vector<bool>(N,false);

    // 仅对第一帧（或校准更改后）执行此操作
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    // 确保网格元素宽度和高度倒数已初始化
    if(mfGridElementWidthInv <= 0 || mfGridElementHeightInv <= 0)
    {
        return;
    }

    AssignFeaturesToGrid();
}

void Frame::AssignFeaturesToGrid()
{
    // 确保N和mvKeysUn有效
    if(N <= 0 || mvKeysUn.empty())
    {
        return;
    }

    int nReserve = FRAME_GRID_RESERVE_FACTOR*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);

    for(int i=0;i<N;i++)
    {
        // 确保索引在有效范围内
        if(i >= (int)mvKeysUn.size())
        {
            continue;
        }

        const cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
        {
            // 确保网格位置在有效范围内
            if(nGridPosX >= 0 && nGridPosX < FRAME_GRID_COLS && 
               nGridPosY >= 0 && nGridPosY < FRAME_GRID_ROWS)
            {
                mGrid[nGridPosX][nGridPosY].push_back(i);
            }
        }
    }
}

void Frame::ExtractORB(int flag, const cv::Mat &im)
{
    // 单目模式只使用左图像提取器
        (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors);
}

void Frame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
}

void Frame::UpdatePoseMatrices()
{ 
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0,3).col(3);
    mOw = -mRcw.t()*mtcw;
}

bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    pMP->mbTrackInView = false;

    // 3D绝对坐标 (免分配获取)
    cv::Point3f p3f;
    pMP->GetWorldPos(p3f);

    // 相机坐标系中的3D坐标 (手动展开消除cv::Mat乘法分配开销)
    const float PcX = mRcw.at<float>(0,0)*p3f.x + mRcw.at<float>(0,1)*p3f.y + mRcw.at<float>(0,2)*p3f.z + mtcw.at<float>(0);
    const float PcY = mRcw.at<float>(1,0)*p3f.x + mRcw.at<float>(1,1)*p3f.y + mRcw.at<float>(1,2)*p3f.z + mtcw.at<float>(1);
    const float PcZ = mRcw.at<float>(2,0)*p3f.x + mRcw.at<float>(2,1)*p3f.y + mRcw.at<float>(2,2)*p3f.z + mtcw.at<float>(2);

    // 检查正深度
    if(PcZ<0.0f)
        return false;

    // 投影到图像中并检查是否超出边界
    const float invz = 1.0f/PcZ;
    const float u=fx*PcX*invz+cx;
    const float v=fy*PcY*invz+cy;

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    // 检查常规点的距离不变性和视角。
    // 对于没有描述符的已加载点，放宽约束以允许基于投影的匹配。
    const float POx = p3f.x - mOw.at<float>(0);
    const float POy = p3f.y - mOw.at<float>(1);
    const float POz = p3f.z - mOw.at<float>(2);
    const float distSq = POx*POx + POy*POy + POz*POz;

    float viewCos = 1.0f;
    int nPredictedLevel = 0;
    if(!(pMP->mbFromLoadedMap && pMP->GetDescriptor().empty()))
    {
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        // 使用距离平方先判定，避免非必要点的 sqrt 开方计算
        if(distSq < minDistance*minDistance || distSq > maxDistance*maxDistance)
            return false;

        // 检查视角使用平方乘积判定，无超越函数无除法
        cv::Point3f normal;
        pMP->GetNormal(normal);
        const float dotVal = POx*normal.x + POy*normal.y + POz*normal.z;
        if(dotVal < 0.0f || dotVal*dotVal < viewingCosLimit*viewingCosLimit*distSq)
            return false;

        const float dist = std::sqrt(distSq);
        viewCos = dotVal / dist;

        // 预测图像中的尺度
        nPredictedLevel = pMP->PredictScale(dist,this);
    }
    else
    {
        // 宽松路径：仅保持在图像边界内；选择一个合理的层级
        nPredictedLevel = std::min(std::max(0, mnScaleLevels/2), mnScaleLevels-1);
    }

    // 跟踪使用的数据
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = u;
    pMP->mTrackProjXR = u - mbf*invz;
    pMP->mTrackProjY = v;
    pMP->mnTrackScaleLevel= nPredictedLevel;
    pMP->mTrackViewCos = viewCos;

    return true;
}

std::vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
{
    std::vector<size_t> vIndices;
    // 半径搜索预留16槽位替代原来的全量reserve(N≈1000)

    // TODO: 临时数值，待测试和优化
    vIndices.reserve(16);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            // 使用 const 引用，避免对每个 grid cell 触发 vector 深拷贝
            const std::vector<size_t>& vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    // 确保网格元素尺寸已初始化
    if(mfGridElementWidthInv <= 0 || mfGridElementHeightInv <= 0)
    {
        posX = -1;
        posY = -1;
        return false;
    }

    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    // 关键点坐标是去畸变的，这可能导致超出图像范围
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}


void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    if(mDistCoef.at<float>(0)!=0.0)
    {
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;
        mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;

        // 去畸变角点
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
        mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
        mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
        mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));

    }
    else
    {
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
}

std::shared_ptr<HBSTTree> Frame::GetHBSTTree() {
    if (!mpTree && !mDescriptors.empty()) {
        mpTree = std::make_shared<HBSTTree>();
        std::vector<size_t> objects(N);
        for (int i = 0; i < N; i++) objects[i] = i;
        HBSTTree::MatchableVector matchables = HBSTTree::getMatchables(mDescriptors, objects, mnId);
        mpTree->add(matchables);
    }
    return mpTree;
}

} //namespace ORB_SLAM2

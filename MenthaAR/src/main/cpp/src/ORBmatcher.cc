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

#include "ORBmatcher.h"

#include<limits.h>

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "Config.h"

//#include<stdint-gcc.h>
#include <stdint.h>
#include <cstring>

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>
#endif

using namespace std;

namespace ORB_SLAM2
{

// 辅助内联函数：用于在 HBST 树中快速查找对应描述子的叶子节点，以避免代码重复
inline const HBSTNode* FindHBSTLeafNode(const HBSTTree* tree, const HBSTMatchable::Descriptor &desc) {
    if (!tree) return nullptr;
    const HBSTNode* node = tree->root();
    while (node && node->hasLeafs()) {
        node = desc[node->indexSplitBit()] ? node->right : node->left;
    }
    return node;
}

const int ORBmatcher::TH_HIGH = ORB_MATCHER_TH_HIGH;
const int ORBmatcher::TH_LOW = ORB_MATCHER_TH_LOW;
const int ORBmatcher::HISTO_LENGTH = ORB_MATCHER_HISTO_LENGTH;

ORBmatcher::ORBmatcher(float nnratio, bool checkOri): mfNNratio(nnratio), mbCheckOrientation(checkOri)
{
}

int ORBmatcher::SearchByProjection(Frame &F, const vector<MapPoint*> &vpMapPoints, const float th)
{
    int nmatches=0;

    const bool bFactor = th!=1.0;

    for(size_t iMP=0; iMP<vpMapPoints.size(); iMP++)
    {
        MapPoint* pMP = vpMapPoints[iMP];
        if(!pMP->mbTrackInView)
            continue;

        if(pMP->isBad())
            continue;

        const int &nPredictedLevel = pMP->mnTrackScaleLevel;

        // 窗口大小取决于视角方向
        float r = RadiusByViewingCos(pMP->mTrackViewCos);

        if(bFactor)
            r*=th;

        const vector<size_t> vIndices =
                F.GetFeaturesInArea(pMP->mTrackProjX,pMP->mTrackProjY,r*F.mvScaleFactors[nPredictedLevel],nPredictedLevel-1,nPredictedLevel);

        if(vIndices.empty())
            continue;

        const cv::Mat MPdescriptor = pMP->GetDescriptor();

        int bestDist=256;
        int bestLevel= -1;
        int bestDist2=256;
        int bestLevel2 = -1;
        int bestIdx =-1 ;

        // 获取附近关键点的最佳和次佳匹配
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            if(F.mvpMapPoints[idx])
                if(F.mvpMapPoints[idx]->Observations()>0)
                    continue;

            const cv::Mat &d = F.mDescriptors.row(idx);

            const int dist = DescriptorDistance(MPdescriptor,d);

            if(dist<bestDist)
            {
                bestDist2=bestDist;
                bestDist=dist;
                bestLevel2 = bestLevel;
                bestLevel = F.mvKeysUn[idx].octave;
                bestIdx=idx;
                if (bestDist == 0) break;  // 零距离无法被超越
            }
            else if(dist<bestDist2)
            {
                bestLevel2 = F.mvKeysUn[idx].octave;
                bestDist2=dist;
            }
        }

        // 对次佳匹配应用比例（仅当最佳和次佳在同一尺度级别时）
        if(bestDist<=TH_HIGH)
        {
            if(bestLevel==bestLevel2 && bestDist>mfNNratio*bestDist2)
                continue;

            F.mvpMapPoints[bestIdx]=pMP;
            nmatches++;
        }
    }

    return nmatches;
}

float ORBmatcher::RadiusByViewingCos(const float &viewCos)
{
    if(viewCos>ORB_MATCHER_VIEW_COS_TH)
        return 2.5;
    else
        return 4.0;
}

bool ORBmatcher::CheckDistEpipolarLine(const cv::KeyPoint &kp1,const cv::KeyPoint &kp2,const cv::Mat &F12,const KeyFrame* pKF2)
{
    // 第二幅图像中的极线 l = x1'F12 = [a b c]
    const float a = kp1.pt.x*F12.at<float>(0,0)+kp1.pt.y*F12.at<float>(1,0)+F12.at<float>(2,0);
    const float b = kp1.pt.x*F12.at<float>(0,1)+kp1.pt.y*F12.at<float>(1,1)+F12.at<float>(2,1);
    const float c = kp1.pt.x*F12.at<float>(0,2)+kp1.pt.y*F12.at<float>(1,2)+F12.at<float>(2,2);

    const float num = a*kp2.pt.x+b*kp2.pt.y+c;

    const float den = a*a+b*b;

    if(den==0)
        return false;

    // 优化: 消除除法 num*num/den < th 等价于 num*num < den*th
    // 数学等价: a/b < c ⇔ a < b*c (当b>0)
    const float epiTh = ORB_MATCHER_EPILINE_TH * pKF2->mvLevelSigma2[kp2.octave];
    return num*num < den * epiTh;
}

int ORBmatcher::SearchByHBST(KeyFrame* pKF,Frame &F, vector<MapPoint*> &vpMapPointMatches)
{
    if(!pKF || pKF->isBad())
        return 0;

    const vector<MapPoint*> vpMapPointsKF = pKF->GetMapPointMatches();
    vpMapPointMatches = vector<MapPoint*>(F.N,static_cast<MapPoint*>(NULL));

    int nmatches=0;

    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    if(pKF->mDescriptors.empty() || F.mDescriptors.empty())
        return 0;

    const int nKFDescriptors = pKF->mDescriptors.rows;
    if (vpMapPointsKF.size() < static_cast<size_t>(nKFDescriptors))
        return 0;

    const int nFDescriptors = F.mDescriptors.rows;

    std::shared_ptr<HBSTTree> treeKF = pKF->GetHBSTTree();
    if (!treeKF)
        return 0;
    const auto& matchablesKF = treeKF->matchables();

    std::shared_ptr<HBSTTree> treeF = F.GetHBSTTree();
    if (!treeF)
        return 0;

    for(int iKF=0; iKF<nKFDescriptors; iKF++)
    {
        MapPoint* pMP = vpMapPointsKF[iKF];
        if(!pMP || pMP->isBad())
            continue;

        // 直接从关键帧预构建的 HBST 树中获取 bitset 描述子，避免重复进行二进制转换
        const HBSTMatchable::Descriptor &descKF = matchablesKF[iKF]->descriptor;

        // 使用辅助内联函数遍历查找叶子节点，减少重复代码
        const HBSTNode* node_current = FindHBSTLeafNode(treeF.get(), descKF);
        if(!node_current)
            continue;

        const auto& candidates = node_current->getMatchables();

        int bestDist1 = 256;
        int bestIdxF = -1;
        int bestDist2 = 256;

        for (const auto* candidate : candidates) {
            size_t realIdxF = candidate->objects.begin()->second;

            if (vpMapPointMatches[realIdxF])
                continue;

            // 直接在 bitset 上通过异或和计数来计算汉明距离，实现极致速度且无内存拷贝与函数调用
            const int dist = (descKF ^ candidate->descriptor).count();

            if (dist < bestDist1) {
                bestDist2 = bestDist1;
                bestDist1 = dist;
                bestIdxF = realIdxF;
            } else if (dist < bestDist2) {
                bestDist2 = dist;
            }
        }

        if (bestDist1 <= TH_LOW) {
            // 尺度一致性检查：若金字塔层级差异过大，增加等效距离
            if (mbCheckOrientation) {
                int scaleKF_i = pKF->mvKeysUn[iKF].octave;
                int scaleF_i = F.mvKeys[bestIdxF].octave;
                float scaleDiff = fabs(static_cast<float>(scaleKF_i - scaleF_i));
                if (scaleDiff > 2.0f) {
                    // 尺度差异过大，增加等效距离减少其被选中的概率
                    float effectiveDist = bestDist1 * (1.0f + scaleDiff * 0.15f);
                    if (effectiveDist > TH_LOW) {
                        continue;  // 等效距离超过阈值，丢弃此匹配
                    }
                    // 仍接受但降低ratio test的实际约束
                    if (static_cast<float>(bestDist1) < mfNNratio * static_cast<float>(bestDist2) * (1.0f + scaleDiff * 0.1f)) {
                        vpMapPointMatches[bestIdxF] = pMP;
                        float rot = pKF->mvKeysUn[iKF].angle - F.mvKeys[bestIdxF].angle;
                        if (rot < 0.0) rot += 360.0f;
                        int bin = round(rot * factor);
                        if (bin == HISTO_LENGTH) bin = 0;
                        assert(bin >= 0 && bin < HISTO_LENGTH);
                        rotHist[bin].push_back(bestIdxF);
                        nmatches++;
                    }
                } else {
                    if (static_cast<float>(bestDist1) < mfNNratio * static_cast<float>(bestDist2)) {
                        vpMapPointMatches[bestIdxF] = pMP;
                        float rot = pKF->mvKeysUn[iKF].angle - F.mvKeys[bestIdxF].angle;
                        if (rot < 0.0) rot += 360.0f;
                        int bin = round(rot * factor);
                        if (bin == HISTO_LENGTH) bin = 0;
                        assert(bin >= 0 && bin < HISTO_LENGTH);
                        rotHist[bin].push_back(bestIdxF);
                        nmatches++;
                    }
                }
            } else {
                if (static_cast<float>(bestDist1) < mfNNratio * static_cast<float>(bestDist2)) {
                    vpMapPointMatches[bestIdxF] = pMP;
                    nmatches++;
                }
            }
        }
    }

    if (mbCheckOrientation) {
        // 小样本时跳过旋转直方图过滤：当匹配总数不足时，
        // 旋转直方图的主方向统计不可靠，过滤会过度剔除有效匹配。
        if (nmatches >= HISTO_LENGTH) {
            int ind1 = -1;
            int ind2 = -1;
            int ind3 = -1;

            ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

            for (int i = 0; i < HISTO_LENGTH; i++) {
                if (i == ind1 || i == ind2 || i == ind3)
                    continue;
                for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
                    vpMapPointMatches[rotHist[i][j]] = static_cast<MapPoint*>(NULL);
                    nmatches--;
                }
            }
        }
    }

    return nmatches;
}

int ORBmatcher::SearchByProjection(KeyFrame* pKF, cv::Mat Scw, const vector<MapPoint*> &vpPoints, vector<MapPoint*> &vpMatched, int th)
{
    // 获取校准参数用于后续投影
    const float &fx = pKF->fx;
    const float &fy = pKF->fy;
    const float &cx = pKF->cx;
    const float &cy = pKF->cy;

    // 分解 Scw
    cv::Mat sRcw = Scw.rowRange(0,3).colRange(0,3);
    const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0)));
    // 用乘法代替矩阵除法以加快计算速度，因为计算机算乘法明显快于除法
    const float inv_scw = 1.0f/scw;
    cv::Mat Rcw = sRcw * inv_scw;
    cv::Mat tcw = Scw.rowRange(0,3).col(3) * inv_scw;
    cv::Mat Ow = -Rcw.t()*tcw;
    // 将矩阵数据在循环外部加载至栈上，消除循环内部的 Mat 访问开销
    const float R00 = Rcw.at<float>(0,0), R01 = Rcw.at<float>(0,1), R02 = Rcw.at<float>(0,2);
    const float R10 = Rcw.at<float>(1,0), R11 = Rcw.at<float>(1,1), R12 = Rcw.at<float>(1,2);
    const float R20 = Rcw.at<float>(2,0), R21 = Rcw.at<float>(2,1), R22 = Rcw.at<float>(2,2);
    const float tx = tcw.at<float>(0), ty = tcw.at<float>(1), tz = tcw.at<float>(2);
    const float Ox = Ow.at<float>(0), Oy = Ow.at<float>(1), Oz = Ow.at<float>(2);

    // 关键帧中已找到的地图点集合
    set<MapPoint*> spAlreadyFound(vpMatched.begin(), vpMatched.end());
    spAlreadyFound.erase(static_cast<MapPoint*>(NULL));

    int nmatches=0;

    // 对每个候选地图点进行投影和匹配
    for(int iMP=0, iendMP=vpPoints.size(); iMP<iendMP; iMP++)
    {
        MapPoint* pMP = vpPoints[iMP];

        // 舍弃坏的地图点 and 已找到的点
        if(pMP->isBad() || spAlreadyFound.count(pMP))
            continue;

        // 获取3D坐标 (使用 Point3f 替代 cv::Mat，消除堆分配)
        cv::Point3f p3Dw;
        pMP->GetWorldPos(p3Dw);

        // 转换到相机坐标系 (标量乘加替代矩阵乘法)
        const float p3DcX = R00 * p3Dw.x + R01 * p3Dw.y + R02 * p3Dw.z + tx;
        const float p3DcY = R10 * p3Dw.x + R11 * p3Dw.y + R12 * p3Dw.z + ty;
        const float p3DcZ = R20 * p3Dw.x + R21 * p3Dw.y + R22 * p3Dw.z + tz;

        // 深度必须为正
        if(p3DcZ<0.0f)
            continue;

        // 投影到图像
        const float invz = 1.0f/p3DcZ;
        const float x = p3DcX*invz;
        const float y = p3DcY*invz;

        const float u = fx*x+cx;
        const float v = fy*y+cy;

        // 点必须在图像内部
        if(!pKF->IsInImage(u,v))
            continue;

        // 深度必须在点的尺度不变区域内
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();

        const float POx = p3Dw.x - Ox;
        const float POy = p3Dw.y - Oy;
        const float POz = p3Dw.z - Oz;
        const float distSq = POx*POx + POy*POy + POz*POz;

        if(distSq < minDistance*minDistance || distSq > maxDistance*maxDistance)
            continue;

        // 观察角度过滤：使用栈获取法向，先进行平方不等式判定，再决定是否计算 sqrt
        cv::Point3f Pn;
        pMP->GetNormal(Pn);
        const float dotVal = POx*Pn.x + POy*Pn.y + POz*Pn.z;
        if(dotVal < 0.0f || dotVal*dotVal < 0.25f*distSq)
            continue;

        const float dist = sqrt(distSq);

        int nPredictedLevel = pMP->PredictScale(dist,pKF);

        // 在半径内搜索
        const float radius = th*pKF->mvScaleFactors[nPredictedLevel];

        const vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // 匹配半径内最相似的关键点
        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = 256;
        int bestIdx = -1;
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;
            if(vpMatched[idx])
                continue;

            const int &kpLevel= pKF->mvKeysUn[idx].octave;

            if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                continue;

            const cv::Mat &dKF = pKF->mDescriptors.row(idx);

            const int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
                if (bestDist == 0) break;
            }
        }

        if(bestDist<=TH_LOW)
        {
            vpMatched[bestIdx]=pMP;
            nmatches++;
        }

    }

    return nmatches;
}

int ORBmatcher::SearchForInitialization(Frame &F1, Frame &F2, vector<cv::Point2f> &vbPrevMatched, vector<int> &vnMatches12, int windowSize)
{
    int nmatches=0;
    vnMatches12 = vector<int>(F1.mvKeysUn.size(),-1);

    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    vector<int> vMatchedDistance(F2.mvKeysUn.size(),INT_MAX);
    vector<int> vnMatches21(F2.mvKeysUn.size(),-1);

    for(size_t i1=0, iend1=F1.mvKeysUn.size(); i1<iend1; i1++)
    {
        cv::KeyPoint kp1 = F1.mvKeysUn[i1];
        int level1 = kp1.octave;
        if(level1>0)
            continue;

        vector<size_t> vIndices2 = F2.GetFeaturesInArea(vbPrevMatched[i1].x,vbPrevMatched[i1].y, windowSize,level1,level1);

        if(vIndices2.empty())
            continue;

        cv::Mat d1 = F1.mDescriptors.row(i1);

        int bestDist = INT_MAX;
        int bestDist2 = INT_MAX;
        int bestIdx2 = -1;

        for(vector<size_t>::iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
        {
            size_t i2 = *vit;

            cv::Mat d2 = F2.mDescriptors.row(i2);

            int dist = DescriptorDistance(d1,d2);

            if(vMatchedDistance[i2]<=dist)
                continue;

            if(dist<bestDist)
            {
                bestDist2=bestDist;
                bestDist=dist;
                bestIdx2=i2;
            }
            else if(dist<bestDist2)
            {
                bestDist2=dist;
            }
        }

        if(bestDist<=TH_LOW)
        {
            if(bestDist<(float)bestDist2*mfNNratio)
            {
                if(vnMatches21[bestIdx2]>=0)
                {
                    vnMatches12[vnMatches21[bestIdx2]]=-1;
                    nmatches--;
                }
                vnMatches12[i1]=bestIdx2;
                vnMatches21[bestIdx2]=i1;
                vMatchedDistance[bestIdx2]=bestDist;
                nmatches++;

                if(mbCheckOrientation)
                {
                    float rot = F1.mvKeysUn[i1].angle-F2.mvKeysUn[bestIdx2].angle;
                    if(rot<0.0)
                        rot+=360.0f;
                    int bin = round(rot*factor);
                    if(bin==HISTO_LENGTH)
                        bin=0;
                    assert(bin>=0 && bin<HISTO_LENGTH);
                    rotHist[bin].push_back(i1);
                }
            }
        }

    }

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                int idx1 = rotHist[i][j];
                if(vnMatches12[idx1]>=0)
                {
                    vnMatches12[idx1]=-1;
                    nmatches--;
                }
            }
        }

    }

    // 更新之前的匹配
    for(size_t i1=0, iend1=vnMatches12.size(); i1<iend1; i1++)
        if(vnMatches12[i1]>=0)
            vbPrevMatched[i1]=F2.mvKeysUn[vnMatches12[i1]].pt;

    return nmatches;
}

int ORBmatcher::SearchByHBST(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches12)
{
    if (!pKF1 || pKF1->isBad() || !pKF2 || pKF2->isBad())
        return 0;

    const vector<cv::KeyPoint> &vKeysUn1 = pKF1->mvKeysUn;
    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
    const cv::Mat &Descriptors1 = pKF1->mDescriptors;

    const vector<cv::KeyPoint> &vKeysUn2 = pKF2->mvKeysUn;
    const vector<MapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches();
    const cv::Mat &Descriptors2 = pKF2->mDescriptors;

    vpMatches12 = vector<MapPoint*>(vpMapPoints1.size(), static_cast<MapPoint*>(NULL));
    vector<bool> vbMatched2(vpMapPoints2.size(), false);

    vector<int> rotHist[HISTO_LENGTH];
    for (int i = 0; i < HISTO_LENGTH; i++)
        rotHist[i].reserve(500);

    const float factor = 1.0f / HISTO_LENGTH;

    if (Descriptors1.empty() || Descriptors2.empty())
        return 0;

    const int nDescriptors1 = Descriptors1.rows;
    const int nDescriptors2 = Descriptors2.rows;

    if (vpMapPoints1.size() < static_cast<size_t>(nDescriptors1) || vpMapPoints2.size() < static_cast<size_t>(nDescriptors2))
        return 0;

    int nmatches = 0;

    std::shared_ptr<HBSTTree> tree1 = pKF1->GetHBSTTree();
    if (!tree1)
        return 0;
    const auto& matchables1 = tree1->matchables();

    // Obtain the pre-built cached tree of KeyFrame pKF2
    std::shared_ptr<HBSTTree> tree2 = pKF2->GetHBSTTree();
    if (!tree2)
        return 0;

    // Loop over KeyFrame pKF1's features
    for (int idx1 = 0; idx1 < nDescriptors1; idx1++) {
        MapPoint* pMP1 = vpMapPoints1[idx1];
        if (!pMP1 || pMP1->isBad())
            continue;

        // 直接从关键帧1预构建的 HBST 树中获取 bitset 描述子，避免重复进行二进制转换
        const HBSTMatchable::Descriptor &desc1 = matchables1[idx1]->descriptor;

        // 使用辅助内联函数遍历查找叶子节点，减少重复代码
        const HBSTNode* node_current = FindHBSTLeafNode(tree2.get(), desc1);
        if (!node_current)
            continue;

        const auto& candidates = node_current->getMatchables();

        int bestDist1 = 256;
        int bestIdx2 = -1;
        int bestDist2 = 256;

        for (const auto* candidate : candidates) {
            size_t idx2 = candidate->objects.begin()->second;

            MapPoint* pMP2 = vpMapPoints2[idx2];
            if (!pMP2 || pMP2->isBad() || vbMatched2[idx2])
                continue;

            // 直接在 bitset 上通过异或和计数来计算汉明距离，实现极致速度且无内存拷贝与函数调用
            const int dist = (desc1 ^ candidate->descriptor).count();

            if (dist < bestDist1) {
                bestDist2 = bestDist1;
                bestDist1 = dist;
                bestIdx2 = idx2;
            } else if (dist < bestDist2) {
                bestDist2 = dist;
            }
        }

        if (bestDist1 < TH_LOW) {
            // 尺度一致性检查：若金字塔层级差异过大，增加等效距离
            if (mbCheckOrientation) {
                int scaleKP1 = vKeysUn1[idx1].octave;
                int scaleKP2 = vKeysUn2[bestIdx2].octave;
                float scaleDiff = fabs(static_cast<float>(scaleKP1 - scaleKP2));
                if (scaleDiff > 2.0f) {
                    float effectiveDist = bestDist1 * (1.0f + scaleDiff * 0.15f);
                    if (effectiveDist > TH_LOW) {
                        continue;
                    }
                    if (static_cast<float>(bestDist1) < mfNNratio * static_cast<float>(bestDist2) * (1.0f + scaleDiff * 0.1f)) {
                        vpMatches12[idx1] = vpMapPoints2[bestIdx2];
                        vbMatched2[bestIdx2] = true;
                        float rot = vKeysUn1[idx1].angle - vKeysUn2[bestIdx2].angle;
                        if (rot < 0.0) rot += 360.0f;
                        int bin = round(rot * factor);
                        if (bin == HISTO_LENGTH) bin = 0;
                        assert(bin >= 0 && bin < HISTO_LENGTH);
                        rotHist[bin].push_back(idx1);
                        nmatches++;
                    }
                } else {
                    if (static_cast<float>(bestDist1) < mfNNratio * static_cast<float>(bestDist2)) {
                        vpMatches12[idx1] = vpMapPoints2[bestIdx2];
                        vbMatched2[bestIdx2] = true;
                        float rot = vKeysUn1[idx1].angle - vKeysUn2[bestIdx2].angle;
                        if (rot < 0.0) rot += 360.0f;
                        int bin = round(rot * factor);
                        if (bin == HISTO_LENGTH) bin = 0;
                        assert(bin >= 0 && bin < HISTO_LENGTH);
                        rotHist[bin].push_back(idx1);
                        nmatches++;
                    }
                }
            } else {
                if (static_cast<float>(bestDist1) < mfNNratio * static_cast<float>(bestDist2)) {
                    vpMatches12[idx1] = vpMapPoints2[bestIdx2];
                    vbMatched2[bestIdx2] = true;
                    nmatches++;
                }
            }
        }
    }

    if (mbCheckOrientation) {
        // 小样本时跳过旋转直方图过滤
        if (nmatches >= HISTO_LENGTH) {
            int ind1 = -1;
            int ind2 = -1;
            int ind3 = -1;

            ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

            for (int i = 0; i < HISTO_LENGTH; i++) {
                if (i == ind1 || i == ind2 || i == ind3)
                    continue;
                for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
                    vpMatches12[rotHist[i][j]] = static_cast<MapPoint*>(NULL);
                    nmatches--;
                }
            }
        }
    }

    return nmatches;
}

int ORBmatcher::SearchForTriangulation(KeyFrame *pKF1, KeyFrame *pKF2, cv::Mat F12,
                                       vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo)
{    
    if (!pKF1 || pKF1->isBad() || !pKF2 || pKF2->isBad())
        return 0;

    // 计算第二幅图像中的像极点
    cv::Mat Cw = pKF1->GetCameraCenter();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();
    std::shared_ptr<HBSTTree> tree1 = pKF1->GetHBSTTree();
    if (!tree1)
        return 0;
    const auto& matchables1 = tree1->matchables();

    std::shared_ptr<HBSTTree> tree2 = pKF2->GetHBSTTree();
    if (!tree2)
        return 0;

    cv::Mat C2 = R2w*Cw+t2w;
    const float invz = 1.0f/C2.at<float>(2);
    const float ex =pKF2->fx*C2.at<float>(0)*invz+pKF2->cx;
    const float ey =pKF2->fy*C2.at<float>(1)*invz+pKF2->cy;

    int nmatches=0;
    vector<bool> vbMatched2(pKF2->N,false);
    vector<int> vMatches12(pKF1->N,-1);

    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);

    const float factor = 1.0f/HISTO_LENGTH;

    // Loop over pKF1's features that DO NOT have map points
    for (int idx1 = 0; idx1 < pKF1->N; idx1++) {
        MapPoint* pMP1 = pKF1->GetMapPoint(idx1);
        if (pMP1)
            continue;

        const bool bStereo1 = false;
        if (bOnlyStereo && !bStereo1)
            continue;

        const cv::KeyPoint &kp1 = pKF1->mvKeysUn[idx1];

        // 直接从关键帧1预构建的 HBST 树中获取 bitset 描述子，避免重复进行二进制转换
        const HBSTMatchable::Descriptor &desc1 = matchables1[idx1]->descriptor;

        // 使用辅助内联函数遍历查找叶子节点，减少重复代码
        const HBSTNode* node_current = FindHBSTLeafNode(tree2.get(), desc1);
        if (!node_current)
            continue;

        const auto& candidates = node_current->getMatchables();

        int bestDist = TH_LOW;
        int bestIdx2 = -1;

        for (const auto* candidate : candidates) {
            size_t idx2 = candidate->objects.begin()->second;

            if (pKF2->GetMapPoint(idx2) || vbMatched2[idx2])
                continue;

            const bool bStereo2 = false;
            if (bOnlyStereo && !bStereo2)
                continue;

            // 直接在 bitset 上通过异或和计数来计算汉明距离，实现极致速度且无内存拷贝与函数调用
            const int dist = (desc1 ^ candidate->descriptor).count();

            if (dist > TH_LOW || dist > bestDist)
                continue;

            const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];

            if (!bStereo1 && !bStereo2) {
                const float distex = ex - kp2.pt.x;
                const float distey = ey - kp2.pt.y;
                if (distex * distex + distey * distey < 100 * pKF2->mvScaleFactors[kp2.octave])
                    continue;
            }

            if (CheckDistEpipolarLine(kp1, kp2, F12, pKF2)) {
                bestIdx2 = idx2;
                bestDist = dist;
            }
        }

        if (bestIdx2 >= 0) {
            vMatches12[idx1] = bestIdx2;
            vbMatched2[bestIdx2] = true;

            if (mbCheckOrientation) {
                float rot = kp1.angle - pKF2->mvKeysUn[bestIdx2].angle;
                if (rot < 0.0) rot += 360.0f;
                int bin = round(rot * factor);
                if (bin == HISTO_LENGTH) bin = 0;
                assert(bin >= 0 && bin < HISTO_LENGTH);
                rotHist[bin].push_back(idx1);
            }
            nmatches++;
        }
    }

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                vMatches12[rotHist[i][j]]=-1;
                nmatches--;
            }
        }
    }

    vMatchedPairs.clear();
    vMatchedPairs.reserve(nmatches);

    for(size_t i=0, iend=vMatches12.size(); i<iend; i++)
    {
        if(vMatches12[i]<0)
            continue;
        vMatchedPairs.push_back(make_pair(i,vMatches12[i]));
    }

    return nmatches;
}

int ORBmatcher::Fuse(KeyFrame *pKF, const vector<MapPoint *> &vpMapPoints, const float th)
{
    if (!pKF || pKF->isBad())
        return 0;

    cv::Mat Rcw = pKF->GetRotation();
    cv::Mat tcw = pKF->GetTranslation();
    cv::Mat Ow = pKF->GetCameraCenter();

    const float &fx = pKF->fx;
    const float &fy = pKF->fy;
    const float &cx = pKF->cx;
    const float &cy = pKF->cy;

    // 预先提取位姿与相机中心的标量值，在循环中以 O(1) 免锁且免分配执行 3D 变换
    const float R00 = Rcw.at<float>(0,0), R01 = Rcw.at<float>(0,1), R02 = Rcw.at<float>(0,2);
    const float R10 = Rcw.at<float>(1,0), R11 = Rcw.at<float>(1,1), R12 = Rcw.at<float>(1,2);
    const float R20 = Rcw.at<float>(2,0), R21 = Rcw.at<float>(2,1), R22 = Rcw.at<float>(2,2);
    const float tx = tcw.at<float>(0), ty = tcw.at<float>(1), tz = tcw.at<float>(2);
    const float Ox = Ow.at<float>(0), Oy = Ow.at<float>(1), Oz = Ow.at<float>(2);

    int nFused=0;

    const int nMPs = vpMapPoints.size();

    for(int i=0; i<nMPs; i++)
    {
        MapPoint* pMP = vpMapPoints[i];

        if(!pMP)
            continue;

        if(pMP->isBad() || pMP->IsInKeyFrame(pKF))
            continue;

        // 使用栈分配的 Point3f 替代 cv::Mat 获取 3D 坐标，消除堆分配
        cv::Point3f p3Dw;
        pMP->GetWorldPos(p3Dw);

        // 标量级 3D 旋转与平移变换，替代 cv::Mat 矩阵乘法
        const float p3DcX = R00*p3Dw.x + R01*p3Dw.y + R02*p3Dw.z + tx;
        const float p3DcY = R10*p3Dw.x + R11*p3Dw.y + R12*p3Dw.z + ty;
        const float p3DcZ = R20*p3Dw.x + R21*p3Dw.y + R22*p3Dw.z + tz;

        // 深度必须为正
        if(p3DcZ<0.0f)
            continue;

        const float invz = 1.0f/p3DcZ;
        const float x = p3DcX*invz;
        const float y = p3DcY*invz;

        const float u = fx*x+cx;
        const float v = fy*y+cy;

        // 点必须在图像内部
        if(!pKF->IsInImage(u,v))
            continue;

        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();

        // PO = p3Dw - Ow (使用纯标量减法)
        const float POx = p3Dw.x - Ox;
        const float POy = p3Dw.y - Oy;
        const float POz = p3Dw.z - Oz;

        // 使用平方距离进行快速范围检查
        const float dist3DSq = POx*POx + POy*POy + POz*POz;
        const float maxDistSq = maxDistance * maxDistance;
        const float minDistSq = minDistance * minDistance;

        // 深度必须在图像的尺度金字塔内
        if(dist3DSq < minDistSq || dist3DSq > maxDistSq)
            continue;

        // 只在需要时计算实际距离
        const float dist3D = sqrt(dist3DSq);

        // 使用栈分配的 Point3f 替代 cv::Mat 获取法向量
        cv::Point3f Pn;
        pMP->GetNormal(Pn);

        // 使用纯标量点乘
        const float dotProd = POx*Pn.x + POy*Pn.y + POz*Pn.z;
        if(dotProd < 0.5f*dist3D)
            continue;

        int nPredictedLevel = pMP->PredictScale(dist3D,pKF);

        // 在半径内搜索
        const float radius = th*pKF->mvScaleFactors[nPredictedLevel];

        const vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // 匹配半径内最相似的关键点

        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = 256;
        int bestIdx = -1;
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            const cv::KeyPoint &kp = pKF->mvKeysUn[idx];

            const int &kpLevel= kp.octave;

            if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                continue;

            // 单目模式只检查2D重投影误差
            const float &kpx = kp.pt.x;
            const float &kpy = kp.pt.y;
            const float ex = u-kpx;
            const float ey = v-kpy;
            const float e2 = ex*ex+ey*ey;

            if(e2*pKF->mvInvLevelSigma2[kpLevel]>5.99)
                continue;

            const cv::Mat &dKF = pKF->mDescriptors.row(idx);

            const int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        // 如果已经有地图点则替换，否则添加新的测量
        if(bestDist<=TH_LOW)
        {
            MapPoint* pMPinKF = pKF->GetMapPoint(bestIdx);
            if(pMPinKF)
            {
                if(!pMPinKF->isBad())
                {
                    if(pMPinKF->Observations()>pMP->Observations())
                        pMP->Replace(pMPinKF);
                    else
                        pMPinKF->Replace(pMP);
                }
            }
            else
            {
                pMP->AddObservation(pKF,bestIdx);
                pKF->AddMapPoint(pMP,bestIdx);
            }
            nFused++;
        }
    }

    return nFused;
}

int ORBmatcher::Fuse(KeyFrame *pKF, cv::Mat Scw, const vector<MapPoint *> &vpPoints, float th, vector<MapPoint *> &vpReplacePoint)
{
    if (!pKF || pKF->isBad())
        return 0;

    // 获取校准参数用于后续投影
    const float &fx = pKF->fx;
    const float &fy = pKF->fy;
    const float &cx = pKF->cx;
    const float &cy = pKF->cy;

    // 分解 Scw
    cv::Mat sRcw = Scw.rowRange(0,3).colRange(0,3);
    const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0)));
    cv::Mat Rcw = sRcw/scw;
    cv::Mat tcw = Scw.rowRange(0,3).col(3)/scw;
    cv::Mat Ow = -Rcw.t()*tcw;

    // 将矩阵数据在循环外部加载至栈上，消除循环内部的 Mat 访问开销
    const float R00 = Rcw.at<float>(0,0), R01 = Rcw.at<float>(0,1), R02 = Rcw.at<float>(0,2);
    const float R10 = Rcw.at<float>(1,0), R11 = Rcw.at<float>(1,1), R12 = Rcw.at<float>(1,2);
    const float R20 = Rcw.at<float>(2,0), R21 = Rcw.at<float>(2,1), R22 = Rcw.at<float>(2,2);
    const float tx = tcw.at<float>(0), ty = tcw.at<float>(1), tz = tcw.at<float>(2);
    const float Ox = Ow.at<float>(0), Oy = Ow.at<float>(1), Oz = Ow.at<float>(2);

    // 关键帧中已找到的地图点集合
    const set<MapPoint*> spAlreadyFound = pKF->GetMapPoints();

    int nFused=0;

    const int nPoints = vpPoints.size();

    // 对每个候选地图点进行投影和匹配
    for(int iMP=0; iMP<nPoints; iMP++)
    {
        MapPoint* pMP = vpPoints[iMP];

        // 舍弃坏的地图点和已找到的点
        if(pMP->isBad() || spAlreadyFound.count(pMP))
            continue;

        // 获取3D坐标 (使用 Point3f 替代 cv::Mat，消除堆分配)
        cv::Point3f p3Dw;
        pMP->GetWorldPos(p3Dw);

        // 转换到相机坐标系 (标量乘加替代矩阵乘法)
        const float p3DcX = R00 * p3Dw.x + R01 * p3Dw.y + R02 * p3Dw.z + tx;
        const float p3DcY = R10 * p3Dw.x + R11 * p3Dw.y + R12 * p3Dw.z + ty;
        const float p3DcZ = R20 * p3Dw.x + R21 * p3Dw.y + R22 * p3Dw.z + tz;

        // 深度必须为正
        if(p3DcZ<0.0f)
            continue;

        // 投影到图像
        const float invz = 1.0f/p3DcZ;
        const float x = p3DcX*invz;
        const float y = p3DcY*invz;

        const float u = fx*x+cx;
        const float v = fy*y+cy;

        // 点必须在图像内部
        if(!pKF->IsInImage(u,v))
            continue;

        // 深度必须在图像的尺度金字塔内
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();

        const float POx = p3Dw.x - Ox;
        const float POy = p3Dw.y - Oy;
        const float POz = p3Dw.z - Oz;
        const float dist3DSq = POx*POx + POy*POy + POz*POz;

        if(dist3DSq < minDistance*minDistance || dist3DSq > maxDistance*maxDistance)
            continue;

        // 观察角度过滤：使用栈获取法向，先进行平方不等式判定，再决定是否计算 sqrt
        cv::Point3f Pn;
        pMP->GetNormal(Pn);
        const float dotVal = POx*Pn.x + POy*Pn.y + POz*Pn.z;
        if(dotVal < 0.0f || dotVal*dotVal < 0.25f*dist3DSq)
            continue;

        const float dist3D = sqrt(dist3DSq);

        // 计算预测的尺度层级
        const int nPredictedLevel = pMP->PredictScale(dist3D,pKF);

        // 在半径内搜索
        const float radius = th*pKF->mvScaleFactors[nPredictedLevel];

        const vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // 匹配半径内最相似的关键点

        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = INT_MAX;
        int bestIdx = -1;
        for(vector<size_t>::const_iterator vit=vIndices.begin(); vit!=vIndices.end(); vit++)
        {
            const size_t idx = *vit;
            const int &kpLevel = pKF->mvKeysUn[idx].octave;

            if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                continue;

            const cv::Mat &dKF = pKF->mDescriptors.row(idx);

            int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        // 如果已经有地图点则替换，否则添加新的测量
        if(bestDist<=TH_LOW)
        {
            MapPoint* pMPinKF = pKF->GetMapPoint(bestIdx);
            if(pMPinKF)
            {
                if(!pMPinKF->isBad())
                    vpReplacePoint[iMP] = pMPinKF;
            }
            else
            {
                pMP->AddObservation(pKF,bestIdx);
                pKF->AddMapPoint(pMP,bestIdx);
            }
            nFused++;
        }
    }

    return nFused;
}

int ORBmatcher::SearchBySim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint*> &vpMatches12,
                             const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th)
{
    if (!pKF1 || pKF1->isBad() || !pKF2 || pKF2->isBad())
        return 0;

    const float &fx = pKF1->fx;
    const float &fy = pKF1->fy;
    const float &cx = pKF1->cx;
    const float &cy = pKF1->cy;

    // 世界坐标系到相机1
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();

    // 世界坐标系到相机2
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    // 相机间的变换
    cv::Mat sR12 = s12*R12;
    cv::Mat sR21 = (1.0/s12)*R12.t();
    cv::Mat t21 = -sR21*t12;

    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
    const int N1 = vpMapPoints1.size();

    const vector<MapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches();
    const int N2 = vpMapPoints2.size();

    vector<bool> vbAlreadyMatched1(N1,false);
    vector<bool> vbAlreadyMatched2(N2,false);

    for(int i=0; i<N1; i++)
    {
        MapPoint* pMP = vpMatches12[i];
        if(pMP)
        {
            vbAlreadyMatched1[i]=true;
            int idx2 = pMP->GetIndexInKeyFrame(pKF2);
            if(idx2>=0 && idx2<N2)
                vbAlreadyMatched2[idx2]=true;
        }
    }

    vector<int> vnMatch1(N1,-1);
    vector<int> vnMatch2(N2,-1);

    // 预将矩阵数据在循环外部加载至栈上，消除循环内部的 Mat 访问开销
    const float R1w00 = R1w.at<float>(0,0), R1w01 = R1w.at<float>(0,1), R1w02 = R1w.at<float>(0,2);
    const float R1w10 = R1w.at<float>(1,0), R1w11 = R1w.at<float>(1,1), R1w12 = R1w.at<float>(1,2);
    const float R1w20 = R1w.at<float>(2,0), R1w21 = R1w.at<float>(2,1), R1w22 = R1w.at<float>(2,2);
    const float t1wX = t1w.at<float>(0), t1wY = t1w.at<float>(1), t1wZ = t1w.at<float>(2);

    const float sR21_00 = sR21.at<float>(0,0), sR21_01 = sR21.at<float>(0,1), sR21_02 = sR21.at<float>(0,2);
    const float sR21_10 = sR21.at<float>(1,0), sR21_11 = sR21.at<float>(1,1), sR21_12 = sR21.at<float>(1,2);
    const float sR21_20 = sR21.at<float>(2,0), sR21_21 = sR21.at<float>(2,1), sR21_22 = sR21.at<float>(2,2);
    const float t21X = t21.at<float>(0), t21Y = t21.at<float>(1), t21Z = t21.at<float>(2);

    // 从 KF1 变换到 KF2 并搜索
    for(int i1=0; i1<N1; i1++)
    {
        MapPoint* pMP = vpMapPoints1[i1];

        if(!pMP || vbAlreadyMatched1[i1])
            continue;

        if(pMP->isBad())
            continue;

        // 获取3D坐标 (使用 Point3f 替代 cv::Mat，消除堆分配)
        cv::Point3f p3Dw;
        pMP->GetWorldPos(p3Dw);

        // 标量乘加完成连续的三维空间变换 (R1w -> sR21)
        const float p3Dc1X = R1w00*p3Dw.x + R1w01*p3Dw.y + R1w02*p3Dw.z + t1wX;
        const float p3Dc1Y = R1w10*p3Dw.x + R1w11*p3Dw.y + R1w12*p3Dw.z + t1wY;
        const float p3Dc1Z = R1w20*p3Dw.x + R1w21*p3Dw.y + R1w22*p3Dw.z + t1wZ;

        const float p3Dc2X = sR21_00*p3Dc1X + sR21_01*p3Dc1Y + sR21_02*p3Dc1Z + t21X;
        const float p3Dc2Y = sR21_10*p3Dc1X + sR21_11*p3Dc1Y + sR21_12*p3Dc1Z + t21Y;
        const float p3Dc2Z = sR21_20*p3Dc1X + sR21_21*p3Dc1Y + sR21_22*p3Dc1Z + t21Z;

        // 深度必须为正
        if(p3Dc2Z<0.0f)
            continue;

        const float invz = 1.0f/p3Dc2Z;
        const float x = p3Dc2X*invz;
        const float y = p3Dc2Y*invz;

        const float u = fx*x+cx;
        const float v = fy*y+cy;

        // 点必须在图像内部
        if(!pKF2->IsInImage(u,v))
            continue;

        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();

        // 使用平方距离进行快速范围检查
        const float dist3DSq = p3Dc2X*p3Dc2X + p3Dc2Y*p3Dc2Y + p3Dc2Z*p3Dc2Z;
        const float maxDistSq = maxDistance * maxDistance;
        const float minDistSq = minDistance * minDistance;

        // 深度必须在尺度不变区域内
        if(dist3DSq < minDistSq || dist3DSq > maxDistSq)
            continue;

        // 只在需要时计算实际距离
        const float dist3D = sqrt(dist3DSq);

        // 计算预测的八度
        const int nPredictedLevel = pMP->PredictScale(dist3D,pKF2);

        // 在半径内搜索
        const float radius = th*pKF2->mvScaleFactors[nPredictedLevel];

        const vector<size_t> vIndices = pKF2->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // 匹配半径内最相似的关键点
        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = INT_MAX;
        int bestIdx = -1;
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            const cv::KeyPoint &kp = pKF2->mvKeysUn[idx];

            if(kp.octave<nPredictedLevel-1 || kp.octave>nPredictedLevel)
                continue;

            const cv::Mat &dKF = pKF2->mDescriptors.row(idx);

            const int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        if(bestDist<=TH_HIGH)
        {
            vnMatch1[i1]=bestIdx;
        }
    }

    // 预将矩阵数据在循环外部加载至栈上，消除循环内部的 Mat 访问开销
    const float R2w00 = R2w.at<float>(0,0), R2w01 = R2w.at<float>(0,1), R2w02 = R2w.at<float>(0,2);
    const float R2w10 = R2w.at<float>(1,0), R2w11 = R2w.at<float>(1,1), R2w12 = R2w.at<float>(1,2);
    const float R2w20 = R2w.at<float>(2,0), R2w21 = R2w.at<float>(2,1), R2w22 = R2w.at<float>(2,2);
    const float t2wX = t2w.at<float>(0), t2wY = t2w.at<float>(1), t2wZ = t2w.at<float>(2);

    const float sR12_00 = sR12.at<float>(0,0), sR12_01 = sR12.at<float>(0,1), sR12_02 = sR12.at<float>(0,2);
    const float sR12_10 = sR12.at<float>(1,0), sR12_11 = sR12.at<float>(1,1), sR12_12 = sR12.at<float>(1,2);
    const float sR12_20 = sR12.at<float>(2,0), sR12_21 = sR12.at<float>(2,1), sR12_22 = sR12.at<float>(2,2);
    const float t12X = t12.at<float>(0), t12Y = t12.at<float>(1), t12Z = t12.at<float>(2);

    // 从 KF2 变换到 KF1 并搜索
    for(int i2=0; i2<N2; i2++)
    {
        MapPoint* pMP = vpMapPoints2[i2];

        if(!pMP || vbAlreadyMatched2[i2])
            continue;

        if(pMP->isBad())
            continue;

        // 获取3D坐标 (使用 Point3f 替代 cv::Mat，消除堆分配)
        cv::Point3f p3Dw;
        pMP->GetWorldPos(p3Dw);

        // 标量乘加完成连续的三维空间变换 (R2w -> sR12)
        const float p3Dc2X = R2w00*p3Dw.x + R2w01*p3Dw.y + R2w02*p3Dw.z + t2wX;
        const float p3Dc2Y = R2w10*p3Dw.x + R2w11*p3Dw.y + R2w12*p3Dw.z + t2wY;
        const float p3Dc2Z = R2w20*p3Dw.x + R2w21*p3Dw.y + R2w22*p3Dw.z + t2wZ;

        const float p3Dc1X = sR12_00*p3Dc2X + sR12_01*p3Dc2Y + sR12_02*p3Dc2Z + t12X;
        const float p3Dc1Y = sR12_10*p3Dc2X + sR12_11*p3Dc2Y + sR12_12*p3Dc2Z + t12Y;
        const float p3Dc1Z = sR12_20*p3Dc2X + sR12_21*p3Dc2Y + sR12_22*p3Dc2Z + t12Z;

        // 深度必须为正
        if(p3Dc1Z<0.0f)
            continue;

        const float invz = 1.0f/p3Dc1Z;
        const float x = p3Dc1X*invz;
        const float y = p3Dc1Y*invz;

        const float u = fx*x+cx;
        const float v = fy*y+cy;

        // 点必须在图像内部
        if(!pKF1->IsInImage(u,v))
            continue;

        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();

        // 使用平方距离进行快速范围检查
        const float dist3DSq = p3Dc1X*p3Dc1X + p3Dc1Y*p3Dc1Y + p3Dc1Z*p3Dc1Z;
        const float maxDistSq = maxDistance * maxDistance;
        const float minDistSq = minDistance * minDistance;

        // 深度必须在图像的尺度金字塔内
        if(dist3DSq < minDistSq || dist3DSq > maxDistSq)
            continue;

        // 只在需要时计算实际距离
        const float dist3D = sqrt(dist3DSq);

        // 计算预测的八度
        const int nPredictedLevel = pMP->PredictScale(dist3D,pKF1);

        // 在 2.5*sigma(尺度层级) 半径内搜索
        const float radius = th*pKF1->mvScaleFactors[nPredictedLevel];

        const vector<size_t> vIndices = pKF1->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // 匹配半径内最相似的关键点
        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = INT_MAX;
        int bestIdx = -1;
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            const cv::KeyPoint &kp = pKF1->mvKeysUn[idx];

            if(kp.octave<nPredictedLevel-1 || kp.octave>nPredictedLevel)
                continue;

            const cv::Mat &dKF = pKF1->mDescriptors.row(idx);

            const int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        if(bestDist<=TH_HIGH)
        {
            vnMatch2[i2]=bestIdx;
        }
    }

    // 检查一致性
    int nFound = 0;

    for(int i1=0; i1<N1; i1++)
    {
        int idx2 = vnMatch1[i1];

        if(idx2>=0)
        {
            int idx1 = vnMatch2[idx2];
            if(idx1==i1)
            {
                vpMatches12[i1] = vpMapPoints2[idx2];
                nFound++;
            }
        }
    }

    return nFound;
}

int ORBmatcher::SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono)
{
    int nmatches = 0;

    // 旋转直方图（用于检查旋转一致性）
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0,3).col(3);

    // 将矩阵数据在循环外部加载至栈上，消除循环内部 of Mat 访问开销
    const float R00 = Rcw.at<float>(0,0), R01 = Rcw.at<float>(0,1), R02 = Rcw.at<float>(0,2);
    const float R10 = Rcw.at<float>(1,0), R11 = Rcw.at<float>(1,1), R12 = Rcw.at<float>(1,2);
    const float R20 = Rcw.at<float>(2,0), R21 = Rcw.at<float>(2,1), R22 = Rcw.at<float>(2,2);
    const float tx = tcw.at<float>(0), ty = tcw.at<float>(1), tz = tcw.at<float>(2);

    const cv::Mat twc = -Rcw.t()*tcw;

    const cv::Mat Rlw = LastFrame.mTcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tlw = LastFrame.mTcw.rowRange(0,3).col(3);

    const cv::Mat tlc = Rlw*twc+tlw;

    const bool bForward = tlc.at<float>(2)>CurrentFrame.mb && !bMono;
    const bool bBackward = -tlc.at<float>(2)>CurrentFrame.mb && !bMono;

    for(int i=0; i<LastFrame.N; i++)
    {
        MapPoint* pMP = LastFrame.mvpMapPoints[i];

        if(pMP)
        {
            if(!LastFrame.mvbOutlier[i])
            {
                // 投影 (使用 Point3f 替代 cv::Mat，消除堆分配)
                cv::Point3f x3Dw;
                pMP->GetWorldPos(x3Dw);
                const float xc = R00*x3Dw.x + R01*x3Dw.y + R02*x3Dw.z + tx;
                const float yc = R10*x3Dw.x + R11*x3Dw.y + R12*x3Dw.z + ty;
                const float zc = R20*x3Dw.x + R21*x3Dw.y + R22*x3Dw.z + tz;

                if(zc<=0.0f)
                    continue;

                const float invzc = 1.0f/zc;

                float u = CurrentFrame.fx*xc*invzc+CurrentFrame.cx;
                float v = CurrentFrame.fy*yc*invzc+CurrentFrame.cy;

                if(u<CurrentFrame.mnMinX || u>CurrentFrame.mnMaxX)
                    continue;
                if(v<CurrentFrame.mnMinY || v>CurrentFrame.mnMaxY)
                    continue;

                int nLastOctave = LastFrame.mvKeys[i].octave;

                // 在窗口中搜索。尺寸取决于尺度
                float radius = th*CurrentFrame.mvScaleFactors[nLastOctave];

                vector<size_t> vIndices2;

                if(bForward)
                    vIndices2 = CurrentFrame.GetFeaturesInArea(u,v, radius, nLastOctave);
                else if(bBackward)
                    vIndices2 = CurrentFrame.GetFeaturesInArea(u,v, radius, 0, nLastOctave);
                else
                    vIndices2 = CurrentFrame.GetFeaturesInArea(u,v, radius, nLastOctave-1, nLastOctave+1);

                if(vIndices2.empty())
                    continue;

                const cv::Mat dMP = pMP->GetDescriptor();

                int bestDist = 256;
                int bestIdx2 = -1;

                for(vector<size_t>::const_iterator vit=vIndices2.begin(), vend=vIndices2.end(); vit!=vend; vit++)
                {
                    const size_t i2 = *vit;

                    const cv::Mat &d = CurrentFrame.mDescriptors.row(i2);

                    const int dist = DescriptorDistance(dMP,d);

                    if(dist<bestDist)
                    {
                        bestDist=dist;
                        bestIdx2=i2;
                    }
                }

                if(bestDist<=TH_HIGH)
                {
                    CurrentFrame.mvpMapPoints[bestIdx2]=pMP;
                    nmatches++;

                    if(mbCheckOrientation)
                    {
                        float rot = LastFrame.mvKeysUn[i].angle-CurrentFrame.mvKeysUn[bestIdx2].angle;
                        if(rot<0.0)
                            rot+=360.0f;
                        int bin = round(rot*factor);
                        if(bin==HISTO_LENGTH)
                            bin=0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(bestIdx2);
                    }
                }
            }
        }
    }

    // 应用旋转一致性
    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i!=ind1 && i!=ind2 && i!=ind3)
            {
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {
                    CurrentFrame.mvpMapPoints[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
                    nmatches--;
                }
            }
        }
    }

    return nmatches;
}

int ORBmatcher::SearchByProjection(Frame &CurrentFrame, KeyFrame *pKF, const set<MapPoint*> &sAlreadyFound, const float th , const int ORBdist)
{
    int nmatches = 0;

    const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0,3).col(3);
    const cv::Mat Ow = -Rcw.t()*tcw;

    // 将矩阵数据在循环外部加载至栈上，消除循环内部 of Mat 访问开销
    const float R00 = Rcw.at<float>(0,0), R01 = Rcw.at<float>(0,1), R02 = Rcw.at<float>(0,2);
    const float R10 = Rcw.at<float>(1,0), R11 = Rcw.at<float>(1,1), R12 = Rcw.at<float>(1,2);
    const float R20 = Rcw.at<float>(2,0), R21 = Rcw.at<float>(2,1), R22 = Rcw.at<float>(2,2);
    const float tx = tcw.at<float>(0), ty = tcw.at<float>(1), tz = tcw.at<float>(2);
    const float Ox = Ow.at<float>(0), Oy = Ow.at<float>(1), Oz = Ow.at<float>(2);

    // 旋转直方图（用于检查旋转一致性）
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

    for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
    {
        MapPoint* pMP = vpMPs[i];

        if(pMP)
        {
            if(!pMP->isBad() && !sAlreadyFound.count(pMP))
            {
                // 投影 (使用 Point3f 替代 cv::Mat，消除堆分配)
                cv::Point3f x3Dw;
                pMP->GetWorldPos(x3Dw);
                const float xc = R00*x3Dw.x + R01*x3Dw.y + R02*x3Dw.z + tx;
                const float yc = R10*x3Dw.x + R11*x3Dw.y + R12*x3Dw.z + ty;
                const float zc = R20*x3Dw.x + R21*x3Dw.y + R22*x3Dw.z + tz;

                if(zc<=0.0f)
                    continue;

                const float invzc = 1.0f/zc;
                const float u = CurrentFrame.fx*xc*invzc+CurrentFrame.cx;
                const float v = CurrentFrame.fy*yc*invzc+CurrentFrame.cy;

                if(u<CurrentFrame.mnMinX || u>CurrentFrame.mnMaxX)
                    continue;
                if(v<CurrentFrame.mnMinY || v>CurrentFrame.mnMaxY)
                    continue;

                // 计算预测的尺度层级 (标量减法与乘加)
                const float POx = x3Dw.x - Ox;
                const float POy = x3Dw.y - Oy;
                const float POz = x3Dw.z - Oz;
                const float dist3DSq = POx*POx + POy*POy + POz*POz;

                const float maxDistance = pMP->GetMaxDistanceInvariance();
                const float minDistance = pMP->GetMinDistanceInvariance();
                const float maxDistSq = maxDistance * maxDistance;
                const float minDistSq = minDistance * minDistance;

                // 深度必须在图像的尺度金字塔内
                if(dist3DSq < minDistSq || dist3DSq > maxDistSq)
                    continue;

                // 只在需要时计算实际距离
                float dist3D = sqrt(dist3DSq);

                int nPredictedLevel = pMP->PredictScale(dist3D,&CurrentFrame);

                // 在窗口中搜索
                const float radius = th*CurrentFrame.mvScaleFactors[nPredictedLevel];

                const vector<size_t> vIndices2 = CurrentFrame.GetFeaturesInArea(u, v, radius, nPredictedLevel-1, nPredictedLevel+1);

                if(vIndices2.empty())
                    continue;

                const cv::Mat dMP = pMP->GetDescriptor();

                int bestDist = 256;
                int bestIdx2 = -1;

                for(vector<size_t>::const_iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
                {
                    const size_t i2 = *vit;
                    if(CurrentFrame.mvpMapPoints[i2])
                        continue;

                    const cv::Mat &d = CurrentFrame.mDescriptors.row(i2);

                    const int dist = DescriptorDistance(dMP,d);

                    if(dist<bestDist)
                    {
                        bestDist=dist;
                        bestIdx2=i2;
                    }
                }

                if(bestDist<=ORBdist)
                {
                    CurrentFrame.mvpMapPoints[bestIdx2]=pMP;
                    nmatches++;

                    if(mbCheckOrientation)
                    {
                        float rot = pKF->mvKeysUn[i].angle-CurrentFrame.mvKeysUn[bestIdx2].angle;
                        if(rot<0.0)
                            rot+=360.0f;
                        int bin = round(rot*factor);
                        if(bin==HISTO_LENGTH)
                            bin=0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(bestIdx2);
                    }
                }

            }
        }
    }

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i!=ind1 && i!=ind2 && i!=ind3)
            {
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {
                    CurrentFrame.mvpMapPoints[rotHist[i][j]]=NULL;
                    nmatches--;
                }
            }
        }
    }

    return nmatches;
}

void ORBmatcher::ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3)
{
    int max1=0;
    int max2=0;
    int max3=0;

    for(int i=0; i<L; i++)
    {
        const int s = histo[i].size();
        if(s>max1)
        {
            max3=max2;
            max2=max1;
            max1=s;
            ind3=ind2;
            ind2=ind1;
            ind1=i;
        }
        else if(s>max2)
        {
            max3=max2;
            max2=s;
            ind3=ind2;
            ind2=i;
        }
        else if(s>max3)
        {
            max3=s;
            ind3=i;
        }
    }

    // 使用整数乘法代替浮点乘法
    if(max2*10 < max1)
    {
        ind2=-1;
        ind3=-1;
    }
    else if(max3*10 < max1)
    {
        ind3=-1;
    }
}

// 位集计数来自 bithacks (CountBitsSetParallel)
int ORBmatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const uint8_t* pa = a.ptr<uint8_t>();
    const uint8_t* pb = b.ptr<uint8_t>();

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
    // ARM NEON 路径: vcntq_u8 + vaddlvq_u8
    // 单指令 16 字节 popcount, 2 次加载覆盖全部 32 字节
    const uint8x16_t va0 = vld1q_u8(pa);
    const uint8x16_t vb0 = vld1q_u8(pb);
    const uint8x16_t va1 = vld1q_u8(pa + 16);
    const uint8x16_t vb1 = vld1q_u8(pb + 16);

    const uint8x16_t xor0 = veorq_u8(va0, vb0);
    const uint8x16_t xor1 = veorq_u8(va1, vb1);

    // vcntq_u8: 每条指令对 16 个字节同时计算 popcount
    // vaddlvq_u8: 将 16 字节的 popcount 水平累加为单个 u32
    const uint8x16_t pop0 = vcntq_u8(xor0);
    const uint8x16_t pop1 = vcntq_u8(xor1);

    // vaddlvq_u8 是 AArch64 特有指令，Armv7 回退到 vpaddlq
    #if defined(__aarch64__)
        return (int)(vaddlvq_u8(pop0) + vaddlvq_u8(pop1));
    #else
        // Armv7 NEON: 使用 vpaddlq 逐步归约
        uint16x8_t sum16 = vpaddlq_u8(vaddq_u8(pop0, pop1));
        uint32x4_t sum32 = vpaddlq_u16(sum16);
        uint64x2_t sum64 = vpaddlq_u32(sum32);
        return (int)(vgetq_lane_u64(sum64, 0) + vgetq_lane_u64(sum64, 1));
    #endif
#else
    // 标量回退: 4 × 64 位 popcount (与之前相同)
    uint64_t va[4], vb[4];
    std::memcpy(va, pa, 32);
    std::memcpy(vb, pb, 32);

    return __builtin_popcountll(va[0] ^ vb[0]) +
           __builtin_popcountll(va[1] ^ vb[1]) +
           __builtin_popcountll(va[2] ^ vb[2]) +
           __builtin_popcountll(va[3] ^ vb[3]);
#endif
}

} //namespace ORB_SLAM2

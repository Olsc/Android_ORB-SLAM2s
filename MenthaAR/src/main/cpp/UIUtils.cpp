/**
 * Copyright (C) 2025-2026 Olsc <OlscStudio@outlook.com>
 *
 * This file is part of the Android ORB-SLAM2s project (a fork of ORB-SLAM2).
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/**
 * Created by Ads on 2017/1/15.
 * 由Olsc于2025/8/25开始进行修改
 */

// UI 工具函数模块实现
#include "UIUtils.h"
#include "Matrix.h"
#include "include/Config.h"
#include "include/Random.h"
#include <algorithm>
#include <cmath>

// 使用 RANSAC 从 3D 地图点中检测平面，选择中值距离最小的模型
Plane* detectPlane(const cv::Mat Tcw, const std::vector<ORB_SLAM2::MapPoint*> &vMPs, const int iterations)
{
    // 提取 3D 点：仅保留观测次数达阈值的稳定地图点
    vector<cv::Point3f> vPoints;
    vPoints.reserve(vMPs.size());
    vector<ORB_SLAM2::MapPoint*> vPointMP;
    vPointMP.reserve(vMPs.size());

    for(size_t i=0; i<vMPs.size(); i++)
    {
        ORB_SLAM2::MapPoint* pMP=vMPs[i];
        if(pMP)
        {
            if(pMP->Observations()>ORB_SLAM2::PLANE_MIN_OBSERVATIONS)  // 过滤观测次数少的不稳定点
            {
                cv::Point3f Pw;
                pMP->GetWorldPos(Pw);
                vPoints.push_back(Pw);
                vPointMP.push_back(pMP);
            }
        }
    }

    const int N = vPoints.size();

    if(N<ORB_SLAM2::PLANE_MIN_POINTS) // 点数过少，无法可靠地拟合平面
        return nullptr;

    // 准备 RANSAC 所需的索引数组
    vector<size_t> vAllIndices;
    vAllIndices.reserve(N);

    for(int i=0; i<N; i++)
    {
        vAllIndices.push_back(i);
    }

    float bestDist = 1e10;
    vector<float> bestvDist;
    vector<float> vDistances(N);
    vector<float> vSorted(N);

    // RANSAC 迭代：寻找最佳平面模型。随机源用与引擎相同的 LCG（rand() 是全局状态且非线程安全）
    ORB_SLAM2::LCG lcg(0);
    for(int n=0; n<iterations; n++)
    {
        // 随机选择 3 个点拟合平面
        size_t idx[3];
        ORB_SLAM2::SampleWithoutReplacement(lcg, vAllIndices, 3, idx);

        // 三点定面闭式解，共线退化时范数趋零直接拒绝
        const cv::Point3f &p1 = vPoints[idx[0]];
        const cv::Point3f &p2 = vPoints[idx[1]];
        const cv::Point3f &p3 = vPoints[idx[2]];
        const float ux = p2.x - p1.x, uy = p2.y - p1.y, uz = p2.z - p1.z;
        const float vx = p3.x - p1.x, vy = p3.y - p1.y, vz = p3.z - p1.z;
        float a = uy*vz - uz*vy;
        float b = uz*vx - ux*vz;
        float c = ux*vy - uy*vx;
        const float nrm = std::sqrt(a*a + b*b + c*c);
        if(nrm < 1e-8f)
            continue; // 三点近共线，无法定义平面
        const float invNrm = 1.0f / nrm;
        a *= invNrm; b *= invNrm; c *= invNrm;
        const float d = -(a*p1.x + b*p1.y + c*p1.z);

        // 法向量 (a,b,c) 已是单位向量，点到平面的真实欧氏距离直接为 |a*x + b*y + c*z + d|
        for(int i=0; i<N; i++)
        {
            vDistances[i] = std::fabs(vPoints[i].x*a + vPoints[i].y*b + vPoints[i].z*c + d);
        }

        // 计算截尾中值距离评估拟合优度
        std::copy(vDistances.begin(), vDistances.end(), vSorted.begin());
        int nth = max((int)(ORB_SLAM2::PLANE_MEDIAN_TAIL_RATIO*N), ORB_SLAM2::PLANE_MEDIAN_MIN_SAMPLES);
        if(nth >= (int)vSorted.size()) nth = (int)vSorted.size() - 1;
        std::nth_element(vSorted.begin(), vSorted.begin() + nth, vSorted.end());
        const float medianDist = vSorted[nth];

        // 保存中值距离最小的模型
        if(medianDist<bestDist)
        {
            bestDist = medianDist;
            bestvDist.swap(vDistances);
            if((int)vDistances.size()!=N) vDistances.resize(N);
        }
    }

    // 无有效模型时直接返回
    if(bestvDist.empty())
        return nullptr;

    // 内点阈值 = 最佳中值距离 * PLANE_INLIER_TH_RATIO
    const float th = ORB_SLAM2::PLANE_INLIER_TH_RATIO*bestDist;
    vector<bool> vbInliers(N,false);
    int nInliers = 0;
    for(int i=0; i<N; i++)
    {
        if(bestvDist[i]<th)
        {
            nInliers++;
            vbInliers[i]=true;
        }
    }

    // 提取内点，用于构建最终的平面
    vector<ORB_SLAM2::MapPoint*> vInlierMPs(nInliers,nullptr);
    int nin = 0;
    for(int i=0; i<N; i++)
    {
        if(vbInliers[i])
        {
            vInlierMPs[nin] = vPointMP[i];
            nin++;
        }
    }

    return new Plane(vInlierMPs,Tcw);
}

// 带重力先验的水平面检测
static Plane* detectHorizontalPlaneOnce(const cv::Mat &Tcw, const std::vector<ORB_SLAM2::MapPoint*> &vMPs,
                                        const cv::Mat &upWorld, const float tolDepthRatio)
{
    // 水平面只需较少的稳定点即可：高度一维聚类对离群点不敏感，观测门槛可比纯视觉 RANSAC 放宽
    const int kMinObservations = 3;
    const int kMinPoints = 20;
    const int kMinInliers = 12;
    const float kTolDepthRatio = tolDepthRatio; // 高度容差占中值深度的比例(单目尺度未知，用场景自身尺度归一)
    const float kLowestClusterRatio = 0.5f; // 取点数不低于最大簇 60% 的簇中最低的一个，用于排除桌面

    if (upWorld.empty() || Tcw.empty() || Tcw.rows < 3 || Tcw.cols < 4)
        return nullptr;

    const float ux = upWorld.at<float>(0), uy = upWorld.at<float>(1), uz = upWorld.at<float>(2);

    // 相机中心 C = -Rcw^T * tcw，用于尺度归一与"相机在平面上方"的合法性检验
    const float* T = Tcw.ptr<float>();
    const int stride = static_cast<int>(Tcw.step / sizeof(float));
    auto Tv = [&](int r, int c) { return T[r * stride + c]; };
    const float cwx = -(Tv(0,0)*Tv(0,3) + Tv(1,0)*Tv(1,3) + Tv(2,0)*Tv(2,3));
    const float cwy = -(Tv(0,1)*Tv(0,3) + Tv(1,1)*Tv(1,3) + Tv(2,1)*Tv(2,3));
    const float cwz = -(Tv(0,2)*Tv(0,3) + Tv(1,2)*Tv(1,3) + Tv(2,2)*Tv(2,3));

    struct Item { float h; ORB_SLAM2::MapPoint* mp; };
    std::vector<Item> items;
    std::vector<float> depths;
    items.reserve(vMPs.size());
    depths.reserve(vMPs.size());
    for (size_t i = 0; i < vMPs.size(); i++)
    {
        ORB_SLAM2::MapPoint* pMP = vMPs[i];
        if (!pMP || pMP->isBad() || pMP->Observations() < kMinObservations)
            continue;
        cv::Point3f Pw;
        pMP->GetWorldPos(Pw);
        items.push_back({Pw.x*ux + Pw.y*uy + Pw.z*uz, pMP});
        const float dx = Pw.x - cwx, dy = Pw.y - cwy, dz = Pw.z - cwz;
        depths.push_back(std::sqrt(dx*dx + dy*dy + dz*dz));
    }

    const int N = static_cast<int>(items.size());
    if (N < kMinPoints)
        return nullptr;

    std::vector<float> dSorted(depths);
    std::nth_element(dSorted.begin(), dSorted.begin() + N/2, dSorted.end());
    const float tol = kTolDepthRatio * dSorted[N/2];
    if (!(tol > 1e-6f))
        return nullptr;

    std::sort(items.begin(), items.end(), [](const Item& a, const Item& b){ return a.h < b.h; });

    // 双指针统计每个起点高度开始、宽度 2*tol 的窗口内点数
    std::vector<int> cnt(N, 0);
    int maxCnt = 0;
    int j = 0;
    for (int i = 0; i < N; i++)
    {
        if (j < i) j = i;
        while (j < N && items[j].h - items[i].h <= 2.0f * tol) j++;
        cnt[i] = j - i;
        if (cnt[i] > maxCnt) maxCnt = cnt[i];
    }

    const int needInliers = std::max(kMinInliers, static_cast<int>(0.15f * N));
    if (maxCnt < needInliers)
        return nullptr;

    // 最低的满足门槛的簇
    const int thresh = std::max(needInliers, static_cast<int>(kLowestClusterRatio * maxCnt));
    int start = -1;
    for (int i = 0; i < N; i++)
    {
        if (cnt[i] >= thresh) { start = i; break; }
    }
    if (start < 0)
        return nullptr;

    // 以窗口均值为中心细化一次，取均值 ±tol 内的点作为内点
    double sumH = 0.0;
    for (int i = start; i < start + cnt[start]; i++) sumH += items[i].h;
    const float meanH = static_cast<float>(sumH / cnt[start]);

    std::vector<ORB_SLAM2::MapPoint*> vInlierMPs;
    vInlierMPs.reserve(cnt[start]);
    for (int i = 0; i < N; i++)
    {
        if (std::fabs(items[i].h - meanH) <= tol)
            vInlierMPs.push_back(items[i].mp);
    }
    if (static_cast<int>(vInlierMPs.size()) < kMinInliers)
        return nullptr;

    // 合法性：相机必须在平面上方。重力方向换算有误(如安装角符号错误)时拒绝并回退，不会把物体放到错误位置
    const float camH = cwx*ux + cwy*uy + cwz*uz;
    if (camH - meanH <= tol)
        return nullptr;

    return new Plane(vInlierMPs, Tcw, upWorld);
}

// 由严到宽依次尝试几档高度容差：远处地图点深度噪声较大，单一容差下地面簇容易被拆散而放弃重力约束
Plane* detectHorizontalPlane(const cv::Mat Tcw, const std::vector<ORB_SLAM2::MapPoint*> &vMPs, const cv::Mat &upWorld)
{
    static const float kTolRatios[] = {0.03f, 0.05f, 0.08f};
    for (float r : kTolRatios)
    {
        Plane* p = detectHorizontalPlaneOnce(Tcw, vMPs, upWorld, r);
        if (p) return p;
    }
    return nullptr;
}

// 将 OpenCV 的 4x4 Mat 转为 OpenGL 列主序矩阵（需转置）
void getColMajorMatrixFromMat(float M[],cv::Mat &Tcw){
    M[0] = Tcw.at<float>(0,0);
    M[1] = Tcw.at<float>(1,0);
    M[2] = Tcw.at<float>(2,0);
    M[3]  = 0.0;
    M[4] = Tcw.at<float>(0,1);
    M[5] = Tcw.at<float>(1,1);
    M[6] = Tcw.at<float>(2,1);
    M[7]  = 0.0;
    M[8] = Tcw.at<float>(0,2);
    M[9] = Tcw.at<float>(1,2);
    M[10] = Tcw.at<float>(2,2);
    M[11]  = 0.0;
    M[12] = Tcw.at<float>(0,3);
    M[13] = Tcw.at<float>(1,3);
    M[14] = Tcw.at<float>(2,3);
    M[15]  = 1.0;
}
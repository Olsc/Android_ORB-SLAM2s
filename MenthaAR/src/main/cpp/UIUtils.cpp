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
    vector<size_t> vAvailableIndices;

    for(int i=0; i<N; i++)
    {
        vAllIndices.push_back(i);
    }

    float bestDist = 1e10;
    vector<float> bestvDist;
    vector<float> vDistances(N);
    vector<float> vSorted(N);

    // RANSAC 迭代：寻找最佳平面模型
    for(int n=0; n<iterations; n++)
    {
        vAvailableIndices = vAllIndices;

        // 随机选择 3 个点拟合平面
        int idx[3];
        for(short i = 0; i < 3; ++i)
        {
            int randi = rand() % vAvailableIndices.size();
            idx[i] = vAvailableIndices[randi];

            // 移除已选点，避免重复
            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

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
            bestvDist = vDistances;
        }
    }

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
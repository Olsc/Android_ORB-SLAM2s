/**
 * Created by Ads on 2017/1/15.
 * 由Olsc于2025/8/25开始进行修改
 */

// UI工具函数模块实现
#include "UIUtils.h"
#include "Matrix.h"
#include "include/Config.h"

// 使用RANSAC算法从3D地图点中检测平面，选择中值距离最小的模型
Plane* detectPlane(const cv::Mat Tcw, const std::vector<ORB_SLAM2::MapPoint*> &vMPs, const int iterations)
{
    // 提取3D点：仅保留观测次数达到阈值的稳定地图点，用 reserve 预分配容量
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

    if(N<ORB_SLAM2::PLANE_MIN_POINTS)  // 点数过少，无法可靠地拟合平面
        return NULL;

    // 准备RANSAC所需的索引数组
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

    // RANSAC迭代: 寻找最佳平面模型
    for(int n=0; n<iterations; n++)
    {
        vAvailableIndices = vAllIndices;

        // 随机选择3个点作为最小集合来拟合平面
        int idx[3];
        for(short i = 0; i < 3; ++i)
        {
            int randi = rand() % vAvailableIndices.size();
            idx[i] = vAvailableIndices[randi];

            // 移除已选点，避免重复
            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        // 三点定面闭式解：叉积法向归一化后与 SVD 零空间解一致，
        // 共线退化时范数趋零直接拒绝，语义与原 SVD 路径一致
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
            continue;   // 三点近共线，无法定义平面
        const float invNrm = 1.0f / nrm;
        a *= invNrm; b *= invNrm; c *= invNrm;
        const float d = -(a*p1.x + b*p1.y + c*p1.z);

        // 法向量 (a,b,c) 已是单位向量，点到平面的真实欧氏距离直接为 |a*x + b*y + c*z + d|
        for(int i=0; i<N; i++)
        {
            vDistances[i] = std::fabs(vPoints[i].x*a + vPoints[i].y*b + vPoints[i].z*c + d);
        }

        // 计算中值距离（取前20%的点的边界值）
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

    // 使用1.4倍最佳距离作为内点阈值
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
    vector<ORB_SLAM2::MapPoint*> vInlierMPs(nInliers,NULL);
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

// 将OpenCV的4x4 Mat矩阵转为OpenGL列主序矩阵，OpenCV行主序与OpenGL列主序需转置
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
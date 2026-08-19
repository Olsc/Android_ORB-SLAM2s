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
    // 提取3D点：仅保留观测次数>5的稳定地图点 (使用栈分配和零拷贝接口，彻底消除堆开销)
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

    // RANSAC迭代: 寻找最佳平面模型
    for(int n=0; n<iterations; n++)
    {
        vAvailableIndices = vAllIndices;

        cv::Mat A(3,4,CV_32F);
        A.col(3) = cv::Mat::ones(3,1,CV_32F);

        // 随机选择3个点作为最小集合来拟合平面
        for(short i = 0; i < 3; ++i)
        {
            int randi = rand() % vAvailableIndices.size();

            int idx = vAvailableIndices[randi];

            A.at<float>(i,0) = vPoints[idx].x;
            A.at<float>(i,1) = vPoints[idx].y;
            A.at<float>(i,2) = vPoints[idx].z;

            // 移除已选点，避免重复
            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        // 使用SVD求解平面方程 ax+by+cz+d=0
        cv::Mat u,w,vt;
        cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

        const float a = vt.at<float>(3,0);
        const float b = vt.at<float>(3,1);
        const float c = vt.at<float>(3,2);
        const float d = vt.at<float>(3,3);

        vector<float> vDistances(N,0);

        const float f = 1.0f/sqrt(a*a+b*b+c*c+d*d);  // 归一化系数

        // 计算所有点到平面的距离
        for(int i=0; i<N; i++)
        {
            vDistances[i] = fabs(vPoints[i].x*a + vPoints[i].y*b + vPoints[i].z*c + d)*f;
        }

        // 计算中值距离（取前20%的点的边界值）
        vector<float> vSorted = vDistances;
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
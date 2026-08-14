/**
 * Created by Ads on 2017/1/15.
 * 由Olsc于2025/8/25开始进行修改
 */

// UI工具函数模块实现
#include "UIUtils.h"
#include "Matrix.h"
#include "include/Config.h"

// 绘制当前帧跟踪到的特征点，青色=新建、绿色=匹配的已加载、红色=未匹配的已加载
void drawTrackedPoints(const std::vector<cv::KeyPoint> &vKeys, const std::vector<ORB_SLAM2::MapPoint *> &vMPs,
                       cv::Mat &im, float cx, float cy)
{
    // 计算从SLAM分辨率到显示分辨率的缩放因子
    float scaleX = ORB_SLAM2::IMAGE_DOWNSCALE_FACTOR, scaleY = ORB_SLAM2::IMAGE_DOWNSCALE_FACTOR;
    if (cx > 0.0f && cy > 0.0f) {
        scaleX = (float)im.cols / (ORB_SLAM2::IMAGE_DOWNSCALE_FACTOR * cx);
        scaleY = (float)im.rows / (ORB_SLAM2::IMAGE_DOWNSCALE_FACTOR * cy);
    }

    const int N = (int)vKeys.size();
    for(int i=0; i<N; i++) {
        if(i>=vMPs.size()) break;
        ORB_SLAM2::MapPoint* pMP = vMPs[i];
        if(pMP) {
            // 根据地图点来源和匹配状态选择颜色
            cv::Scalar color = cv::Scalar(ORB_SLAM2::UI_COLOR_NEW_POINT_B, ORB_SLAM2::UI_COLOR_NEW_POINT_G, ORB_SLAM2::UI_COLOR_NEW_POINT_R);  // 默认：新建点为青色
            if(pMP->mbFromLoadedMap){
                color = cv::Scalar(ORB_SLAM2::UI_COLOR_LOADED_POINT_B, ORB_SLAM2::UI_COLOR_LOADED_POINT_G, ORB_SLAM2::UI_COLOR_LOADED_POINT_R);  // 已加载地图点统一绿色（所有加载点在本帧均为内点，无需红色分支）
            }
            // 将关键点从SLAM分辨率缩放到显示分辨率
            cv::circle(im, cv::Point2f(vKeys[i].pt.x * scaleX, vKeys[i].pt.y * scaleY), ORB_SLAM2::UI_POINT_RADIUS, color, -1);
        }
    }
}

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

        // 计算中值距离（取前20%的点的边界值；nth_element 替代全排序）
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

// 绘制所有地图点，用于AR重定位时显示完整点云
void drawAllMapPoints(const cv::Mat &Tcw, const std::vector<ORB_SLAM2::MapPoint*> &allMapPoints,
                      cv::Mat &im, float fx, float fy, float cx, float cy, bool drawOnlyLoaded)
{
    if(Tcw.empty() || allMapPoints.empty())
        return;

    // 确保Tcw是有效的位姿矩阵
    if(Tcw.rows < 3 || Tcw.cols < 4)
        return;

    // 提取旋转矩阵和平移向量（一次性完成，避免重复at<>操作）
    const float R11 = Tcw.at<float>(0,0), R12 = Tcw.at<float>(0,1), R13 = Tcw.at<float>(0,2);
    const float R21 = Tcw.at<float>(1,0), R22 = Tcw.at<float>(1,1), R23 = Tcw.at<float>(1,2);
    const float R31 = Tcw.at<float>(2,0), R32 = Tcw.at<float>(2,1), R33 = Tcw.at<float>(2,2);
    const float tx = Tcw.at<float>(0,3);
    const float ty = Tcw.at<float>(1,3);
    const float tz = Tcw.at<float>(2,3);

    const int imgWidth = im.cols;
    const int imgHeight = im.rows;

    int drawnCount = 0;
    const int maxDrawPoints = ORB_SLAM2::UI_MAX_DRAWN_POINTS;  // 性能保护：限制最大绘制点数

    // 计算从SLAM分辨率到显示分辨率的动态缩放因子
    const float dispScaleX = (ORB_SLAM2::IMAGE_DOWNSCALE_FACTOR * cx > 0.0f) ? (float)im.cols / (ORB_SLAM2::IMAGE_DOWNSCALE_FACTOR * cx) : ORB_SLAM2::IMAGE_DOWNSCALE_FACTOR;
    const float dispScaleY = (ORB_SLAM2::IMAGE_DOWNSCALE_FACTOR * cy > 0.0f) ? (float)im.rows / (ORB_SLAM2::IMAGE_DOWNSCALE_FACTOR * cy) : ORB_SLAM2::IMAGE_DOWNSCALE_FACTOR;
    const float fx2 = fx * dispScaleX;
    const float fy2 = fy * dispScaleY;
    const float cx2 = cx * dispScaleX;
    const float cy2 = cy * dispScaleY;

    const cv::Scalar colorLoaded(ORB_SLAM2::UI_COLOR_LOADED_POINT_B, ORB_SLAM2::UI_COLOR_LOADED_POINT_G, ORB_SLAM2::UI_COLOR_LOADED_POINT_R);   // 绿色：已加载点
    const cv::Scalar colorNew(ORB_SLAM2::UI_COLOR_NEW_POINT_B, ORB_SLAM2::UI_COLOR_NEW_POINT_G, ORB_SLAM2::UI_COLOR_NEW_POINT_R);   // 青色：在线扫描建图点

    for(size_t i = 0; i < allMapPoints.size(); i++)
    {
        ORB_SLAM2::MapPoint* pMP = allMapPoints[i];

        // 快速过滤无效点
        if(!pMP || pMP->isBad())
            continue;

        if(drawOnlyLoaded && !pMP->mbFromLoadedMap)
            continue;

        // 获取3D世界坐标 (使用栈分配和零拷贝接口，彻底消除堆开销)
        cv::Point3f Pw;
        pMP->GetWorldPos(Pw);

        const float Xw = Pw.x;
        const float Yw = Pw.y;
        const float Zw = Pw.z;

        // 世界坐标转相机坐标（手动矩阵乘法，避免OpenCV函数调用开销）
        const float Xc = R11*Xw + R12*Yw + R13*Zw + tx;
        const float Yc = R21*Xw + R22*Yw + R23*Zw + ty;
        const float Zc = R31*Xw + R32*Yw + R33*Zw + tz;

        // 深度检查（早期剔除策略）
        if(Zc <= ORB_SLAM2::PROJECT_MIN_DEPTH)  // 点在相机后方或过近
            continue;

        // 投影到图像平面
        const float invZ = 1.0f / Zc;
        const float u_display = fx2 * Xc * invZ + cx2;
        const float v_display = fy2 * Yc * invZ + cy2;

        // 边界检查（早期剔除策略）
        if(u_display < 0 || u_display >= imgWidth || v_display < 0 || v_display >= imgHeight)
            continue;

        // 绘制点
        cv::circle(im, cv::Point2f(u_display, v_display), ORB_SLAM2::UI_CLOUD_POINT_RADIUS,
                   pMP->mbFromLoadedMap ? colorLoaded : colorNew, -1);

        drawnCount++;
        if(drawnCount >= maxDrawPoints)
            break;  // 达到最大点数限制，保证实时性
    }
}
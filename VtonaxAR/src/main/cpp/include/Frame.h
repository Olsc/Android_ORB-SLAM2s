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

#ifndef FRAME_H
#define FRAME_H

#include<vector>

#include "MapPoint.h"
#include "HBSTTypes.h"
#include "KeyFrame.h"
#include "ORBextractor.h"
#include "Config.h"

#include <opencv2/opencv.hpp>

namespace ORB_SLAM2
{
//#define FRAME_GRID_ROWS 48
//#define FRAME_GRID_COLS 64

class MapPoint;
class KeyFrame;

class Frame
{
public:
    Frame();

    // 复制构造函数。
    Frame(const Frame &frame);

    // 单目相机的构造函数。
    Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth = 0.0f);

    // 在图像上提取ORB。0表示左图像，1表示右图像。
    void ExtractORB(int flag, const cv::Mat &im);

    // 设置相机位姿。
    void SetPose(cv::Mat Tcw);

    // 从相机位姿计算旋转、平移和相机中心矩阵。
    void UpdatePoseMatrices();

    // 返回相机中心。
    inline cv::Mat GetCameraCenter(){
        return mOw.clone();
    }

    // 返回旋转的逆
    inline cv::Mat GetRotationInverse(){
        return mRwc.clone();
    }

    // 检查地图点是否在相机的视锥体内
    // 并填充地图点的变量以供跟踪使用
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

    // 计算关键点的单元格（如果在网格外则返回false）
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

public:

    // 特征提取器。
    ORBextractor* mpORBextractorLeft;

    // 帧时间戳。
    double mTimeStamp;

    // 校准矩阵和OpenCV失真参数。
    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;

    // 立体基线乘以fx。
    float mbf;

    // 立体基线（米）。
    float mb;

    // 关键点数量。
    int N;

    // 关键点向量（原始用于可视化）和去失真（实际由系统使用）。
    std::vector<cv::KeyPoint> mvKeys;
    std::vector<cv::KeyPoint> mvKeysUn;

    // ORB描述符，每行与一个关键点关联。
    cv::Mat mDescriptors;

    // HBST树缓存，用于快速匹配
    std::shared_ptr<HBSTTree> mpTree;
    std::shared_ptr<HBSTTree> GetHBSTTree();

    // 与关键点关联的地图点，如果没有关联则为NULL指针。
    std::vector<MapPoint*> mvpMapPoints;

    // 标识异常值关联的标志。
    std::vector<bool> mvbOutlier;

    // 关键点被分配到网格中的单元格以减少投影地图点时的匹配复杂度。
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // 相机位姿。
    cv::Mat mTcw;

    // 当前和下一帧id。
    static long unsigned int nNextId;
    long unsigned int mnId;

    // 参考关键帧。
    KeyFrame* mpReferenceKF;

    // 尺度金字塔信息。
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    std::vector<float> mvScaleFactors;
    std::vector<float> mvInvScaleFactors;
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;

    // 去失真图像边界（计算一次）。
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    static bool mbInitialComputations;


private:

    // 计算去失真图像的边界（在构造函数中调用）。
    void ComputeImageBounds(const cv::Mat &imLeft);

    // 将关键点分配给网格以加速特征匹配（在构造函数中调用）。
    void AssignFeaturesToGrid();

    // 旋转、平移和相机中心
    cv::Mat mRcw;
    cv::Mat mtcw;
    cv::Mat mRwc;
    cv::Mat mOw; //==mtwc
};

}// namespace ORB_SLAM2

#endif // FRAME_H

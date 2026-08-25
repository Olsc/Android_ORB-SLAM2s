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

#ifndef SYSTEM_H
#define SYSTEM_H

#include<string>
#include<thread>
#include<mutex>
#include<chrono>
#include<opencv2/core/core.hpp>

#include "Config.h"
#include "Tracking.h"
#include "FrameDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"

namespace ORB_SLAM2
{

class FrameDrawer;
class Map;
class Tracking;
class LocalMapping;
class LoopClosing;

// SLAM系统主类：协调跟踪、建图、闭环检测等模块的工作
class System
{
public:
    // 输入传感器类型枚举，当前实现仅支持单目相机
    enum eSensor{
        MONOCULAR=0  // 单目相机
    };

public:

    // 构造函数：初始化SLAM系统，启动局部建图、闭环检测与可视化线程
    System(const string &strSettingsFile, const eSensor sensor);

    // 析构函数：确保所有线程被 join、子模块与地图被释放（Shutdown 未被调用时兜底）
    ~System();

    // 处理单目图像帧：提取ORB特征、跟踪、局部地图跟踪并决定是否创建关键帧，返回相机位姿矩阵Tcw，跟踪失败时返回空矩阵
    cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp);

    // 动态更新相机内参，当分辨率改变时由JNI层调用
    void UpdateCalibration(float fx, float fy, float cx, float cy);

    // 检查地图是否发生重大变化（闭环或全局BA）
    bool MapChanged();

    // 重置SLAM系统，bKeepMap为true时保留地图仅清跟踪状态
    void Reset(bool bKeepMap = false);

    // 关闭系统：请求所有线程结束并等待安全退出，保存轨迹前必须调用
    void Shutdown();

    // 以TUM RGB-D格式保存关键帧轨迹，调用前需先调用Shutdown()
    void SaveKeyFrameTrajectoryTUM(const string &filename);

    // 获取当前定位到的地图ID
    int GetCurrentMapId();

    // 保存地图到文件，maxMapPoints 限制最大特征点数（超限时按时间裁剪保留最新）
    void SaveMap(const string &filename, int maxMapPoints = SYSTEM_MAX_MPS_SAVE);

    // 加载地图，mapId 为地图ID，bAppend 为追加模式
    void LoadMap(const string &filename, int mapId = 0, bool bAppend = false);

    // 获取地图中的关键帧数量
    int GetNumKeyFrames();

    // 获取地图中的地图点数量
    int GetNumMapPoints();

    // 获取所有地图点
    std::vector<MapPoint*> GetAllMapPoints();

    // 获取当前跟踪状态，可在TrackMonocular()后立即调用
    int GetTrackingState();

    // 获取当前帧跟踪到的地图点
    std::vector<MapPoint*> GetTrackedMapPoints();

    // 获取当前帧跟踪到的关键点（去畸变后）
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

    // 获取重定位对齐置信度，反映当前位姿与加载地图的对齐程度
    float GetRelocAlignConfidence();

    // 获取重定位匹配分数，不依赖PnP求解，可更早提示是否进入目标区域
    float GetRelocMatchScore();

    // 检查是否已建立地图对齐
    bool HasMapAlignment();

    // 获取对齐后的相机位姿，将SLAM坐标系位姿转换到加载地图坐标系
    cv::Mat GetMapAlignedPose(const cv::Mat &TcwSlam);

    // 检查是否已加载地图
    bool HasLoadedMap();

    // 创建新地图（子地图），用于跟踪丢失时的恢复
    void CreateNewMap();

    // 完整拆除旧子地图：跨图引用剥离、候选库摘除、全量释放
    void RetireSubmap(Map* pOldMap);

    // 切换到指定地图
    void SwitchToMap(Map* pMap);

//public:
private:
    // 多地图容器
    std::vector<Map*> mvpMaps;
    // 子地图 ID 单调计数器（初始地图 mnId=0，此后递增分配，
    // 避免逐出旧地图后 mvpMaps.size() 变小导致新地图 ID 与现存地图重复）
    unsigned long mnNextMapId = 1;

    // 传感器配置
    eSensor mSensor;  // 输入传感器类型

    // 用于位置识别的关键帧数据库（重定位和回环检测）。
    KeyFrameDatabase* mpKeyFrameDatabase;

    // 存储所有关键帧和地图点指针的地图结构。
    Map* mpMap;

    // 跟踪器：接收帧并计算相机位姿，决定何时插入关键帧、创建地图点，丢失时执行重定位。
    Tracking* mpTracker;

    // 局部建图器。它管理局部地图并执行局部束调整。
    LocalMapping* mpLocalMapper;

    // 回环闭合器。它搜索每个新关键帧的回环。如果存在回环，它会执行位姿图优化，然后执行完整的束调整（在新线程中）。
    LoopClosing* mpLoopCloser;

    FrameDrawer* mpFrameDrawer;

    // 系统线程：局部建图、回环闭合、可视化。
    // 跟踪线程"生活"在创建System对象的主执行线程中。
    std::thread* mptLocalMapping;
    std::thread* mptLoopClosing;

    // 重置标志
    std::mutex mMutexReset;
    bool mbReset;
    bool mbResetKeepMap;  // 重置时是否保留地图

    // 跟踪状态
    int mTrackingState;
    std::vector<MapPoint*> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    std::mutex mMutexState;
};

}// namespace ORB_SLAM2

#endif // SYSTEM_H
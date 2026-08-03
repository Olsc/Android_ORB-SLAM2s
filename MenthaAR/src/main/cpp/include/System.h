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

/**
 * SLAM系统主类
 * 协调跟踪、建图、闭环检测等模块的工作
 */
class System
{
public:
    /**
     * 输入传感器类型枚举
     * 当前实现仅支持单目相机
     */
    enum eSensor{
        MONOCULAR=0  // 单目相机
    };

public:

    /**
     * 构造函数：初始化SLAM系统
     * 启动局部建图、闭环检测和可视化线程
     * 
     * @param strVocFile ORB词汇表文件路径
     * @param strSettingsFile 相机配置文件路径（YAML格式）
     * @param sensor 传感器类型（当前仅支持MONOCULAR）
     */
    System(const string &strSettingsFile, const eSensor sensor);

    /**
     * 处理单目图像帧（主处理函数）
     * 
     * 处理流程：
     *   1. 提取ORB特征
     *   2. 跟踪上一帧或重定位
     *   3. 局部地图跟踪
     *   4. 决定是否创建关键帧
     * 
     * @param im 输入图像（支持RGB CV_8UC3或灰度CV_8U，RGB会自动转换为灰度）
     * @param timestamp 时间戳（秒）
     * @return 相机位姿矩阵Tcw（世界到相机的变换），跟踪失败时返回空矩阵
     */
    cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp);

    /**
     * 动态更新相机内参（当分辨率改变时由JNI层调用）
     * 
     * @param fx x轴焦距
     * @param fy y轴焦距
     * @param cx x轴中心点
     * @param cy y轴中心点
     */
    void UpdateCalibration(float fx, float fy, float cx, float cy);

    /**
     * 检查地图是否发生重大变化
     * 
     * @return true-自上次调用以来发生了闭环或全局BA，false-无重大变化
     */
    bool MapChanged();

    /**
     * 重置SLAM系统
     * 
     * @param bKeepMap true-仅清除跟踪状态但保留地图, false-完全重置（清空地图）
     */
    void Reset(bool bKeepMap = false);

    /**
     * 关闭系统
     * 请求所有线程结束并等待它们安全退出
     * 注意：在保存轨迹之前必须调用此函数
     */
    void Shutdown();

    /**
     * 保存关键帧轨迹
     * 以TUM RGB-D数据集格式保存（时间戳 x y z qx qy qz qw）
     * 
     * 注意：调用前需先调用Shutdown()
     * 格式详情: http://vision.in.tum.de/data/datasets/rgbd-dataset
     * 
     * @param filename 输出文件路径
     */
    void SaveKeyFrameTrajectoryTUM(const string &filename);

    // 获取当前定位到的地图ID
    int GetCurrentMapId();

    /**
     * 保存地图到文件
     * 序列化地图点、关键帧等数据
     * 
     * @param filename 地图文件路径
     */
    void SaveMap(const string &filename);

    /**
     * 加载地图
     * @param filename 地图文件路径
     * @param mapId 地图ID (0为默认/主地图)
     * @param bAppend 是否追加模式 (true=保留现有地图, false=清除现有地图)
     */
    void LoadMap(const string &filename, int mapId = 0, bool bAppend = false);

    /**
     * 获取地图中的关键帧数量
     * @return 关键帧总数
     */
    int GetNumKeyFrames();

    /**
     * 获取地图中的地图点数量
     * @return 地图点总数
     */
    int GetNumMapPoints();

    /**
     * 获取所有地图点
     * @return 地图点指针列表
     */
    std::vector<MapPoint*> GetAllMapPoints();

    /**
     * 获取当前跟踪状态
     * 可在TrackMonocular()后立即调用
     * 
     * @return 状态值: -1=未初始化, 0=丢失, 1=跟踪中, 2=OK
     */
    int GetTrackingState();

    /**
     * 获取当前帧跟踪到的地图点
     * @return 地图点指针列表
     */
    std::vector<MapPoint*> GetTrackedMapPoints();

    /**
     * 获取当前帧跟踪到的关键点（去畸变后）
     * @return 关键点列表
     */
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

    /**
     * 获取重定位对齐置信度
     * 用于UI显示，反映当前位姿与加载地图的对齐程度
     * 
     * @return 置信度值（0.0-1.0）
     */
    float GetRelocAlignConfidence();

    /**
     * 获取重定位匹配分数
     * 不依赖PnP求解，可以更早地提示是否进入目标区域
     * 
     * @return 匹配分数（0.0-1.0）
     */
    float GetRelocMatchScore();

    /**
     * 检查是否已建立地图对齐
     * @return true-已对齐, false-未对齐
     */
    bool HasMapAlignment();

    /**
     * 获取对齐后的相机位姿
     * 将SLAM坐标系下的位姿转换到加载地图的坐标系
     * 
     * @param TcwSlam SLAM坐标系下的位姿
     * @return 对齐后的位姿
     */
    cv::Mat GetMapAlignedPose(const cv::Mat &TcwSlam);

    /**
     * 检查是否已加载地图
     * 
     * @return true-已加载地图, false-未加载地图
     */
    bool HasLoadedMap();

    /**
     * 创建新地图（子地图），用于跟踪丢失时的恢复
     */
    void CreateNewMap();

    /**
     * 切换到指定地图
     * @param pMap 目标地图指针
     */
    void SwitchToMap(Map* pMap);

//public:
private:
    // 多地图容器
    std::vector<Map*> mvpMaps;

    // ========== 传感器配置 ==========
    eSensor mSensor;  // 输入传感器类型

    // 用于位置识别的关键帧数据库（重定位和回环检测）。
    KeyFrameDatabase* mpKeyFrameDatabase;

    // 存储所有关键帧和地图点指针的地图结构。
    Map* mpMap;

    // 跟踪器：接收帧并计算相机位姿，决定何时插入关键帧、创建地图点，丢失时执行重定位。
    Tracking* mpTracker;

    // 局部建图器。它管理局部地图并执行局部束调整。
    LocalMapping* mpLocalMapper;

    // 回环闭合器。它搜索每个新关键帧的回环。如果存在回环，它会执行
    // 位姿图优化，然后执行完整的束调整（在新线程中）。
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

    // CreateNewMap 限频保护：防止高性能机器上频繁丢失导致连续触发新建子地图
    std::mutex mMutexNewMap;
    std::chrono::steady_clock::time_point mLastNewMapTime;
};

}// namespace ORB_SLAM2

#endif // SYSTEM_H

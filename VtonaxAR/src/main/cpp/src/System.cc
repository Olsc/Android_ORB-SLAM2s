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

#include "Common.h"
#include "System.h"
#include "Converter.h"
#include "Map.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "Config.h"
#include "EmbeddedResources.h"
#include <fstream>
#include <thread>
#include <chrono>
#include <condition_variable>
#include <Eigen/Core>
#include <iomanip>
#include <sstream>
#include "VtonaxProfiler.h" // 性能分析器

namespace ORB_SLAM2
{

System::System(const std::string &strSettingsFile, const eSensor sensor):mSensor(sensor),  mbReset(false),mbResetKeepMap(false),mbActivateLocalizationMode(false),
        mbDeactivateLocalizationMode(false)
{
    {
        cv::setNumThreads(1);
    }

    // 输出欢迎消息-此处虽注释掉但要保留！
        // std::cout << std::endl <<
        // "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << std::endl <<
        // "This program comes with ABSOLUTELY NO WARRANTY;" << std::endl  <<
        // "This is free software, and you are welcome to redistribute it" << std::endl <<
        // "under certain conditions. See LICENSE.txt." << std::endl << std::endl;

    // 加载 ORB LUT (优先尝试从嵌入资源加载)
    {
        const unsigned char* lutData = nullptr;
        size_t lutSize = 0;
        if(EmbeddedResources::Get("ORB_LUT.bin", lutData, lutSize)) {
            LOGD("正在从嵌入资源加载 ORB LUT (大小: %zu)", lutSize);
            ORBextractor::LoadLUT(lutData, lutSize);
        } else {
            LOGD("警告: 未找到嵌入的 ORB LUT，将回退到运行时计算");
        }
    }

    //创建关键帧数据库
    mpKeyFrameDatabase = new KeyFrameDatabase();

    //创建地图
    mpMap = new Map();
    mpMap->mnId = 0;
    mvpMaps.push_back(mpMap);

    //创建绘制器。这些由Viewer使用
    mpFrameDrawer = new FrameDrawer(mpMap);

    //初始化跟踪线程
    //(它将运行在主执行线程中，即调用此构造函数的线程)
    mpTracker = new Tracking(this, mpFrameDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

    //初始化局部建图线程并启动
    mpLocalMapper = new LocalMapping(mpMap);
    mptLocalMapping = new std::thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    //初始化回环闭合线程并启动
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase);
    mptLoopClosing = new std::thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    //设置线程间的指针
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);
}

void System::UpdateCalibration(float fx, float fy, float cx, float cy)
{
    if(mpTracker)
        mpTracker->UpdateCalibration(fx, fy, cx, cy);
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    VT_PROFILE_FUNCTION();
    if(mSensor!=MONOCULAR)
    {
        LOGE("TrackMonocular: 传感器未设置为单目");
        exit(-1);
    }

    // 检查模式更改
    {
        std::unique_lock<std::mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // 仅在 LM 已停止时切换模式，否则等下一帧再检查
            if(mpLocalMapper->isStopped())
            {
                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
                // 释放 LM 线程，使其从 Stopped 状态恢复。
                // 进入仅定位模式后 Tracking 不会再创建新关键帧，
                // LM 会在无新 KF 时空闲等待（3ms 事件循环），不会消耗 CPU。
                mpLocalMapper->Release();
            }
            // 未停止时不清除标志，下一帧继续等待
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // 检查重置
    {
        std::unique_lock<std::mutex> lock(mMutexReset);
        if(mbReset)
        {
            // 如果 LM 正处于 Stopped 状态等待 Release，需要先唤醒它，
            // 否则 RequestReset 设置的标志永远不会被 LM 读取到。
            if (mpLocalMapper) {
                mpLocalMapper->Release();
            }

            if(mbResetKeepMap) {
                mpTracker->ClearTrackingState();
            } else {
                mpTracker->Reset();
            }
            mbReset = false;
            mbResetKeepMap = false;
        }
    }

    /////////////
    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);
    /////////////

    std::unique_lock<std::mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

void System::ActivateLocalizationMode()
{
    std::unique_lock<std::mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    std::unique_lock<std::mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset(bool bKeepMap)
{
    std::unique_lock<std::mutex> lock(mMutexReset);
    if(bKeepMap) {
        LOGD("系统::请求重置 (保留地图模式)");
    }
    mbReset = true;
    mbResetKeepMap = bKeepMap;
}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();

    // 等待所有线程真正停止
    std::mutex mtx;
    std::condition_variable cv;
    std::unique_lock<std::mutex> lock(mtx);
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished()
          || mpLoopCloser->isRunningGBA())
    {
        cv.wait_for(lock, std::chrono::milliseconds(5));
    }

}

void System::SaveKeyFrameTrajectoryTUM(const std::string &filename)
{
    std::vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    std::sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    std::ofstream f;
    f.open(filename.c_str());
    f << std::fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        std::vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << std::setprecision(6) << pKF->mTimeStamp << std::setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << std::endl;
    }

    f.close();
}

int System::GetTrackingState()
{
    std::unique_lock<std::mutex> lock(mMutexState);
    return mTrackingState;
}

std::vector<MapPoint*> System::GetTrackedMapPoints()
{
    std::unique_lock<std::mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

std::vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    std::unique_lock<std::mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

float System::GetRelocAlignConfidence()
{
    // 由于Tracking在此处是完整类型（我们在System.cc中包含Tracking.h），可以安全访问
    return mpTracker ? mpTracker->GetAlignConfidence() : 0.0f;
}

float System::GetRelocMatchScore()
{
    return mpTracker ? mpTracker->GetRelocMatchScore() : 0.0f;
}

bool System::HasMapAlignment()
{
    return mpTracker ? mpTracker->HasMapAlignment() : false;
}

bool System::HasLoadedMap()
{
    return mpTracker ? mpTracker->HasLoadedMapData() : false;
}

void System::CreateNewMap()
{
    // ===== 限频保护：加 5 秒冷却期，防止频繁切换导致主线程被阻塞数十~数百毫秒 =====
    {
        std::unique_lock<std::mutex> lock(mMutexNewMap);
        auto now = std::chrono::steady_clock::now();
        if (mLastNewMapTime.time_since_epoch().count() != 0) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - mLastNewMapTime).count();
            if (elapsed < NEW_MAP_COOLDOWN_MS) {
                LOGD("System::CreateNewMap 距上次仅 %lld ms (< %d ms)，跳过以防抖动", NEW_MAP_COOLDOWN_MS,
                     (long long)elapsed);
                return;
            }
        }
        mLastNewMapTime = now;
    }

    LOGD("System::CreateNewMap 开始创建新子地图");

    // ===== 1. 暂停 LocalMapping/LoopClosing 接收新数据 =====
    if (mpLocalMapper) {
        mpLocalMapper->SetAcceptKeyFrames(false);
        mpLocalMapper->InterruptBA();
        mpLocalMapper->ClearQueues();
        // 释放 LM 线程
        mpLocalMapper->Release();
    }

    // 清空 LoopClosing 的回环关键帧队列（旧 Map 的 KF），避免对新空 Map 做 CorrectLoop。ClearQueue 不阻塞等待，耗时 O(1)。
    if (mpLoopCloser) {
        mpLoopCloser->ClearQueue();
    }

    // ===== 2. 停止后台重定位线程（防止切 Map 期间访问悬空指针） =====
    if (mpTracker) {
        mpTracker->StopGlobalRelocThread();
    }

    // ===== 3. 清空与旧 Map 绑定的重定位/对齐缓存 =====
    // 此时后台线程已停止，无竞争。不清 Map 本身，旧 Map 继续保留在 mvpMaps 中。
    if (mpTracker) {
        mpTracker->ClearRelocCacheForMapSwitch();
    }

    // ===== 4. 退出仅定位模式 =====
    // CreateNewMap 后需要在新地图上正常建图，不能停留在仅定位模式。
    {
        std::unique_lock<std::mutex> lock(mMutexMode);
        mbActivateLocalizationMode = false;
        mbDeactivateLocalizationMode = false;
    }
    if (mpTracker) {
        mpTracker->InformOnlyTracking(false);
    }

    // ===== 5. 创建新 Map 并切换 =====
    // 限制子地图总数，超出时删除最旧的地图（id=0 的初始地图不删）
    if ((int)mvpMaps.size() >= MAX_SUBMAP_COUNT) {
        // 找出最旧的非 id=0 地图并释放
        for (size_t i = 1; i < mvpMaps.size(); ++i) {
            if (mvpMaps[i] && mvpMaps[i] != mpMap) {
                LOGD("System::CreateNewMap 子地图数=%zu 达上限=%d，释放旧地图 id=%lu",
                     mvpMaps.size(), MAX_SUBMAP_COUNT, mvpMaps[i]->mnId);
                Map* pOldMap = mvpMaps[i];
                mvpMaps.erase(mvpMaps.begin() + i);
                delete pOldMap;
                break;
            }
        }
    }

    Map* pNewMap = new Map();
    pNewMap->mnId = mvpMaps.size();
    mvpMaps.push_back(pNewMap);

    LOGD("System::CreateNewMap 新地图 ID=%lu (旧地图保留为子地图，共 %zu 个)",
         pNewMap->mnId, mvpMaps.size());

    SwitchToMap(pNewMap);

    // ===== 6. 轻量重置跟踪运行时状态 =====
    // PrepareForNewMap 代替 Reset()：不阻塞 spin、不清旧 Map、不停重定位线程，全程 < 1 ms
    if (mpTracker) {
        mpTracker->PrepareForNewMap();
    }

    // ===== 7. 重启后台重定位线程 =====
    if (mpTracker) {
        mpTracker->StartGlobalRelocThread();
    }

    // ===== 8. 恢复 LocalMapping =====
    if (mpLocalMapper) {
        mpLocalMapper->SetAcceptKeyFrames(true);
    }

    LOGD("System::CreateNewMap 完成");
}

void System::SwitchToMap(Map* pMap)
{
    mpMap = pMap;
    if(mpTracker) {
        mpTracker->SetMap(pMap);
        mpTracker->mnCurrentMapId = (int)pMap->mnId;
    }
    if(mpLocalMapper) mpLocalMapper->SetMap(pMap);
    if(mpLoopCloser) mpLoopCloser->SetMap(pMap);
    if(mpFrameDrawer) mpFrameDrawer->SetMap(pMap);
}

cv::Mat System::GetMapAlignedPose(const cv::Mat &TcwSlam)
{
    return mpTracker ? mpTracker->GetMapAlignedPose(TcwSlam) : TcwSlam.clone();
}

int System::GetNumKeyFrames(){ return static_cast<int>(mpMap->KeyFramesInMap()); }
int System::GetNumMapPoints(){ return static_cast<int>(mpMap->MapPointsInMap()); }
std::vector<MapPoint*> System::GetAllMapPoints(){ return mpMap->GetAllMapPoints(); }

void System::SaveMap(const std::string &filename)
{
    // 增强的二进制序列化：保存完整的KF和MP信息以提高重定位精度
    LOGD("保存地图: 开始保存地图到 %s", filename.c_str());
    std::vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    std::vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();
    std::sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    std::ofstream ofs(filename, std::ios::binary);
    if(!ofs.is_open()){
        LOGE("保存地图: 无法打开文件 %s", filename.c_str());
        return;
    }
    const uint32_t magic = SYSTEM_MAP_FILE_MAGIC;
    const uint32_t version = SYSTEM_MAP_FILE_VERSION;
    ofs.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
    ofs.write(reinterpret_cast<const char*>(&version), sizeof(version));
    uint32_t nKFs = static_cast<uint32_t>(vpKFs.size());
    uint32_t nMPs = static_cast<uint32_t>(vpMPs.size());
    ofs.write(reinterpret_cast<const char*>(&nKFs), sizeof(nKFs));
    ofs.write(reinterpret_cast<const char*>(&nMPs), sizeof(nMPs));
    LOGD("保存地图: 关键帧=%u 地图点=%u", nKFs, nMPs);

    // 关键帧：[id][时间戳][位姿 16个浮点数][关键点数量][关键点及描述子]
    uint32_t writtenKFs = 0;
    for(KeyFrame* pKF : vpKFs)
    {
        if(!pKF || pKF->isBad()) continue;
        uint32_t id = static_cast<uint32_t>(pKF->mnId);
        ofs.write(reinterpret_cast<const char*>(&id), sizeof(id));
        double ts = pKF->mTimeStamp;
        ofs.write(reinterpret_cast<const char*>(&ts), sizeof(ts));
        cv::Mat Tcw = pKF->GetPose();
        float data[16] = {0};
        cv::Mat Tcwm = Tcw;
        // 行优先 4x4矩阵
        for(int r=0;r<4;r++) for(int c=0;c<4;c++) data[r*4+c] = Tcwm.at<float>(r,c);
        ofs.write(reinterpret_cast<const char*>(data), sizeof(data));
        
        // 保存关键点和描述子（用于更好的重定位）
        const std::vector<cv::KeyPoint>& vKeys = pKF->mvKeysUn;
        const cv::Mat& descriptors = pKF->mDescriptors;
        uint32_t numKeys = static_cast<uint32_t>(vKeys.size());
        ofs.write(reinterpret_cast<const char*>(&numKeys), sizeof(numKeys));
        
        if(numKeys > 0) {
            // 保存关键点（仅保存位置、尺度、角度）
            for(const auto& kp : vKeys) {
                float kpData[4] = {kp.pt.x, kp.pt.y, kp.size, kp.angle};
                ofs.write(reinterpret_cast<const char*>(kpData), sizeof(kpData));
            }
            
            // 保存描述子
            if(!descriptors.empty() && descriptors.rows == numKeys) {
                uint32_t descCols = descriptors.cols;
                ofs.write(reinterpret_cast<const char*>(&descCols), sizeof(descCols));
                ofs.write(reinterpret_cast<const char*>(descriptors.data), 
                         descriptors.rows * descriptors.cols * sizeof(uchar));
            } else {
                uint32_t descCols = 0;
                ofs.write(reinterpret_cast<const char*>(&descCols), sizeof(descCols));
            }
        }
        
        writtenKFs++;
    }

    // 地图点格式: [id][坐标 3浮点][描述符长度+数据][法向量 3浮点][最小距离][最大距离]
    // 实际上只保存 [id][pos][desc][normal]
    uint32_t writtenMPs = 0;
    for(MapPoint* pMP : vpMPs)
    {
        if(!pMP || pMP->isBad()) continue;
        // 确保描述符存在以进行持久化
        if(pMP->GetDescriptor().empty()) pMP->ComputeDistinctiveDescriptors();
        uint32_t id = static_cast<uint32_t>(pMP->mnId);
        ofs.write(reinterpret_cast<const char*>(&id), sizeof(id));
        cv::Mat Pw = pMP->GetWorldPos();
        float xyz[3] = {Pw.at<float>(0), Pw.at<float>(1), Pw.at<float>(2)};
        ofs.write(reinterpret_cast<const char*>(xyz), sizeof(xyz));

        // 描述符
        cv::Mat desc = pMP->GetDescriptor();
        uint32_t dlen = (desc.empty()? 0u : static_cast<uint32_t>(desc.cols));
        ofs.write(reinterpret_cast<const char*>(&dlen), sizeof(dlen));
        if(dlen>0){
            ofs.write(reinterpret_cast<const char*>(desc.ptr<uint8_t>(0)), dlen);
        }
        // 法向量和深度范围
        cv::Mat nrm = pMP->GetNormal();
        float n3[3] = {0,0,1};
        if(!nrm.empty()){ n3[0]=nrm.at<float>(0); n3[1]=nrm.at<float>(1); n3[2]=nrm.at<float>(2);}        
        ofs.write(reinterpret_cast<const char*>(n3), sizeof(n3));
        float mind = pMP->GetMinDistanceInvariance();
        float maxd = pMP->GetMaxDistanceInvariance();
        ofs.write(reinterpret_cast<const char*>(&mind), sizeof(mind));
        ofs.write(reinterpret_cast<const char*>(&maxd), sizeof(maxd));
        writtenMPs++;
    }
    ofs.close();
}

void System::LoadMap(const std::string &filename, int mapId, bool bAppend)
{
    LOGD("加载地图: %s (ID=%d, 追加=%d)", filename.c_str(), mapId, bAppend);
    std::ifstream ifs(filename, std::ios::binary);
    if(!ifs.is_open()){
        LOGE("加载地图: 无法打开文件 %s", filename.c_str());
        return;
    }
    uint32_t magic=0, version=0; 
    ifs.read(reinterpret_cast<char*>(&magic),4); 
    ifs.read(reinterpret_cast<char*>(&version),4);
    
    // 只支持MAP1格式
    if(magic != SYSTEM_MAP_FILE_MAGIC) {
        LOGE("加载地图: 无效的地图文件格式 (魔数=0x%08X)，只支持MAP1格式", magic);
        LOGE("加载地图: 请重新保存地图以使用MAP1格式");
        ifs.close();
        return;
    }
    if(version != SYSTEM_MAP_FILE_VERSION) {
        LOGE("加载地图: 不支持的版本 v%u，只支持 v3", version);
        ifs.close();
        return;
    }
    LOGD("加载地图: 格式 MAP1 v%u", version);
    uint32_t nKFs=0, nMPs=0; 
    ifs.read(reinterpret_cast<char*>(&nKFs),4); 
    ifs.read(reinterpret_cast<char*>(&nMPs),4);
    LOGD("加载地图: 关键帧=%u 地图点=%u", nKFs, nMPs);

    // 内存保护：检查地图大小，防止加载过大地图导致崩溃
    const uint32_t MAX_KFS = SYSTEM_MAX_KFS_LOAD;  // 最大关键帧数
    const uint32_t MAX_MPS = SYSTEM_MAX_MPS_LOAD; // 最大地图点数
    if(nKFs > MAX_KFS) {
        LOGE("加载地图: 地图过大！关键帧=%u 超过限制 %u，可能导致内存不足", nKFs, MAX_KFS);
        LOGD("加载地图: 建议：将地图分割或精简后再加载");
        // 但仍尝试加载，只是警告
    }
    if(nMPs > MAX_MPS) {
        LOGE("加载地图: 地图过大！地图点=%u 超过限制 %u，可能导致内存不足", nMPs, MAX_MPS);
        LOGD("加载地图: 建议：将地图分割或精简后再加载");
        // 但仍尝试加载，只是警告
    }
    
    // 只清除之前加载的地图点，保留当前扫描的点，保持跟踪状态连续，新点直接添加到现有地图
    
    if(!bAppend)
    {
        // 只清除之前加载的地图点，保留当前扫描建立的点
        std::vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();
        int removedLoadedMPs = 0;
        for(MapPoint* pMP : vpMPs) {
            if(pMP && pMP->mbFromLoadedMap) {
                mpMap->EraseMapPoint(pMP);
                removedLoadedMPs++;
            }
        }
        
        if(removedLoadedMPs > 0) {
            LOGD("加载地图: 已清除旧的加载地图点=%d，保留当前扫描点", removedLoadedMPs);
        }
        
        // 清除旧的重定位缓存，但不清除跟踪状态
        if(mpTracker) {
            mpTracker->ClearRelocCache();
        }
    }
    
    // 注意：不完全清空KeyFrameDatabase，保留当前扫描建立的关键帧
    // mpKeyFrameDatabase->clear(); // 暂时注释，避免影响当前跟踪
    
    // 优化重定位配置以提高加载地图后的跟踪稳定性
    mpTracker->SetRelocConfig(SYSTEM_RELOC_CONFIG_TOP_K, SYSTEM_RELOC_CONFIG_MAX_CANDIDATES, SYSTEM_RELOC_CONFIG_MATCH_CHUNK, SYSTEM_RELOC_CONFIG_BG_SLEEP_US, SYSTEM_RELOC_CONFIG_MAX_BIND_INLIERS, SYSTEM_RELOC_CONFIG_MAX_PROJ_BINDS);

    // 加载关键帧（MAP1格式包含完整特征点和描述子）
    uint32_t readKFs = 0;
    for(uint32_t i=0;i<nKFs;i++)
    {
        uint32_t id; double ts; float data[16];
        ifs.read(reinterpret_cast<char*>(&id),4);
        ifs.read(reinterpret_cast<char*>(&ts),sizeof(ts));
        ifs.read(reinterpret_cast<char*>(data),sizeof(data));
        
        // 读取关键点和描述子
        uint32_t numKeys = 0;
        ifs.read(reinterpret_cast<char*>(&numKeys), sizeof(numKeys));
        
        if(numKeys > 0) {
            // 读取关键点
            std::vector<cv::KeyPoint> vKeys;
            vKeys.reserve(numKeys);
            for(uint32_t j=0; j<numKeys; j++) {
                float kpData[4];
                ifs.read(reinterpret_cast<char*>(kpData), sizeof(kpData));
                cv::KeyPoint kp;
                kp.pt.x = kpData[0];
                kp.pt.y = kpData[1];
                kp.size = kpData[2];
                kp.angle = kpData[3];
                vKeys.push_back(kp);
            }
            
            // 读取描述子
            uint32_t descCols = 0;
            ifs.read(reinterpret_cast<char*>(&descCols), sizeof(descCols));
            if(descCols > 0) {
                cv::Mat descriptors(numKeys, descCols, CV_8U);
                ifs.read(reinterpret_cast<char*>(descriptors.data), 
                        numKeys * descCols * sizeof(uchar));
            }
        }
        
        // 注意：暂时只读取数据，不创建KeyFrame对象
        // 依靠正常跟踪流程在恢复后重建关键帧连接
        readKFs++;
    }
    LOGD("加载地图: 已读关键帧=%u", readKFs);

    // 加载地图点：创建点并恢复完整的描述子和几何信息用于匹配
    uint32_t createdMPs = 0;
    for(uint32_t i=0;i<nMPs;i++)
    {
        uint32_t id; float xyz[3];
        ifs.read(reinterpret_cast<char*>(&id),4);
        ifs.read(reinterpret_cast<char*>(xyz),sizeof(xyz));
        cv::Mat Pw = (cv::Mat_<float>(3,1) << xyz[0], xyz[1], xyz[2]);
        MapPoint* pMP = new MapPoint(Pw, mpMap);
        pMP->mbFromLoadedMap = true;
        pMP->SetMapId(mapId);
        
        // 读取描述子
        uint32_t dlen=0; ifs.read(reinterpret_cast<char*>(&dlen),4);
        if(dlen>0){
            std::vector<uint8_t> buf(dlen);
            ifs.read(reinterpret_cast<char*>(buf.data()), dlen);
            cv::Mat desc(1, dlen, CV_8U, buf.data());
            pMP->SetDescriptor(desc);
        }
        
        // 读取法线和深度范围
        float n3[3]; ifs.read(reinterpret_cast<char*>(n3), sizeof(n3));
        float mind=0, maxd=0; 
        ifs.read(reinterpret_cast<char*>(&mind),4); 
        ifs.read(reinterpret_cast<char*>(&maxd),4);
        cv::Mat nrm = (cv::Mat_<float>(3,1) << n3[0], n3[1], n3[2]);
        pMP->SetNormalAndDepth(nrm, mind, maxd);
        
        // 初始化可见性统计，保持Found/Visible比例为1.0以避免被MapPointCulling删除
        // 同时增加Found和Visible，让GetFoundRatio()=1.0 (远大于0.25阈值)
        if(!pMP->GetDescriptor().empty()) {
            pMP->IncreaseVisible(10);  // 有描述子的点
            pMP->IncreaseFound(10);    // 保持比例=1.0
        } else {
            pMP->IncreaseVisible(5);   // 无描述子的点
            pMP->IncreaseFound(5);     // 保持比例=1.0
        }
        mpMap->AddMapPoint(pMP);
        createdMPs++;
    }
    ifs.close();
    
    // 验证加载的地图点
    {
        const std::vector<MapPoint*> vAll = mpMap->GetAllMapPoints();
        int cntLoaded=0, cntLoadedWithDesc=0, cntLoadedWithNormal=0;
        for(MapPoint* p : vAll){
            if(!p || p->isBad()) continue;
            if(p->mbFromLoadedMap){
                cntLoaded++;
                if(!p->GetDescriptor().empty()) cntLoadedWithDesc++;
                if(!p->GetNormal().empty()) cntLoadedWithNormal++;
            }
        }
        LOGD("加载地图: 点数=%d, 描述子=%d (%.1f%%), 法线=%d (%.1f%%)", 
             cntLoaded, cntLoadedWithDesc, 
             cntLoaded > 0 ? (100.0f * cntLoadedWithDesc / cntLoaded) : 0.0f,
             cntLoadedWithNormal,
             cntLoaded > 0 ? (100.0f * cntLoadedWithNormal / cntLoaded) : 0.0f);
    }
    
    // 立即重建参考缓存，避免需要多次点击才生效
    if(mpTracker){ 
        mpTracker->BuildLoadedRefCache(); 
        LOGD("加载地图: 参考缓存已重建");
    }
    
    // 确保仍处于SLAM建图模式（不是仅定位），继续正常扫描与建图
    if(mpTracker) mpTracker->InformOnlyTracking(false);
    if(mpLocalMapper) mpLocalMapper->Release();
    
    LOGD("加载地图: 完成 KF=%lu MP=%lu", 
         mpMap->KeyFramesInMap(), mpMap->MapPointsInMap());
}

int System::GetCurrentMapId() {
    return mpTracker ? mpTracker->GetCurrentMapId() : 0;
}

} //namespace ORB_SLAM2

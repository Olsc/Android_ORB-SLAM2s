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
#include "ORBextractor.h"
#include <fstream>
#include <thread>
#include <chrono>
#include <condition_variable>
#include <Eigen/Core>
#include <iomanip>
#include <sstream>
#include "MenthaProfiler.h" // 性能分析器
#include <atomic>

namespace ORB_SLAM2
{

System::System(const std::string &strSettingsFile, const eSensor sensor):mSensor(sensor),  mbReset(false),mbResetKeepMap(false)
{
    {
        cv::setNumThreads(0);
    }

    // 输出欢迎消息-此处虽注释掉但要保留！
        // std::cout << std::endl <<
        // "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << std::endl <<
        // "This program comes with ABSOLUTELY NO WARRANTY;" << std::endl  <<
        // "This is free software, and you are welcome to redistribute it" << std::endl <<
        // "under certain conditions. See LICENSE.txt." << std::endl << std::endl;

    // 初始化 ORB 查找表 (自动在内存中生成)
    ORBextractor::InitLUT();

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

bool System::MapChanged()
{
    // 原为函数级 static int：detect JNI 线程与主线程并发调用存在数据竞争
    static std::atomic<int> n{0};
    int curn = mpMap->GetLastBigChangeIdx();
    int prev = n.load(std::memory_order_relaxed);
    while(prev < curn && !n.compare_exchange_weak(prev, curn)) {}
    return prev < curn;
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
    // 幂等保护：线程已 join 后不再重复关停
    if(!mptLocalMapping && !mptLoopClosing)
        return;

    // 先停后台重定位线程（防止 join 期间仍访问地图数据）
    if(mpTracker)
        mpTracker->StopGlobalRelocThread();

    // 停止可能正在运行的 GBA（join 而非 detach，见 LoopClosing::RequestStopGBA）
    if(mpLoopCloser)
        mpLoopCloser->RequestStopGBA();

    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();

    // join 替代原先"从未被 notify 的 wait_for 轮询"——确定性等待，零空转
    if(mptLocalMapping && mptLocalMapping->joinable())
        mptLocalMapping->join();
    if(mptLoopClosing && mptLoopClosing->joinable())
        mptLoopClosing->join();

    delete mptLocalMapping; mptLocalMapping = nullptr;
    delete mptLoopClosing;  mptLoopClosing = nullptr;
}

System::~System()
{
    Shutdown();
    // Shutdown 后 LM/LC/GlobalReloc 线程均已 join；按依赖序释放子模块后，
    // 已无任何持有者，可安全释放子地图中的全部 KeyFrame/MapPoint。
    // 注意：Map 没有析构清理（~Map 不能调 clear()——CreateNewMap 的子地图
    // 逐出路径中 KFD/回环队列仍持有旧地图 KF 指针，析构时 clear 会产生
    // 悬空指针），因此必须在此显式 clear() 后再 delete。
    delete mpTracker;       // Tracking 析构不再有线程成员（后台线程已停）
    delete mpLocalMapper;
    delete mpLoopCloser;
    delete mpFrameDrawer;
    delete mpKeyFrameDatabase;
    for(Map* pMap : mvpMaps)
    {
        if(pMap) { pMap->clear(); delete pMap; }
    }
    mvpMaps.clear();
    mpMap = nullptr;
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

        // 栈版读取（锁内拷贝）：R = GetRotation().t()，t = 相机中心
        float Rf[9];
        pKF->GetRotation(Rf);
        cv::Point3f t;
        pKF->GetCameraCenter(t);

        Eigen::Matrix3d Rm;
        Rm << Rf[0], Rf[1], Rf[2],
              Rf[3], Rf[4], Rf[5],
              Rf[6], Rf[7], Rf[8];
        Eigen::Quaterniond q(Rm.transpose());
        std::vector<float> qv(4);
        qv[0] = (float)q.x(); qv[1] = (float)q.y(); qv[2] = (float)q.z(); qv[3] = (float)q.w();

        f << std::setprecision(6) << pKF->mTimeStamp << std::setprecision(7) << " " << t.x << " " << t.y << " " << t.z
          << " " << qv[0] << " " << qv[1] << " " << qv[2] << " " << qv[3] << std::endl;
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
    // 限频已由 Tracking 侧的帧计数冷却（TRACKING_NEW_MAP_COOLDOWN_FRAMES=150）承担，
    // 原先的 5 秒墙钟冷却属于时间驱动防抖（R8），且与帧计数冷却重复，删除。

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

    if (mpTracker) {
        // 确保新地图上正常建图（无操作时也无害）
        mpTracker->InformOnlyTracking(false);
    }

    // ===== 5. 创建新 Map 并切换 =====
    std::vector<MapPoint*> savedLoadedMPs;
    {
        std::vector<MapPoint*> allMPs = mpMap->GetAllMapPoints();
        for(MapPoint* p : allMPs) {
            if(p && !p->isBad() && p->mbFromLoadedMap) {
                savedLoadedMPs.push_back(p);
                mpMap->EraseMapPoint(p, false); // 仅从旧地图集合移除，不 delete 对象
            }
        }
        if(!savedLoadedMPs.empty()) {
            LOGD("System::CreateNewMap 从旧地图移出 %d 个已加载地图点（待迁移到新地图）",
                 (int)savedLoadedMPs.size());
        }
    }

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

    // 将已加载地图点迁移到新地图，使后台重定位线程在新地图中仍可访问
    if(!savedLoadedMPs.empty()) {
        for(MapPoint* p : savedLoadedMPs) {
            pNewMap->AddMapPoint(p);
        }
        LOGD("System::CreateNewMap 已将 %d 个已加载地图点迁移到新地图 ID=%lu",
             (int)savedLoadedMPs.size(), pNewMap->mnId);
    }

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

    // 若迁移了已加载点，标记重建重定位缓存
    if(!savedLoadedMPs.empty() && mpTracker) {
        mpTracker->InvalidateRefCache();
        LOGD("System::CreateNewMap 已标记重建重定位缓存（%d 个已加载点），新地图可立即匹配",
             (int)savedLoadedMPs.size());
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

void System::SaveMap(const std::string &filename, int maxMapPoints)
{
    // 增强的二进制序列化：保存完整的KF和MP信息以提高重定位精度
    LOGD("保存地图: 开始保存地图到 %s (特征点上限=%d)", filename.c_str(), maxMapPoints);
    std::vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    std::vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();

    // 1. 过滤无效(null/bad)的关键帧与地图点
    std::vector<KeyFrame*> vpValidKFs;
    vpValidKFs.reserve(vpKFs.size());
    for(KeyFrame* pKF : vpKFs) {
        if(pKF && !pKF->isBad())
            vpValidKFs.push_back(pKF);
    }
    std::sort(vpValidKFs.begin(), vpValidKFs.end(), KeyFrame::lId);

    std::vector<MapPoint*> vpValidMPs;
    vpValidMPs.reserve(vpMPs.size());
    for(MapPoint* pMP : vpMPs) {
        if(pMP && !pMP->isBad())
            vpValidMPs.push_back(pMP);
    }

    // 2. 将地图特征点按时间线排序（mnId递增：从早期旧点到晚期新点）
    std::sort(vpValidMPs.begin(), vpValidMPs.end(), [](MapPoint* a, MapPoint* b){
        return a->mnId < b->mnId;
    });

    // 3. 检查特征点数量是否超过上限；若超出，从早期到晚期时间线裁剪，优先保留最新点
    size_t totalValidMPs = vpValidMPs.size();
    if(maxMapPoints > 0 && totalValidMPs > static_cast<size_t>(maxMapPoints)) {
        size_t numToPrune = totalValidMPs - static_cast<size_t>(maxMapPoints);
        LOGD("保存地图: 有效特征点数=%zu 超过上限=%d，从早期时间线裁剪掉 %zu 个旧点，保留最新 %d 个点",
             totalValidMPs, maxMapPoints, numToPrune, maxMapPoints);
        vpValidMPs.erase(vpValidMPs.begin(), vpValidMPs.begin() + numToPrune);
    } else {
        LOGD("保存地图: 有效特征点数=%zu (未超上限 %d)", totalValidMPs, maxMapPoints);
    }

    std::ofstream ofs(filename, std::ios::binary);
    if(!ofs.is_open()){
        LOGE("保存地图: 无法打开文件 %s", filename.c_str());
        return;
    }
    const uint32_t magic = SYSTEM_MAP_FILE_MAGIC;
    const uint32_t version = SYSTEM_MAP_FILE_VERSION;
    ofs.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
    ofs.write(reinterpret_cast<const char*>(&version), sizeof(version));

    // 写入实际保存的关键帧与地图点数量（保证二进制序列化头准确）
    uint32_t nKFs = static_cast<uint32_t>(vpValidKFs.size());
    uint32_t nMPs = static_cast<uint32_t>(vpValidMPs.size());
    ofs.write(reinterpret_cast<const char*>(&nKFs), sizeof(nKFs));
    ofs.write(reinterpret_cast<const char*>(&nMPs), sizeof(nMPs));
    LOGD("保存地图: 实际写入 关键帧=%u 地图点=%u", nKFs, nMPs);

    // 关键帧：[id][时间戳][位姿 16个浮点数][关键点数量][关键点及描述子]
    for(KeyFrame* pKF : vpValidKFs)
    {
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
    }

    // 地图点格式: [id][坐标 3浮点][描述符长度+数据][法向量 3浮点][最小距离][最大距离]
    for(MapPoint* pMP : vpValidMPs)
    {
        // 确保描述符存在以进行持久化
        if(pMP->GetDescriptor().empty()) pMP->ComputeDistinctiveDescriptors();
        uint32_t id = static_cast<uint32_t>(pMP->mnId);
        ofs.write(reinterpret_cast<const char*>(&id), sizeof(id));
        // 栈版读取
        cv::Point3f Pw;
        pMP->GetWorldPos(Pw);
        float xyz[3] = {Pw.x, Pw.y, Pw.z};
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
    }
    ofs.close();
    LOGD("保存地图: 完成写入 %s (KF=%u, MP=%u)", filename.c_str(), nKFs, nMPs);
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
            pMP->IncreaseVisible(LOADED_MP_INIT_VISIBLE);  // 有描述子的点
            pMP->IncreaseFound(LOADED_MP_INIT_VISIBLE);    // 保持比例=1.0
        } else {
            pMP->IncreaseVisible(LOADED_MP_INIT_VISIBLE_NO_DESC);   // 无描述子的点
            pMP->IncreaseFound(LOADED_MP_INIT_VISIBLE_NO_DESC);     // 保持比例=1.0
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

    // 标记重建参考缓存（[M2] 由后台重定位线程异步重建，避免加载时主线程卡顿）
    if(mpTracker){ 
        mpTracker->InvalidateRefCache(); 
        LOGD("加载地图: 参考缓存已标记重建");
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

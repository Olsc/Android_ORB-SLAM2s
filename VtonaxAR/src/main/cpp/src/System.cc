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
#include <iomanip>
#include <sstream>
#include <Utils.h>
#include "VtonaxProfiler.h" // 性能分析器

namespace ORB_SLAM2
{

System::System(const std::string &strVocFile, const std::string &strSettingsFile, const eSensor sensor):mSensor(sensor),  mbReset(false),mbResetKeepMap(false),mbActivateLocalizationMode(false),
        mbDeactivateLocalizationMode(false)
{
    // 输出欢迎消息-此处虽注释掉但要保留！
        // std::cout << std::endl <<
        // "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << std::endl <<
        // "This program comes with ABSOLUTELY NO WARRANTY;" << std::endl  <<
        // "This is free software, and you are welcome to redistribute it" << std::endl <<
        // "under certain conditions. See LICENSE.txt." << std::endl << std::endl;

    // std::cout << "输入传感器设置为: ";

    if(mSensor==MONOCULAR)
        // std::cout << "单目" << std::endl;
        ;
    else
    {
        // std::cerr << "错误：不支持的传感器类型" << std::endl;
        exit(-1);
    }

    // 检测是否使用嵌入资源（strVocFile为":embedded:"时）
    bool useEmbedded = (strVocFile == ":embedded:");

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

    //加载ORB词汇表
    LOGD("正在加载ORB词汇表...");

    mpVocabulary = new ORBVocabulary();

    bool bVocLoad=false;
    
    if(useEmbedded) {
        // 从嵌入资源加载词汇表
        LOGD("正在从嵌入资源加载词汇表");
        const unsigned char* vocData = nullptr;
        size_t vocSize = 0;
        if(EmbeddedResources::Get("ORBvoc.txt.arm.bin", vocData, vocSize)) {
            LOGD("嵌入词汇表大小：%zu", vocSize);
            bVocLoad = mpVocabulary->loadFromMemoryBin(vocData, vocSize);
        } else {
            // std::cerr << "无法获取嵌入的词汇表资源" << std::endl;
            exit(-1);
        }
    } else {
        // 传统方式：从文件加载
        if(USE_BINARY) bVocLoad=mpVocabulary->loadFromBinFile(strVocFile+".arm.bin");
        else
            bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    }

    //mpVocabulary->saveToBinFile(strVocFile+".arm.bin");
    //mpVocabulary->saveToTextFile(strVocFile+".min.txt");
    if(!bVocLoad)
    {
        // std::cerr << "词汇表路径错误。 " << std::endl;
        // std::cerr << "无法打开: " << strVocFile << std::endl;
        exit(-1);
    }
    LOGD("词汇表加载完成！");

    //创建关键帧数据库
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //创建地图
    mpMap = new Map();
    mpMap->mnId = 0;
    mvpMaps.push_back(mpMap);

    //创建绘制器。这些由Viewer使用
    mpFrameDrawer = new FrameDrawer(mpMap);

    //初始化跟踪线程
    //(它将运行在主执行线程中，即调用此构造函数的线程)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

    //初始化局部建图线程并启动
    mpLocalMapper = new LocalMapping(mpMap);
    mptLocalMapping = new std::thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    //初始化回环闭合线程并启动
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary);
    mptLoopClosing = new std::thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    //设置线程间的指针
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    VT_PROFILE_FUNCTION();
    recordTime();
    if(mSensor!=MONOCULAR)
    {
        // std::cerr << "错误：您调用了TrackMonocular但输入传感器未设置为单目。" << std::endl;
        exit(-1);
    }

    // 检查模式更改
    {
        std::unique_lock<std::mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // 等待局部建图器真正停止
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
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
        if(mbResetKeepMap) {
            mpTracker->ClearTrackingState();  // 只清除跟踪状态，保留地图
        } else {
            mpTracker->Reset();  // 完全重置
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
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished()
          || mpLoopCloser->isRunningGBA())
    {
        usleep(5000);
    }

}

void System::SaveKeyFrameTrajectoryTUM(const std::string &filename)
{
    // std::cout << std::endl << "Saving keyframe trajectory to " << filename << " ..." << std::endl;

    std::vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    std::sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // 变换所有关键帧，使第一个关键帧位于原点。
    // 闭环后，第一个关键帧可能不在原点。
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    std::ofstream f;
    f.open(filename.c_str());
    f << std::fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        std::vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << std::setprecision(6) << pKF->mTimeStamp << std::setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << std::endl;

    }

    f.close();
    // std::cout << std::endl << "trajectory saved!" << std::endl;
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
    // 暂停LocalMapping接受新关键帧
    if(mpLocalMapper)
    {
        mpLocalMapper->SetAcceptKeyFrames(false);
        mpLocalMapper->InterruptBA();
        mpLocalMapper->ClearQueues();
    }

    // 创建新地图
    Map* pNewMap = new Map();
    pNewMap->mnId = mvpMaps.size();
    mvpMaps.push_back(pNewMap);
    
    LOGD("系统: 创建新地图 ID: %lu", pNewMap->mnId);

    // 切换所有模块到新地图
    SwitchToMap(pNewMap);
    
    // 重置跟踪状态（进入初始化模式）
    if(mpTracker)
        mpTracker->Reset();
    
    // 恢复LocalMapping
    if(mpLocalMapper)
        mpLocalMapper->SetAcceptKeyFrames(true);
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
    // LOGD("保存地图完成: 已写入 关键帧=%u/%u 地图点=%u/%u", writtenKFs, nKFs, writtenMPs, nMPs);
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
    
    // 1. 只清除之前加载的地图点（mbFromLoadedMap=true），保留当前扫描的点
    // 2. 不调用 ClearTrackingState()，保持跟踪状态连续
    // 3. 直接将新地图点添加到现有地图中
    
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

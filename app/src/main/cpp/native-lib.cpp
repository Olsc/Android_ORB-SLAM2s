/**
 * 由Olsc于2025/8/25开始进行修改
 */

 #include <jni.h>
#include <string>
#include <sstream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <cmath>
#include <map>
#include <vector>
#include <opencv2/opencv.hpp>

#include "include/System.h"
#include "Common.h"
#include "Plane.h"
#include "UIUtils.h"
#include "Matrix.h"
#include "MapPoint.h"
#include "EmbeddedResources.h"
#include "include/Config.h"

extern "C" {

// 全局变量声明
std::string modelPath;

ORB_SLAM2::System* slamSys;
Plane* pPlane;
bool planeLoadedFromMap = false;  // 标记平面是否从地图加载

float fx, fy, cx, cy;
double timeStamp;
bool slamInited = false;

std::vector<ORB_SLAM2::MapPoint*> vMPs;
std::vector<cv::KeyPoint> vKeys;
cv::Mat Tcw;

// 用于vMPs和vKeys线程安全访问的互斥锁
std::mutex gMapPointsMutex;

// 平面检测状态常量引用 Config.h
// const int PLANE_DETECTED = ORB_SLAM2::PLANE_DETECTED;

// AR重定位点云显示控制
bool gEnableFullMapDisplay = true;       // 默认启用完整地图点云显示
int gLoadedMapPointCount = 0;            // 加载的地图点数量
const int MIN_NEW_POINTS_BEFORE_AR = ORB_SLAM2::MIN_NEW_POINTS_BEFORE_AR; // 至少需要新建50个地图点才启用AR模式

// 点云显示开关（同时控制绿色和蓝色点云）
bool gEnablePointCloudDisplay = true;  // 默认启用点云显示

// SLAM 开关控制
bool gEnableSLAM = true;  // 默认启用 SLAM

// SLAM丢失自动重置相关变量
double lastOkTime = 0.0;            // 上次SLAM正常工作的时间
bool wasLost = false;                // 上一帧是否处于LOST状态
const double LOST_RESET_TIMEOUT = ORB_SLAM2::LOST_RESET_TIMEOUT; // LOST状态持续3秒后重置

// AR对象存储
struct ArObjectInfo {
    float modelMatrix[16];  // AR对象的模型矩阵
    std::string objectId;   // 对象标识符
    bool isValid;
    float scale;           // 对象缩放系数
};
std::vector<ArObjectInfo> gArObjects;

// 多地图支持
std::mutex gMapDataMutex;
// 保护SLAM核心状态的互斥锁，防止在跟踪过程中进行Reset/LoadMap等破坏性操作造成崩溃
std::mutex gSlamStateMutex;
std::map<int, Plane*> gMapPlanes;
std::map<int, std::vector<ArObjectInfo>> gMapArObjects;
int gActiveMapId = 0;
bool gMapSwitching = false;
int gMapSwitchCounter = 0;
const int MAP_SWITCH_THRESHOLD = ORB_SLAM2::MAP_SWITCH_THRESHOLD; // 至少连续3帧识别到新地图才切换

// AR对象渲染状态
bool gShouldDrawArObject = false;
float gArObjectScale = ORB_SLAM2::AR_OBJECT_SCALE_DEFAULT;  // 默认缩放
float gCurrentModelMatrix[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
float gCurrentViewMatrix[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
float gCurrentProjectionMatrix[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};

// 3DOF/SLAM 视图矩阵偏移（用于在SLAM丢失时平滑切换到IMU视图）
// 偏移量 = View_SLAM * View_IMU^-1
// 当SLAM丢失时: View_Pred = Offset * View_IMU_Curr
float gViewMatrixOffset[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
// EKFQuat gViewOffsetQuat(1, 0, 0, 0); // 用于平滑插值
bool gOffsetInited = false;
float gOffsetAlpha = ORB_SLAM2::VIEW_SMOOTH_ALPHA; // 平滑系数

// 记录SLAM最后有效的世界坐标（Twc平移），用于3DOF/6DOF回退保持位置
float gLastTwcPosX = 0.0f, gLastTwcPosY = 0.0f, gLastTwcPosZ = 0.0f;
bool gHasLastTwcPos = false;


/**
 * 保存平面和AR对象信息到文件
 * 
 * 文件格式：
 *   - 魔数: 'ARIN' (0x4152494E)
 *   - 版本号: 1
 *   - 平面信息: 原点(x,y,z)、法向量(x,y,z)、旋转角
 *   - AR对象列表: 每个对象包含模型矩阵和缩放系数
 * 
 * 用途：
 *   与SLAM地图文件配套保存，用于重定位后恢复AR场景
 * 
 * @param filename 地图文件路径（不含扩展名）
 */
void SavePlaneAndArInfo(const std::string& filename)
{
    FUNCTION_TRACE;
    std::lock_guard<std::mutex> lock(gMapDataMutex);
    
    std::string infoFile = filename + ".arinfo";
    std::ofstream ofs(infoFile, std::ios::binary);
    if (!ofs.is_open()) return;
    
    // 文件头：魔数和版本号
    const uint32_t magic = 0x4152494E; // 'ARIN'（AR信息）
    const uint32_t version = 1;
    ofs.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
    ofs.write(reinterpret_cast<const char*>(&version), sizeof(version));
    
    // 保存平面信息
    uint8_t hasPlane = (pPlane != nullptr) ? 1 : 0;
    ofs.write(reinterpret_cast<const char*>(&hasPlane), sizeof(hasPlane));
    
    if (pPlane)
    {
        // 保存平面的原点坐标和法向量
        float o3[3] = {pPlane->o.at<float>(0), pPlane->o.at<float>(1), pPlane->o.at<float>(2)};
        float n3[3] = {pPlane->n.at<float>(0), pPlane->n.at<float>(1), pPlane->n.at<float>(2)};
        ofs.write(reinterpret_cast<const char*>(o3), sizeof(o3));
        ofs.write(reinterpret_cast<const char*>(n3), sizeof(n3));
        ofs.write(reinterpret_cast<const char*>(&pPlane->rang), sizeof(pPlane->rang));
    }
    
    // 保存AR对象
    uint32_t numObjects = static_cast<uint32_t>(gArObjects.size());
    ofs.write(reinterpret_cast<const char*>(&numObjects), sizeof(numObjects));
    
    for (const auto& obj : gArObjects)
    {
        if (!obj.isValid) continue;
        
        ofs.write(reinterpret_cast<const char*>(obj.modelMatrix), sizeof(obj.modelMatrix));
        
        uint32_t idLen = static_cast<uint32_t>(obj.objectId.length());
        ofs.write(reinterpret_cast<const char*>(&idLen), sizeof(idLen));
        
        if (idLen > 0)
        {
            ofs.write(obj.objectId.c_str(), idLen);
        }
    }
    
    ofs.close();
    LOGD("保存平面和AR信息：已保存到%s", infoFile.c_str());
}

// 从文件加载平面和AR对象信息
void LoadPlaneAndArInfo(const std::string& filename, int mapId)
{
    FUNCTION_TRACE;
    
    std::string infoFile = filename + ".arinfo";
    std::ifstream ifs(infoFile, std::ios::binary);
    
    if (!ifs.is_open())
    {
        LOGD("加载平面和AR信息：未找到AR信息文件(%s)", infoFile.c_str());
        return;
    }
    
    uint32_t magic = 0, version = 0;
    ifs.read(reinterpret_cast<char*>(&magic), 4);
    ifs.read(reinterpret_cast<char*>(&version), 4);
    
    if (magic != 0x4152494E)
    {
        LOGE("加载平面和AR信息：错误的魔数");
        ifs.close();
        return;
    }
    
    // 临时存储加载的数据
    Plane* loadedPlane = nullptr;
    std::vector<ArObjectInfo> loadedObjects;

    // 加载平面信息
    uint8_t hasPlane = 0;
    ifs.read(reinterpret_cast<char*>(&hasPlane), sizeof(hasPlane));
    if(hasPlane) {
        float o3[3], n3[3], rang;
        ifs.read(reinterpret_cast<char*>(o3), sizeof(o3));
        ifs.read(reinterpret_cast<char*>(n3), sizeof(n3));
        ifs.read(reinterpret_cast<char*>(&rang), sizeof(rang));
        
        // 验证数据有效性
        bool dataValid = true;
        for(int i=0; i<3; i++) {
            if(std::isnan(o3[i]) || std::isinf(o3[i]) || std::isnan(n3[i]) || std::isinf(n3[i])) {
                dataValid = false;
                LOGE("加载平面和AR信息：检测到无效的平面数据");
                break;
            }
        }
        
        if(dataValid) {
            loadedPlane = new Plane(n3[0], n3[1], n3[2], o3[0], o3[1], o3[2]);
            loadedPlane->rang = rang;
            LOGD("加载平面和AR信息：为地图%d加载平面", mapId);
        }
    }
    
    // 加载AR对象
    uint32_t numObjects = 0;
    ifs.read(reinterpret_cast<char*>(&numObjects), sizeof(numObjects));
    loadedObjects.reserve(numObjects);
    for(uint32_t i=0; i<numObjects; i++) {
        ArObjectInfo obj;
        ifs.read(reinterpret_cast<char*>(obj.modelMatrix), sizeof(obj.modelMatrix));
        uint32_t idLen = 0;
        ifs.read(reinterpret_cast<char*>(&idLen), sizeof(idLen));
        if(idLen > 0) {
            std::vector<char> buf(idLen);
            ifs.read(buf.data(), idLen);
            obj.objectId = std::string(buf.data(), idLen);
        }
        obj.isValid = true;
        loadedObjects.push_back(obj);
    }
    ifs.close();
    LOGD("加载平面和AR信息：为地图%d加载%d个AR对象", numObjects, mapId);

    // 更新全局映射
    {
        std::lock_guard<std::mutex> lock(gMapDataMutex);
        if(gMapPlanes.count(mapId) && gMapPlanes[mapId]) {
            delete gMapPlanes[mapId];
        }
        gMapPlanes[mapId] = loadedPlane;
        gMapArObjects[mapId] = loadedObjects;

        // 如果加载的是当前活跃地图，更新当前显示状态
        if (mapId == gActiveMapId) {
            if(pPlane) delete pPlane;
            pPlane = loadedPlane ? new Plane(*loadedPlane) : nullptr;
            planeLoadedFromMap = (pPlane != nullptr);
            
            gArObjects = loadedObjects;
            
            if(pPlane) {
                getRUBModelMatrixFromRDF(pPlane->glTpw, gCurrentModelMatrix);
                // 投影矩阵在SLAM初始化时已预计算，无需重复计算
                LOGD("加载平面和AR信息：更新地图%d的当前显示状态", mapId);
            }
        }
    }
}

int processImage(cv::Mat& image, cv::Mat& outputImage, int statusBuf[])
{
    FUNCTION_TRACE;
    timeStamp += 1.0 / 30.0;
    
    // SLAM 开关控制：如果 SLAM 被关闭，跳过 SLAM 处理
    int status = 0;  // 默认状态：NO_IMAGES_YET
    
    if (!gEnableSLAM)
    {
        // SLAM 已关闭，不进行跟踪，直接返回状态
        status = 0;  // 返回 NO_IMAGES_YET 状态
        
        {
            std::lock_guard<std::mutex> lock(gMapPointsMutex);
            vMPs.clear();
            vKeys.clear();
        }
        
        gShouldDrawArObject = false;  // 关闭 AR 对象显示
    }
    else
    {
        // SLAM 正常运行
        // LOGD("已启动");  // 注释掉避免刷屏
        
        cv::Mat imgSmall;
        cv::resize(image, imgSmall, cv::Size(image.cols / 2, image.rows / 2));
        bool isDark = false;
        
        if (imgSmall.channels() == 4)
        {
            cv::Mat gray;
            cv::cvtColor(imgSmall, gray, cv::COLOR_RGBA2GRAY);
            isDark = cv::mean(gray)[0] < 8.0;
        }
        else if (imgSmall.channels() == 1)
        {
            isDark = cv::mean(imgSmall)[0] < 8.0;
        }
        else
        {
            cv::Mat gray;
            cv::cvtColor(imgSmall, gray, cv::COLOR_BGR2GRAY);
            isDark = cv::mean(gray)[0] < 8.0;
        }
        
        if (!isDark)
        {
            // SLAM跟踪线程拥有最高优先级，绝不能因为拿不到锁而丢弃帧（会导致跟踪丢失）
            // 如果UI线程持有锁，SLAM线程等待几毫秒是完全可以接受的
            std::unique_lock<std::mutex> lock(gSlamStateMutex);
            
            if(slamSys) {
                // 执行跟踪
                Tcw = slamSys->TrackMonocular(imgSmall, timeStamp);
                status = slamSys->GetTrackingState();
                
                // 必须全程持有锁保护 slamSys 的访问！
                // 不能提前释放，否则在获取地图点时 slamSys 可能被 Reset 线程修改或销毁
                
                // 更新缓存
                {
                    std::lock_guard<std::mutex> lock2(gMapPointsMutex);
                    if(slamSys) { 
                        vMPs = slamSys->GetTrackedMapPoints();
                        vKeys = slamSys->GetTrackedKeyPointsUn();
                    }
                }
            } else {
                Tcw = cv::Mat();
                status = 0;
            }
            // 锁在这里自动释放（超出作用域）
        } else {
            // 黑暗状态
            Tcw = cv::Mat();
            std::unique_lock<std::mutex> lock(gSlamStateMutex);
            if(slamSys) {
                status = slamSys->GetTrackingState();
            } else {
                status = 0;
            }
        }
        
        // 确保 vMPs 在任何情况下都处于安全状态
        
        // 安全检查：确保slamSys仍然有效
        if(!slamSys) {
            // slamSys已失效，提前返回
            // 此情况罕见（仅在系统销毁时），但需要保护
            return status;
        }
        
        // 检查是否切换了地图
        int currentMapId = slamSys->GetCurrentMapId();
        if (currentMapId != gActiveMapId) {
            static int lastTargetMapId = -1;
            if (currentMapId == lastTargetMapId) {
                gMapSwitchCounter++;
            } else {
                gMapSwitchCounter = 1;
                lastTargetMapId = currentMapId;
            }

            if (gMapSwitchCounter >= MAP_SWITCH_THRESHOLD) {
                std::lock_guard<std::mutex> lock(gMapDataMutex);
                LOGD("检测到并确认地图切换：%d -> %d", gActiveMapId, currentMapId);
                
                // 保存当前状态到旧地图ID
                if (pPlane) {
                    if (gMapPlanes.count(gActiveMapId) && gMapPlanes[gActiveMapId]) {
                        delete gMapPlanes[gActiveMapId];
                    }
                    gMapPlanes[gActiveMapId] = new Plane(*pPlane);
                }
                gMapArObjects[gActiveMapId] = gArObjects;
                
                gActiveMapId = currentMapId;
                
                // 切换AR上下文
                if (pPlane) {
                    delete pPlane;
                    pPlane = nullptr;
                }
                
                if (gMapPlanes.count(currentMapId) && gMapPlanes[currentMapId]) {
                    pPlane = new Plane(*gMapPlanes[currentMapId]); // 克隆一份作为当前活跃平面
                    planeLoadedFromMap = true;
                } else {
                    planeLoadedFromMap = false;
                }
                
                // 切换AR对象列表
                if (gMapArObjects.count(currentMapId)) {
                    gArObjects = gMapArObjects[currentMapId]; // 复制vector
                } else {
                    gArObjects.clear();
                }
                
                // 更新模型矩阵（投影矩阵已在初始化时预计算）
                if (pPlane) {
                    getRUBModelMatrixFromRDF(pPlane->glTpw, gCurrentModelMatrix);
                    LOGD("恢复地图%d的AR上下文", currentMapId);
                }
                gMapSwitchCounter = 0;
            }
        } else {
            gMapSwitchCounter = 0;
        }

        // 如果SLAM正在跟踪，更新AR对象视图矩阵
        if(status == 2) {  // SLAM正常工作
            // 如果有对齐，使用对齐后的位姿更新AR对象的视图矩阵
            // 确保AR对象在加载地图的坐标系下正确显示
            cv::Mat TcwForAR = Tcw;
            if(slamSys->HasMapAlignment()) {
                TcwForAR = slamSys->GetMapAlignedPose(Tcw);
            }
            float tmpM[16];
            getColMajorMatrixFromMat(tmpM, TcwForAR);
            getRUBViewMatrixFromRDF(tmpM, gCurrentViewMatrix);
        }
        
        // AR对象显示条件
        // - SLAM正常跟踪 (status == 2)
        // - 平面存在 (pPlane != nullptr)
        // - 对齐检查：
        //   * 如果平面是从地图加载的，必须对齐成功（因为平面位置在地图坐标系下）
        //   * 如果是手动检测的平面，不需要对齐（可以正常显示）
        //   * 如果地图已加载但没有平面，需要对齐后才能显示（防止错误匹配）
        bool alignmentOK = true;  // 默认允许
        {
            std::lock_guard<std::mutex> lock(gMapDataMutex);
            if(pPlane != nullptr && planeLoadedFromMap) {
                // 平面从地图加载：必须对齐成功才能使用（平面位置在地图坐标系下）
                alignmentOK = slamSys->HasMapAlignment();
            } else if(pPlane != nullptr && !planeLoadedFromMap) {
                // 手动检测的平面：不需要对齐检查，可以直接显示
                alignmentOK = true;
            } else if(pPlane == nullptr && slamSys->HasLoadedMap()) {
                // 没有平面但地图已加载：需要对齐成功才能显示AR物体（防止错误匹配）
                alignmentOK = slamSys->HasMapAlignment();
            }
            // 如果没有地图也没有平面，alignmentOK保持为true（允许正常显示）
            gShouldDrawArObject = (status == 2) && (pPlane != nullptr) && alignmentOK;
        }

        
        // 检测SLAM丢失状态并自动重置（保留加载的地图）
        // status: 0=NO_IMAGES_YET, 1=NOT_INITIALIZED, 2=OK, 3=LOST
        bool isLost = (status == 3);
        
        if(!isLost) {
            // SLAM正常工作，更新最后正常时间
            lastOkTime = timeStamp;
            wasLost = false;
        } else {
            // SLAM处于LOST状态
            if(!wasLost) {
                // 刚进入LOST状态，记录时间
                wasLost = true;
                LOGD("SLAM进入丢失状态，开始计时...");
            } else {
                // 持续LOST，检查是否超时
                double lostDuration = timeStamp - lastOkTime;
                if(lostDuration >= LOST_RESET_TIMEOUT) {
                    LOGD("SLAM已丢失%.1f秒，执行轻量重置（保留加载的地图）...", lostDuration);
                    slamSys->Reset(true);  // 保留地图的重置
                    wasLost = false;
                    lastOkTime = timeStamp;
                    LOGD("SLAM轻量重置完成，已加载的地图数据已保留");
                }
            }
        }
    }

    // AR模式：显示完整地图点云（绿色）
    if(status==2) {
        // 统计当前所有地图点（每10帧统计一次以减少开销）
        // 注意：这个统计主要用于日志和监控，不影响点云显示
        static int currentNewPoints = 0;
        static int frameCounter = 0;
        if(frameCounter++ % 10 == 0) {
            currentNewPoints = 0;
            vector<ORB_SLAM2::MapPoint*> allMapPoints = slamSys->GetAllMapPoints();
            for(auto pMP : allMapPoints) {
                if(pMP && !pMP->isBad()) {
                    if(!pMP->mbFromLoadedMap) {
                        currentNewPoints++;  // 新建的地图点
                    }
                }
            }
        }
        
        // 一旦对齐成功，立即显示完整地图点云，无需等待新点数量
        // MIN_NEW_POINTS_BEFORE_AR 仍然影响后台对齐线程何时开始尝试对齐（在Tracking.cc中），
        // 但对齐成功后就应该立即展示结果，提供即时的视觉反馈
        bool hasAlignment = slamSys->HasMapAlignment();
        
        if(hasAlignment)
        {
            // 获取对齐后的相机位姿（在地图坐标系下）
            cv::Mat TcwForProjection = slamSys->GetMapAlignedPose(Tcw);
            
            // 获取所有地图点并绘制（绿色点云）- 受点云显示开关控制
            if(gEnablePointCloudDisplay) {
                vector<ORB_SLAM2::MapPoint*> allMapPoints = slamSys->GetAllMapPoints();
                drawAllMapPoints(TcwForProjection, allMapPoints, outputImage, fx, fy, cx, cy, true);
            }
        }
    }
    
    // 最后绘制跟踪到的特征点（蓝色点云）- 受点云显示开关控制
    if(gEnablePointCloudDisplay) {
        drawTrackedPoints(vKeys,vMPs,outputImage);
    }
    
    //cv::imwrite(modelPath+"/lala2.jpg",outputImage);
    return status;
}

JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_initSLAM(JNIEnv* env, jobject instance, jstring path_)
{
    const char* path = env->GetStringUTFChars(path_, 0);
    
    if (slamInited) return;
    
    slamInited = true;
    modelPath = path;
    // LOGD("SLAM初始化：使用SO中的嵌入资源");
    
    env->ReleaseStringUTFChars(path_, path);
    
    // 从Config.h加载相机参数
    fx = ORB_SLAM2::CAMERA_FX;
    fy = ORB_SLAM2::CAMERA_FY;
    cx = ORB_SLAM2::CAMERA_CX;
    cy = ORB_SLAM2::CAMERA_CY;
    // LOGD("相机参数：fx=%.2f fy=%.2f cx=%.2f cy=%.2f", fx, fy, cx, cy);
    
    timeStamp = 0.0;
    
    // 预计算投影矩阵（相机内参固定，只需计算一次）
    // 使用640x360分辨率（1280/2, 720/2 - 因为输入图像被缩放）
    frustumM_RUB(640, 360, fx, fy, cx, cy, 0.1, 1000, gCurrentProjectionMatrix);
    // LOGD("投影矩阵已预计算");
    
    // 初始化SLAM系统
    // LOGD("正在使用嵌入词汇表初始化SLAM系统...");
    // 传递空字符串作为YAML参数，因为相机参数现在在Config.h中
    slamSys = new ORB_SLAM2::System(":embedded:", "", ORB_SLAM2::System::MONOCULAR);
}

JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_saveMap(JNIEnv* env, jobject instance, jstring path_)
{
    const char* path = env->GetStringUTFChars(path_, 0);
    
    if (slamSys)
    {
        // LOGD("JNI保存地图开始：%s", path);
        
        double t0 = static_cast<double>(cv::getTickCount());
        slamSys->SaveMap(std::string(path));
        SavePlaneAndArInfo(std::string(path)); // 保存平面和AR信息
        
        double t1 = static_cast<double>(cv::getTickCount());
        double ms = (t1 - t0) * 1000.0 / cv::getTickFrequency();
        
        // LOGD("JNI保存地图结束：%s，耗时%.1f毫秒，当前关键帧=%d 地图点=%d", path, ms,
        //     slamSys->GetNumKeyFrames(), slamSys->GetNumMapPoints());
    }
    
    env->ReleaseStringUTFChars(path_, path);
}

JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_loadMap(JNIEnv* env, jobject instance, jstring path_)
{
    const char* path = env->GetStringUTFChars(path_, 0);
    
    if (slamSys)
    {
        // 在加载地图时锁定SLAM状态，阻止processImage进行跟踪
        std::lock_guard<std::mutex> lock(gSlamStateMutex);
        
        LOGD("JNI加载地图开始：%s", path);
        
        double t0 = static_cast<double>(cv::getTickCount());
        slamSys->LoadMap(std::string(path), 0, false); // 默认ID=0，覆盖模式
        LoadPlaneAndArInfo(std::string(path), 0); // 加载平面和AR信息
        
        double t1 = static_cast<double>(cv::getTickCount());
        double ms = (t1 - t0) * 1000.0 / cv::getTickFrequency();
        
        // 统计加载的地图点数量
        gLoadedMapPointCount = 0;
        std::vector<ORB_SLAM2::MapPoint*> allMPs = slamSys->GetAllMapPoints();
        
        for (auto pMP : allMPs)
        {
            if (pMP && !pMP->isBad() && pMP->mbFromLoadedMap)
            {
                gLoadedMapPointCount++;
            }
        }
        
        // LOGD("JNI加载地图结束：%s，耗时%.1f毫秒，内存中关键帧=%d 地图点=%d (加载=%d)", path, ms,
        //     slamSys->GetNumKeyFrames(), slamSys->GetNumMapPoints(), gLoadedMapPointCount);
        
        // LOGD("AR模式：将在构建%d个新地图点后可用以确保稳定性", MIN_NEW_POINTS_BEFORE_AR);
    }
    
    env->ReleaseStringUTFChars(path_, path);
}

JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_loadMapWithId(JNIEnv *env, jobject instance,
                                               jstring path_, jint mapId, jboolean append) {
    const char *path = env->GetStringUTFChars(path_, 0);
    if(slamSys){
        // 在加载地图时锁定SLAM状态，阻止processImage进行跟踪
        std::lock_guard<std::mutex> lock(gSlamStateMutex);

        // LOGD("JNI加载地图ID开始：%s，ID=%d，追加=%d", path, mapId, append);
        double t0 = (double)cv::getTickCount();
        
        // 如果不是追加模式，清理旧的全局数据
        if (!append) {
             std::lock_guard<std::mutex> lock(gMapDataMutex);
             // 清理 gMapPlanes 和 gMapArObjects
             for (auto& kv : gMapPlanes) {
                 if (kv.second) delete kv.second;
             }
             gMapPlanes.clear();
             gMapArObjects.clear();
             gActiveMapId = mapId; // 强制设置活跃地图ID
             // 注意：System::LoadMap(append=false) 会清理地图点，但我们这里也需要清理关联的AR数据
        }

        slamSys->LoadMap(std::string(path), mapId, append);
        LoadPlaneAndArInfo(std::string(path), mapId);
        
        double t1 = (double)cv::getTickCount();
        double ms = (t1 - t0) * 1000.0 / cv::getTickFrequency();
        
        // LOGD("JNI加载地图ID结束：%s，耗时%.1f毫秒", path, ms);
    }
    env->ReleaseStringUTFChars(path_, path);
}

JNIEXPORT jint JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_getCurrentMapId(JNIEnv* env, jobject instance)
{
    return slamSys ? slamSys->GetCurrentMapId() : 0;
}

JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_nativeProcessFrameMat(
    JNIEnv* env, jobject instance, jlong matAddrGr, jlong matAddrRgba, jintArray statusBuf_)
{
    jint* statusBuf = env->GetIntArrayElements(statusBuf_, nullptr);
    
    cv::Mat& mGr = *(cv::Mat*)matAddrGr;
    cv::Mat& mRgba = *(cv::Mat*)matAddrRgba;
    
    statusBuf[0] = processImage(mGr, mRgba, statusBuf);
    
    env->ReleaseIntArrayElements(statusBuf_, statusBuf, 0);
}
JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_detect(JNIEnv *env, jobject instance,
                                               jintArray statusBuf_) {
    jint *statusBuf = env->GetIntArrayElements(statusBuf_, NULL);
    // 性能优化：使用try_lock避免阻塞，如果锁被占用则跳过本次检测
    std::unique_lock<std::mutex> slamLock(gSlamStateMutex, std::try_to_lock);
    std::unique_lock<std::mutex> dataLock(gMapDataMutex, std::try_to_lock);
    if(!slamLock.owns_lock() || !dataLock.owns_lock() || Tcw.empty()){
        statusBuf[1] = ORB_SLAM2::PLANE_NOT_DETECTED;
        env->ReleaseIntArrayElements(statusBuf_, statusBuf, 0);
        return;
    }
    if(!Tcw.empty()){
        // 平面检测也应该在对齐后的坐标系下进行（如果有对齐）
        cv::Mat TcwForPlane = Tcw;
        if(slamSys->HasMapAlignment()) {
            TcwForPlane = slamSys->GetMapAlignedPose(Tcw);
        }
        
        pPlane=detectPlane(TcwForPlane,vMPs,50);
        if(pPlane && slamSys->MapChanged())
            pPlane->Recompute();
        statusBuf[1]=pPlane? ORB_SLAM2::PLANE_DETECTED : ORB_SLAM2::PLANE_NOT_DETECTED;
        
        // 检测到平面时更新AR对象矩阵
        if(pPlane) {
            planeLoadedFromMap = false;  // 手动检测的平面，标记为非地图加载
            
            // 更新模型矩阵（投影矩阵已在SLAM初始化时预计算）
            getRUBModelMatrixFromRDF(pPlane->glTpw, gCurrentModelMatrix);
            
            // LOGD("检测到平面，AR对象矩阵已更新（如有对齐）");
        }
    }
    env->ReleaseIntArrayElements(statusBuf_, statusBuf, 0);
}

JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_getM(JNIEnv *env, jobject instance, jfloatArray modelM_) {
    jfloat *modelM = env->GetFloatArrayElements(modelM_, NULL);

    std::lock_guard<std::mutex> lock(gMapDataMutex);
    getRUBModelMatrixFromRDF(pPlane->glTpw,modelM);

    env->ReleaseFloatArrayElements(modelM_, modelM, 0);
}
JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_getV(JNIEnv *env, jobject instance, jfloatArray viewM_) {
    jfloat *viewM = env->GetFloatArrayElements(viewM_, NULL);

    bool useSlam = false;
    cv::Mat TcwForView;
    if(slamSys){
        int st = slamSys->GetTrackingState();
        if(st==2 && !Tcw.empty()) {
            useSlam = true;
            TcwForView = Tcw;
            if(slamSys->HasMapAlignment()) {
                TcwForView = slamSys->GetMapAlignedPose(Tcw);
            }
        }
    }
    if(useSlam){
        float tmpM[16];
        getColMajorMatrixFromMat(tmpM, TcwForView);
        getRUBViewMatrixFromRDF(tmpM, viewM);
    } else {
        setIdentityM(viewM);
    }

    env->ReleaseFloatArrayElements(viewM_, viewM, 0);
}

JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_getP(JNIEnv *env, jobject instance, jint imageWidth,
                                             jint imageHeight, jfloatArray projectionM_) {
    jfloat *projectionM = env->GetFloatArrayElements(projectionM_, NULL);

    //TODO:/2
    frustumM_RUB(imageWidth/2,imageHeight/2,fx,fy,cx,cy,0.1,1000,projectionM);

    env->ReleaseFloatArrayElements(projectionM_, projectionM, 0);
}

// 获取地图统计信息
JNIEXPORT jintArray JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_getMapStats(JNIEnv *env, jobject instance) {
    std::lock_guard<std::mutex> lock(gMapDataMutex);
    jintArray result = env->NewIntArray(3);
    if(slamSys) {
        jint stats[3];
        stats[0] = slamSys->GetNumKeyFrames();
        stats[1] = slamSys->GetNumMapPoints();
        stats[2] = (pPlane != nullptr) ? 1 : 0;
        env->SetIntArrayRegion(result, 0, 3, stats);
    }
    return result;
}

// 比较器，用于按MapPoint的ID排序
struct MapPointComparator {
    bool operator()(const ORB_SLAM2::MapPoint* a, const ORB_SLAM2::MapPoint* b) const {
        // 通常 ORB-SLAM2 中 mnId 是 public
        return a->mnId > b->mnId; // 降序，ID 大的在前（最新的）
    }
};

JNIEXPORT jfloatArray JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_getMiniMapPoints(JNIEnv *env, jobject instance, jint maxPoints) {
    // 性能优化：使用try_lock避免阻塞UI线程
    std::unique_lock<std::mutex> slamLock(gSlamStateMutex, std::try_to_lock);
    if(!slamLock.owns_lock() || !slamSys) {
        return env->NewFloatArray(0);
    }
    std::vector<ORB_SLAM2::MapPoint*> v = slamSys->GetAllMapPoints();
    // 立即释放锁，后续处理不需要锁保护
    slamLock.unlock();
    
    std::vector<float> out;
    size_t total = v.size();
    
    // 策略：使用 Top-K 算法获取最新的 maxPoints 个点（按 ID 降序）
    // 这样可以确保画面实时更新，且严格遵守数量限制，实现"遗忘"旧点
    
    if (total > 0) {
        if (total > (size_t)maxPoints) {
            // 使用 partial_sort 找出前 maxPoints 个最大的（最新的）点
            // 注意：这会改变 v 的前 maxPoints 个元素的顺序
            // 因为 v 是我们拷贝出来的副本（GetAllMapPoints 返回值），所以修改它是安全的
            std::partial_sort(v.begin(), v.begin() + maxPoints, v.end(), MapPointComparator());
            
            // 只取前 maxPoints 个
            out.reserve((size_t)maxPoints * 3);
            for(size_t i=0; i<(size_t)maxPoints; ++i) {
                ORB_SLAM2::MapPoint* p = v[i];
                // 访问前再次检查指针有效性
                if(!p) continue;
                if(p->isBad()) continue;
                cv::Mat P = p->GetWorldPos();
                if(P.empty()) continue;  // 检查空矩阵的安全性检查
                out.push_back(P.at<float>(0));
                out.push_back(P.at<float>(1));
                out.push_back(P.at<float>(2));
            }
        } else {
            // 点数不足，全部返回
            out.reserve(total * 3);
            for(size_t i=0; i<total; ++i) {
                ORB_SLAM2::MapPoint* p = v[i];
                // 访问前再次检查指针有效性
                if(!p) continue;
                if(p->isBad()) continue;
                cv::Mat P = p->GetWorldPos();
                if(P.empty()) continue;  // 检查空矩阵的安全性检查
                out.push_back(P.at<float>(0));
                out.push_back(P.at<float>(1));
                out.push_back(P.at<float>(2));
            }
        }
    }
 
    jfloatArray arr = env->NewFloatArray((jsize)out.size());
    if(arr && !out.empty()) env->SetFloatArrayRegion(arr, 0, (jsize)out.size(), out.data());
    return arr;
}

// 获取当前跟踪的地图点
JNIEXPORT jfloatArray JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_getTrackedPoints(JNIEnv *env, jobject instance, jint maxPoints) {
    // 性能优化：getTrackedPoints只需要gMapPointsMutex，不需要gSlamStateMutex
    // 因为我们访问的是已经复制好的vMPs缓存，而不是直接访问slamSys
    std::vector<float> out;
    
    // 线程安全地复制vMPs以防止与SLAM线程的竞态条件
    std::vector<ORB_SLAM2::MapPoint*> localMPs;
    {
        std::lock_guard<std::mutex> lock(gMapPointsMutex);
        localMPs = vMPs;  // 持有锁时创建副本
    }
    
    size_t total = localMPs.size();
    
    // 限制点数（如果需要）
    size_t limit = (maxPoints > 0 && (size_t)maxPoints < total) ? (size_t)maxPoints : total;
    
    out.reserve(limit * 3);
    for(size_t i=0; i<limit; ++i) {
        ORB_SLAM2::MapPoint* p = localMPs[i];
        // 访问前再次检查指针有效性
        if(!p) continue;
        if(p->isBad()) continue;
        cv::Mat P = p->GetWorldPos();
        if(P.empty()) continue;  // 检查空矩阵的安全性检查
        out.push_back(P.at<float>(0));
        out.push_back(P.at<float>(1));
        out.push_back(P.at<float>(2));
    }
 
    jfloatArray arr = env->NewFloatArray((jsize)out.size());
    if(arr && !out.empty()) env->SetFloatArrayRegion(arr, 0, (jsize)out.size(), out.data());
    return arr;
}

// 获取所有AR对象数据
// 格式: [数量, m0...m15, 缩放, m0...m15, 缩放, ...]
JNIEXPORT jfloatArray JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_getAllArObjectsData(JNIEnv *env, jobject instance) {
    std::lock_guard<std::mutex> lock(gMapDataMutex);
    std::vector<float> data;
    data.push_back((float)gArObjects.size());
    
    for(const auto& obj : gArObjects) {
        if(!obj.isValid) continue;
        for(int i=0; i<16; i++) {
            data.push_back(obj.modelMatrix[i]);
        }
        data.push_back(obj.scale);
    }
    
    jfloatArray arr = env->NewFloatArray((jsize)data.size());
    if(arr && !data.empty()) env->SetFloatArrayRegion(arr, 0, (jsize)data.size(), data.data());
    return arr;
}

// 保存当前模型矩阵到AR对象列表
JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_saveArObject(JNIEnv *env, jobject instance, 
                                                    jstring objectId_, jfloatArray matrix_) {
    const char *objectId = env->GetStringUTFChars(objectId_, 0);
    jfloat *matrix = env->GetFloatArrayElements(matrix_, NULL);
    
    std::lock_guard<std::mutex> lock(gMapDataMutex);
    ArObjectInfo obj;
    for(int i=0; i<16; i++) obj.modelMatrix[i] = matrix[i];
    obj.objectId = std::string(objectId);
    obj.isValid = true;
    gArObjects.push_back(obj);
    
    LOGD("保存AR对象：已添加对象%s", objectId);
    
    env->ReleaseFloatArrayElements(matrix_, matrix, 0);
    env->ReleaseStringUTFChars(objectId_, objectId);
}

// 清除AR对象
JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_clearArObjects(JNIEnv *env, jobject instance) {
    std::lock_guard<std::mutex> lock(gMapDataMutex);
    gArObjects.clear();
    LOGD("清除AR对象：已清除所有AR对象");
}

// 获取AR对象数量
JNIEXPORT jint JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_getArObjectCount(JNIEnv *env, jobject instance) {
    return static_cast<jint>(gArObjects.size());
}

// 重置SLAM系统（轻量重置：只清除跟踪状态，保留加载的地图）
JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_resetSLAM(JNIEnv *env, jobject instance) {
    // 必须先获取gSlamStateMutex，确保processImage不会同时访问slamSys
    std::lock_guard<std::mutex> slamLock(gSlamStateMutex);
    std::lock_guard<std::mutex> dataLock(gMapDataMutex);
    if(slamSys) {
        LOGD("手动重置SLAM（保留地图模式）");
        slamSys->Reset(true);  // 保留地图的重置
        
        //  只重置手动检测的平面，保留从地图加载的平面
        if(pPlane && !planeLoadedFromMap) {
            // 如果平面是手动检测的（非地图加载），则删除
            delete pPlane;
            pPlane = nullptr;
            LOGD("已重置手动检测的平面");
        } else if(pPlane && planeLoadedFromMap) {
            // 如果平面是从地图加载的，保留它
            LOGD("保留从地图加载的平面（重置后仍可见）");
            // 重置AR对象渲染状态会在下一帧自动根据pPlane存在与否恢复
        }
        
        gArObjects.clear();
        // 注意：不重置 gShouldDrawArObject，让它由状态自动更新
        // gShouldDrawArObject = false;  // 移除这行，让它根据pPlane自动判断
        gArObjectScale = 0.20f;
        
        // 重置丢失计时器
        wasLost = false;
        lastOkTime = timeStamp;
        // 注意：不需要重置gLoadedMapPointCount，因为地图还在
        LOGD("SLAM已重置，地图数据已保留，新地图点计数器将重新开始");
    } else {
        LOGD("警告：SLAM系统尚未初始化");
    }
}

// 启用/禁用完整地图显示（AR重定位模式）
JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_setFullMapDisplay(JNIEnv *env, jobject instance, jboolean enable) {
    gEnableFullMapDisplay = (bool)enable;
    // LOGD("完整地图显示模式：%s", gEnableFullMapDisplay ? "启用" : "禁用");
}

// 获取完整地图显示状态
JNIEXPORT jboolean JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_isFullMapDisplayEnabled(JNIEnv *env, jobject instance) {
    return (jboolean)gEnableFullMapDisplay;
}

// ========== AR对象管理（C++端） ==========

// 检查是否应该绘制AR对象（基于SLAM状态和平面检测）
JNIEXPORT jboolean JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_shouldDrawArObject(JNIEnv *env, jobject instance) {
    return (jboolean)gShouldDrawArObject;
}

// 获取AR对象的当前模型矩阵
JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_getArObjectModelMatrix(JNIEnv *env, jobject instance, jfloatArray matrix_) {
    jfloat *matrix = env->GetFloatArrayElements(matrix_, NULL);
    for(int i = 0; i < 16; i++) {
        matrix[i] = gCurrentModelMatrix[i];
    }
    env->ReleaseFloatArrayElements(matrix_, matrix, 0);
}

// 获取AR对象的当前视图矩阵
JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_getArObjectViewMatrix(JNIEnv *env, jobject instance, jfloatArray matrix_) {
    jfloat *matrix = env->GetFloatArrayElements(matrix_, NULL);
    for(int i = 0; i < 16; i++) {
        matrix[i] = gCurrentViewMatrix[i];
    }
    env->ReleaseFloatArrayElements(matrix_, matrix, 0);
}

// 获取AR对象的当前投影矩阵
JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_getArObjectProjectionMatrix(JNIEnv *env, jobject instance, jfloatArray matrix_) {
    jfloat *matrix = env->GetFloatArrayElements(matrix_, NULL);
    for(int i = 0; i < 16; i++) {
        matrix[i] = gCurrentProjectionMatrix[i];
    }
    env->ReleaseFloatArrayElements(matrix_, matrix, 0);
}

// 更新AR对象缩放（当用户进行捏合缩放时从Java调用）
JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_updateArObjectScale(JNIEnv *env, jobject instance, jfloat scaleFactor) {
    float zoomFac = (scaleFactor - 1.0f) / 5.0f;
    gArObjectScale += zoomFac;
    gArObjectScale = fmax(0.03f, gArObjectScale);  // 最小缩放
    LOGD("AR对象缩放已更新：%.3f", gArObjectScale);
}

// 获取当前AR对象缩放
JNIEXPORT jfloat JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_getArObjectScale(JNIEnv *env, jobject instance) {
    return gArObjectScale;
}

// 设置点云显示开关（控制绿色和蓝色点云）
JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_setPointCloudDisplay(JNIEnv *env, jobject instance, jboolean enable) {
    gEnablePointCloudDisplay = (bool)enable;
    LOGD("点云显示模式：%s", gEnablePointCloudDisplay ? "启用" : "禁用");
}

// 获取点云显示状态
JNIEXPORT jboolean JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_isPointCloudDisplayEnabled(JNIEnv *env, jobject instance) {
    return (jboolean)gEnablePointCloudDisplay;
}


// ========== SLAM 开关控制接口 ==========

// 启用/禁用 SLAM
JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_setEnableSLAM(JNIEnv *env, jobject instance, jboolean enable) {
    bool wasEnabled = gEnableSLAM;
    gEnableSLAM = (bool)enable;
    
    if(wasEnabled && !gEnableSLAM) {
        // SLAM 从开启变为关闭，清理相关状态
        {
            std::lock_guard<std::mutex> lock(gMapPointsMutex);
            vMPs.clear();
            vKeys.clear();
        }
        gShouldDrawArObject = false;
        LOGD("SLAM 已关闭");
    } else if(!wasEnabled && gEnableSLAM) {
        // SLAM 从关闭变为开启
        LOGD("SLAM 已开启");
    }
}

// 获取 SLAM 启用状态
JNIEXPORT jboolean JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_isEnableSLAM(JNIEnv *env, jobject instance) {
    return (jboolean)gEnableSLAM;
}

// 启用/禁用 光流
JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_setOpticalFlowEnabled(JNIEnv *env, jobject instance, jboolean enable) {
    if(slamSys) {
        slamSys->SetOpticalFlow((bool)enable);
        LOGD("光流状态已更新: %s", enable ? "开启" : "关闭");
    }
}

}
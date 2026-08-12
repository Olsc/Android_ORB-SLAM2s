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
#include <cstring>
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
#include "include/Config.h"
#include "MenthaProfiler.h"

extern "C" {

// 全局变量声明
std::string modelPath;

ORB_SLAM2::System* slamSys;
Plane* pPlane;
bool planeLoadedFromMap = false;  // 标记平面是否从地图加载

float fx, fy, cx, cy;
float gBaseFx, gBaseFy, gBaseCx, gBaseCy;  // 基准内参 (640x360校准值)
float gScaledFx, gScaledFy, gScaledCx, gScaledCy;  // 缩放后的内参
double timeStamp;
bool slamInitialized = false;

std::vector<ORB_SLAM2::MapPoint*> vMPs;
std::vector<cv::KeyPoint> vKeys;

// 用于vMPs和vKeys线程安全访问的互斥锁
std::mutex gMapPointsMutex;

// 点云显示开关（同时控制绿色和蓝色点云）
std::atomic<bool> gEnablePointCloudDisplay{true};  // 默认启用点云显示

// SLAM 开关控制
std::atomic<bool> gEnableSLAM{true};  // 默认启用 SLAM

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

// ========== SLAM 系统访问的读写锁优化 ==========
static std::mutex gSlamPtrLock;                    // 仅保护 slamSys 指针（极短临界区）
static std::atomic<int> gProcessingFrames{0};      // 正在处理的帧数（用于写操作协调）
static std::condition_variable gCvProcessingFrames; // gProcessingFrames 归零时通知写操作
static std::mutex gTcwLock;                        // 保护 Tcw 缓存
static cv::Mat gCachedTcw;                         // 线程安全的 Tcw 缓存
static std::atomic<int> gCachedTrackingState{0};   // 无锁原子的跟踪状态缓存
std::map<int, Plane*> gMapPlanes;
std::map<int, std::vector<ArObjectInfo>> gMapArObjects;
int gActiveMapId = 0;
bool gMapSwitching = false;
int gMapSwitchCounter = 0;
const int MAP_SWITCH_THRESHOLD = ORB_SLAM2::MAP_SWITCH_THRESHOLD; // 至少连续3帧识别到新地图才切换

// AR对象渲染状态
std::atomic<bool> gShouldDrawArObject{false};
std::atomic<float> gArObjectScale{ORB_SLAM2::AR_OBJECT_SCALE_DEFAULT};  // 默认缩放
float gCurrentModelMatrix[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
float gCurrentViewMatrix[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
float gCurrentProjectionMatrix[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};

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
    std::lock_guard<std::mutex> lock(gMapDataMutex);

    std::string infoFile = filename + ".arinfo";
    std::ofstream ofs(infoFile, std::ios::binary);
    if (!ofs.is_open()) return;

    // 文件头：魔数和版本号
    const uint32_t magic = ORB_SLAM2::AR_INFO_FILE_MAGIC; // 'ARIN'（AR信息）
    const uint32_t version = ORB_SLAM2::AR_INFO_FILE_VERSION;
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
    auto numObjects = static_cast<uint32_t>(gArObjects.size());
    ofs.write(reinterpret_cast<const char*>(&numObjects), sizeof(numObjects));

    for (const auto& obj : gArObjects)
    {
        if (!obj.isValid) continue;

        ofs.write(reinterpret_cast<const char*>(obj.modelMatrix), sizeof(obj.modelMatrix));
        ofs.write(reinterpret_cast<const char*>(&obj.scale), sizeof(obj.scale));

        auto idLen = static_cast<uint32_t>(obj.objectId.length());
        ofs.write(reinterpret_cast<const char*>(&idLen), sizeof(idLen));

        if (idLen > 0)
        {
            ofs.write(obj.objectId.c_str(), static_cast<std::streamsize>(idLen));
        }
    }

    ofs.close();
    LOGD("保存平面和AR信息：已保存到%s", infoFile.c_str());
}

// 从文件加载平面和AR对象信息
void LoadPlaneAndArInfo(const std::string& filename, int mapId)
{

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

    if (magic != ORB_SLAM2::AR_INFO_FILE_MAGIC)
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
        ifs.read(reinterpret_cast<char*>(&obj.scale), sizeof(obj.scale));
        uint32_t idLen = 0;
        ifs.read(reinterpret_cast<char*>(&idLen), sizeof(idLen));
        if(idLen > 0) {
            std::vector<char> buf(idLen);
            ifs.read(buf.data(), static_cast<std::streamsize>(idLen));
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
    VT_PROFILE_FUNCTION(); // 跟踪主图像处理循环
    timeStamp += 1.0 / ORB_SLAM2::SYSTEM_FPS;

    // SLAM 开关控制：如果 SLAM 被关闭，跳过 SLAM 处理
    int status = 0;  // 默认状态：NO_IMAGES_YET

    if (!gEnableSLAM.load())
    {
        // SLAM 已关闭，不进行跟踪，直接返回状态
        status = 0;  // 返回 NO_IMAGES_YET 状态

        {
            std::lock_guard<std::mutex> lock(gMapPointsMutex);
            vMPs.clear();
            vKeys.clear();
        }

        gShouldDrawArObject.store(false);  // 关闭 AR 对象显示
    }
    else
    {
        // SLAM 正常运行

        const float DOWNSCALE = ORB_SLAM2::IMAGE_DOWNSCALE_FACTOR;
        // 使用静态线程局部变量复用内存，避免每帧 resize 时重新分配内存
        static thread_local cv::Mat imgSmall;
        if (image.empty()) {
            LOGE("processImage: 输入图像为空，跳帧处理");
            return 0;
        }
        const int scaledW = cvRound(static_cast<double>(image.cols) / DOWNSCALE);
        const int scaledH = cvRound(static_cast<double>(image.rows) / DOWNSCALE);
        cv::resize(image, imgSmall, cv::Size(scaledW, scaledH), 0, 0, cv::INTER_LINEAR);

        // ===== 读写锁优化：不再全程持有全局锁 =====
        ORB_SLAM2::System* currentSlamSys = nullptr;
        {
            std::lock_guard<std::mutex> _ptrLock(gSlamPtrLock);
            currentSlamSys = slamSys;
            if (currentSlamSys)
                gProcessingFrames.fetch_add(1, std::memory_order_relaxed);
        }

        // 使用线程局部 Tcw，避免全局 Tcw 的数据竞争
        cv::Mat localTcw;
        if(currentSlamSys) {
            // 执行跟踪（无全局锁！SLAM 系统内部锁保证线程安全）
            localTcw = currentSlamSys->TrackMonocular(imgSmall, timeStamp);
            int localStatus = currentSlamSys->GetTrackingState();

            // 线程安全地缓存跟踪结果，供其他 JNI 函数无锁读取
            {
                std::lock_guard<std::mutex> _tcwLock(gTcwLock);
                gCachedTcw = localTcw;  // 共享引用赋值 O(1)
            }
            gCachedTrackingState.store(localStatus, std::memory_order_relaxed);
            {
                std::lock_guard<std::mutex> _mpLock(gMapPointsMutex);
                vMPs = currentSlamSys->GetTrackedMapPoints();
                vKeys = currentSlamSys->GetTrackedKeyPointsUn();
            }

            status = localStatus;
            // 标记跟踪完成（写操作可通过 gProcessingFrames 感知）
            gProcessingFrames.fetch_sub(1, std::memory_order_release);
            gCvProcessingFrames.notify_one();  // 唤醒可能在等待的加载地图线程
        } else {
            {
                std::lock_guard<std::mutex> _tcwLock(gTcwLock);
                gCachedTcw = cv::Mat();
            }
            gCachedTrackingState.store(0, std::memory_order_relaxed);
            status = 0;
        }

        // 确保 vMPs 在任何情况下都处于安全状态

        // 确保slamSys仍然有效
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
        if(status == ORB_SLAM2::Tracking::OK) {  // SLAM正常工作
            // 如果有对齐，使用对齐后的位姿更新AR对象的视图矩阵
            cv::Mat TcwForAR = localTcw;
            if(slamSys->HasMapAlignment()) {
                TcwForAR = slamSys->GetMapAlignedPose(localTcw);
            }
            float tmpM[16];
            getColMajorMatrixFromMat(tmpM, TcwForAR);
            {
                std::lock_guard<std::mutex> dataLock(gMapDataMutex);
                getRUBViewMatrixFromRDF(tmpM, gCurrentViewMatrix);
            }
        }
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
            gShouldDrawArObject.store((status == ORB_SLAM2::Tracking::OK) && (pPlane != nullptr) && alignmentOK);
        }

        // 检测SLAM丢失状态并自动重置（保留加载的地图）
        // status: 0=NO_IMAGES_YET, 1=NOT_INITIALIZED, 2=OK, 3=LOST
        bool isLost = (status == ORB_SLAM2::Tracking::LOST);

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

        // 3D扫描建模与AR模式：仅在重定位/加载离线地图时显示历史地图点云 (drawOnlyLoaded = true 避免在线扫描点云持久化堆积)
        if(status==ORB_SLAM2::Tracking::OK) {
            if(gEnablePointCloudDisplay.load()) {
                // 获取相机位姿（若有地图对齐则使用对齐后的位姿）
                cv::Mat TcwForProjection = localTcw;
                if(slamSys->HasMapAlignment()) {
                    TcwForProjection = slamSys->GetMapAlignedPose(localTcw);
                }

                // 仅绘制已加载地图的固定点云，在线扫描新生成的点不进行画面持久化投影
                vector<ORB_SLAM2::MapPoint*> allMapPoints = slamSys->GetAllMapPoints();
                drawAllMapPoints(TcwForProjection, allMapPoints, outputImage, fx, fy, cx, cy, true);
            }
        }

        // 最后绘制跟踪到的特征点（蓝色点云）- 受点云显示开关控制
        if(gEnablePointCloudDisplay.load()) {
            drawTrackedPoints(vKeys,vMPs,outputImage);
        }
    }

    return status;
}

JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_initSLAM(JNIEnv* env, jobject instance, jstring path_)
{
    const char* path = env->GetStringUTFChars(path_, nullptr);

    if (slamInitialized) return;

    slamInitialized = true;
    modelPath = path;

    env->ReleaseStringUTFChars(path_, path);

    // 从Config.h加载相机参数 (基准值: 640x360校准)
    fx = ORB_SLAM2::CAMERA_FX;
    fy = ORB_SLAM2::CAMERA_FY;
    cx = ORB_SLAM2::CAMERA_CX;
    cy = ORB_SLAM2::CAMERA_CY;
    gBaseFx = fx;
    gBaseFy = fy;
    gBaseCx = cx;
    gBaseCy = cy;

    // 默认使用640x360 (初始未设置相机分辨率时)
    gScaledFx = fx;
    gScaledFy = fy;
    gScaledCx = cx;
    gScaledCy = cy;

    timeStamp = 0.0;

    // 预计算投影矩阵（基于缩放后的内参）
    frustumM_RUB((int)ORB_SLAM2::BASE_SLAM_WIDTH, (int)ORB_SLAM2::BASE_SLAM_HEIGHT, gScaledFx, gScaledFy, gScaledCx, gScaledCy, ORB_SLAM2::PROJECTION_ZNEAR, ORB_SLAM2::PROJECTION_ZFAR, gCurrentProjectionMatrix);

    // 初始化分析器 (仅在开发模式下生效)
    VT_PROFILE_INITIALIZE(std::string(path) + "/mentha_profile.bin");
    LOGD("Create SLAM System...");
    slamSys = new ORB_SLAM2::System("", ORB_SLAM2::System::MONOCULAR);
    slamSys->UpdateCalibration(fx, fy, cx, cy);
}

/**
 * 根据相机实际分辨率缩放内参
 * 基准内参在640x360下标定，按比例缩放到当前工作分辨率
 */
void updateScaledIntrinsics(int cameraWidth, int cameraHeight) {
    if (cameraWidth <= 0 || cameraHeight <= 0) return;

    // 内部SLAM工作分辨率 = 相机分辨率的一半
    int slamWidth = (int)(cameraWidth / ORB_SLAM2::IMAGE_DOWNSCALE_FACTOR);
    int slamHeight = (int)(cameraHeight / ORB_SLAM2::IMAGE_DOWNSCALE_FACTOR);
    if (slamWidth < 1) slamWidth = 1;
    if (slamHeight < 1) slamHeight = 1;

    // 按比例缩放内参
    float scaleX = (float)slamWidth / ORB_SLAM2::BASE_SLAM_WIDTH;
    float scaleY = (float)slamHeight / ORB_SLAM2::BASE_SLAM_HEIGHT;

    gScaledFx = gBaseFx * scaleX;
    gScaledFy = gBaseFy * scaleY;
    gScaledCx = gBaseCx * scaleX;
    gScaledCy = gBaseCy * scaleY;

    // 更新当前使用的内参
    fx = gScaledFx;
    fy = gScaledFy;
    cx = gScaledCx;
    cy = gScaledCy;
}

/**
 * JNI: 更新相机分辨率并重新计算内参和投影矩阵
 * 在相机启动或屏幕旋转时由Java层调用
 */
JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_nativeUpdateResolution(JNIEnv* env, jobject instance,
                                                               jint cameraWidth, jint cameraHeight) {
    updateScaledIntrinsics(cameraWidth, cameraHeight);

    int slamWidth = (int)(cameraWidth / ORB_SLAM2::IMAGE_DOWNSCALE_FACTOR);
    int slamHeight = (int)(cameraHeight / ORB_SLAM2::IMAGE_DOWNSCALE_FACTOR);
    if (slamWidth < 1) slamWidth = 1;
    if (slamHeight < 1) slamHeight = 1;

    // 重新计算投影矩阵
    frustumM_RUB(slamWidth, slamHeight, gScaledFx, gScaledFy,
                 gScaledCx, gScaledCy, ORB_SLAM2::PROJECTION_ZNEAR, ORB_SLAM2::PROJECTION_ZFAR, gCurrentProjectionMatrix);

    // 动态同步更新SLAM核心模块内的焦距与投影内参，防止尺度不匹配引发跟踪丢失
    if (slamSys) {
        slamSys->UpdateCalibration(gScaledFx, gScaledFy, gScaledCx, gScaledCy);
    }

    //      cameraWidth, cameraHeight, slamWidth, slamHeight,
    //      gScaledFx, gScaledFy, gScaledCx, gScaledCy);
}

JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_saveMap(JNIEnv* env, jobject instance, jstring path_, jint maxPoints)
{
    const char* path = env->GetStringUTFChars(path_, nullptr);

    if (slamSys)
    {
        auto t0 = static_cast<double>(cv::getTickCount());
        if (maxPoints > 0) {
            slamSys->SaveMap(std::string(path), maxPoints);
        } else {
            slamSys->SaveMap(std::string(path)); // 使用默认 SYSTEM_MAX_MPS_SAVE 上限
        }
        SavePlaneAndArInfo(std::string(path)); // 保存平面和AR信息

        auto t1 = static_cast<double>(cv::getTickCount());
        double ms = (t1 - t0) * 1000.0 / cv::getTickFrequency();
    }

    env->ReleaseStringUTFChars(path_, path);
}

JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_loadMapWithId(JNIEnv *env, jobject instance,
                                               jstring path_, jint mapId, jboolean append) {
    const char *path = env->GetStringUTFChars(path_, nullptr);
    if(slamSys){
        // 写锁协议：与 loadMap 一致
        std::unique_lock<std::mutex> lock(gSlamPtrLock);
        gCvProcessingFrames.wait(lock, []{
            return gProcessingFrames.load(std::memory_order_acquire) == 0;
        });

        auto t0 = static_cast<double>(cv::getTickCount());

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

        auto t1 = static_cast<double>(cv::getTickCount());
        double ms = (t1 - t0) * 1000.0 / cv::getTickFrequency();

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

    // statusBuf: [0]=tracking, [1]=shouldDraw, [2]=scaleBits
    statusBuf[0] = processImage(mGr, mRgba, statusBuf);
    statusBuf[1] = gShouldDrawArObject.load() ? 1 : 0;
    float scaleBits = gArObjectScale.load();
    std::memcpy(&statusBuf[2], &scaleBits, sizeof(scaleBits));

    env->ReleaseIntArrayElements(statusBuf_, statusBuf, 0);
}
JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_detect(JNIEnv *env, jobject instance,
                                               jintArray statusBuf_) {
    jint *statusBuf = env->GetIntArrayElements(statusBuf_, nullptr);

    // 从线程安全缓存读取最新 Tcw，无需阻塞跟踪线程
    cv::Mat currentTcw;
    {
        std::lock_guard<std::mutex> tcwLock(gTcwLock);
        currentTcw = gCachedTcw.clone();
    }

    // 同时也需要 gMapDataMutex 保护平面数据
    std::unique_lock<std::mutex> dataLock(gMapDataMutex, std::try_to_lock);
    if(!dataLock.owns_lock() || currentTcw.empty()){
        statusBuf[1] = ORB_SLAM2::PLANE_NOT_DETECTED;
        env->ReleaseIntArrayElements(statusBuf_, statusBuf, 0);
        return;
    }
    if(!currentTcw.empty()){
        // 平面检测也应该在对齐后的坐标系下进行（如果有对齐）
        cv::Mat TcwForPlane = currentTcw;
        if(slamSys->HasMapAlignment()) {
            TcwForPlane = slamSys->GetMapAlignedPose(currentTcw);
        }

        pPlane=detectPlane(TcwForPlane,vMPs,ORB_SLAM2::PLANE_DETECT_RANSAC_ITERS);
        if(pPlane && slamSys->MapChanged())
            pPlane->Recompute();
        statusBuf[1]=pPlane? ORB_SLAM2::PLANE_DETECTED : ORB_SLAM2::PLANE_NOT_DETECTED;

        // 检测到平面时更新AR对象矩阵
        if(pPlane) {
            planeLoadedFromMap = false;  // 手动检测的平面，标记为非地图加载

            // 更新模型矩阵（投影矩阵已在SLAM初始化时预计算）
            getRUBModelMatrixFromRDF(pPlane->glTpw, gCurrentModelMatrix);

        }
    }
    env->ReleaseIntArrayElements(statusBuf_, statusBuf, 0);
}

// 统一获取MVP矩阵（替代getM/getV/getP三个独立JNI）
JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_nativeGetMVP(JNIEnv *env, jobject instance,
    jfloatArray modelM_, jfloatArray viewM_, jfloatArray projM_, jint imageWidth, jint imageHeight)
{
    jfloat *modelM = env->GetFloatArrayElements(modelM_, nullptr);
    jfloat *viewM  = env->GetFloatArrayElements(viewM_, nullptr);
    jfloat *projM  = env->GetFloatArrayElements(projM_, nullptr);

    // Model
    {
        std::lock_guard<std::mutex> lock(gMapDataMutex);
        for(int i=0; i<16; i++) modelM[i] = gCurrentModelMatrix[i];
    }
    // View
    {
        bool useSlam = false;
        cv::Mat TcwForView;
        {
            std::lock_guard<std::mutex> tcwLock(gTcwLock);
            if(gCachedTrackingState.load(std::memory_order_relaxed)==2 && !gCachedTcw.empty()) {
                useSlam = true; TcwForView = gCachedTcw.clone();
            }
        }
        if(useSlam) {
            if(slamSys && slamSys->HasMapAlignment())
                TcwForView = slamSys->GetMapAlignedPose(TcwForView);
            float tmp[16]; getColMajorMatrixFromMat(tmp, TcwForView);
            getRUBViewMatrixFromRDF(tmp, viewM);
        } else {
            std::lock_guard<std::mutex> lk(gMapDataMutex);
            for(int i=0; i<16; i++) viewM[i] = gCurrentViewMatrix[i];
        }
    }
    // Projection
    frustumM_RUB((int)(imageWidth/ORB_SLAM2::IMAGE_DOWNSCALE_FACTOR), (int)(imageHeight/ORB_SLAM2::IMAGE_DOWNSCALE_FACTOR), fx, fy, cx, cy,
                 ORB_SLAM2::PROJECTION_ZNEAR, ORB_SLAM2::PROJECTION_ZFAR, projM);

    env->ReleaseFloatArrayElements(modelM_, modelM, 0);
    env->ReleaseFloatArrayElements(viewM_,  viewM,  0);
    env->ReleaseFloatArrayElements(projM_,  projM,  0);
}

// getV保留供WebServer使用
JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_getV(JNIEnv *env, jobject instance, jfloatArray viewM_) {
    jfloat *viewM = env->GetFloatArrayElements(viewM_, nullptr);
    bool useSlam = false; cv::Mat TcwForView;
    {
        std::lock_guard<std::mutex> tcwLock(gTcwLock);
        if(gCachedTrackingState.load(std::memory_order_relaxed)==2 && !gCachedTcw.empty()) {
            useSlam = true; TcwForView = gCachedTcw.clone();
        }
    }
    if(useSlam) {
        if(slamSys && slamSys->HasMapAlignment())
            TcwForView = slamSys->GetMapAlignedPose(TcwForView);
        float tmp[16]; getColMajorMatrixFromMat(tmp, TcwForView);
        getRUBViewMatrixFromRDF(tmp, viewM);
    } else {
        std::lock_guard<std::mutex> lk(gMapDataMutex);
        for(int i=0; i<16; i++) viewM[i] = gCurrentViewMatrix[i];
    }
    env->ReleaseFloatArrayElements(viewM_, viewM, 0);
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

JNIEXPORT jfloatArray JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_getMiniMapPoints(JNIEnv *env, jobject instance, jint maxPoints) {
    if(!slamSys) {
        return env->NewFloatArray(0);
    }
    std::vector<ORB_SLAM2::MapPoint*> v = slamSys->GetAllMapPoints();

    std::vector<float> out;
    size_t total = v.size();

    // 全地图均匀采样，确保全物体/多视角点云均匀保留，不丢弃旧视角点
    if (total > 0) {
        size_t limit = (maxPoints > 0 && (size_t)maxPoints < total) ? (size_t)maxPoints : total;
        size_t step = (total > limit) ? (total / limit) : 1;

        out.reserve(limit * 3);
        for(size_t i=0; i<total && out.size() < limit * 3; i += step) {
            ORB_SLAM2::MapPoint* p = v[i];
            if(!p || p->isBad()) continue;
            cv::Mat P = p->GetWorldPos();
            if(P.empty()) continue;
            out.push_back(P.at<float>(0));
            out.push_back(P.at<float>(1));
            out.push_back(P.at<float>(2));
        }
    }

    jfloatArray arr = env->NewFloatArray((jsize)out.size());
    if(arr && !out.empty()) env->SetFloatArrayRegion(arr, 0, (jsize)out.size(), out.data());
    return arr;
}

// 获取当前跟踪的地图点
JNIEXPORT jfloatArray JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_getTrackedPoints(JNIEnv *env, jobject instance, jint maxPoints) {
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
        cv::Point3f Pw;
        p->GetWorldPos(Pw);
        out.push_back(Pw.x);
        out.push_back(Pw.y);
        out.push_back(Pw.z);
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

// 更新AR对象缩放（当用户进行捏合缩放时从Java调用）
JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_updateArObjectScale(JNIEnv *env, jobject instance, jfloat scaleFactor) {
    float zoomFac = (scaleFactor - 1.0f) / ORB_SLAM2::AR_SCALE_ZOOM_DIVISOR;
    // 原子读-改-写（UI 线程写、分析线程读，消除数据竞争）
    float cur = gArObjectScale.load();
    cur += zoomFac;
    cur = fmax(ORB_SLAM2::AR_SCALE_MIN, cur);  // 最小缩放
    gArObjectScale.store(cur);
    LOGD("AR对象缩放已更新：%.3f", gArObjectScale.load());
}

// 获取当前AR对象缩放
JNIEXPORT jfloat JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_getArObjectScale(JNIEnv *env, jobject instance) {
    return gArObjectScale.load();
}

// 设置点云显示开关（控制绿色和蓝色点云）
JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_setPointCloudDisplay(JNIEnv *env, jobject instance, jboolean enable) {
    gEnablePointCloudDisplay.store((bool)enable);
    LOGD("点云显示模式：%s", gEnablePointCloudDisplay.load() ? "启用" : "禁用");
}

// 获取点云显示状态
JNIEXPORT jboolean JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_isPointCloudDisplayEnabled(JNIEnv *env, jobject instance) {
    return (jboolean)gEnablePointCloudDisplay.load();
}

// ========== SLAM 开关控制接口 ==========

// 启用/禁用 SLAM
JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_setEnableSLAM(JNIEnv *env, jobject instance, jboolean enable) {
    bool wasEnabled = gEnableSLAM.load();
    gEnableSLAM.store((bool)enable);

    if(wasEnabled && !gEnableSLAM.load()) {
        // SLAM 从开启变为关闭，清理相关状态
        {
            std::lock_guard<std::mutex> lock(gMapPointsMutex);
            vMPs.clear();
            vKeys.clear();
        }
        gShouldDrawArObject.store(false);
        LOGD("SLAM 已关闭");
    } else if(!wasEnabled && gEnableSLAM.load()) {
        // SLAM 从关闭变为开启
        LOGD("SLAM 已开启");
    }
}

// 获取 SLAM 启用状态
JNIEXPORT jboolean JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_isEnableSLAM(JNIEnv *env, jobject instance) {
    return (jboolean)gEnableSLAM.load();
}

}

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
#include <cstring>
#include <sys/mman.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>

#include "include/System.h"
#include "Common.h"
#include "Plane.h"
#include "UIUtils.h"
#include "Matrix.h"
#include "MapPoint.h"
#include "include/Config.h"
#include "MenthaProfiler.h"
#include "ArAnchor.h"

extern "C" {

// 全局变量声明
std::string modelPath;

ORB_SLAM2::System* slamSys;

float fx, fy, cx, cy;
float gBaseFx, gBaseFy, gBaseCx, gBaseCy;  // 基准内参 (640x360校准值)
float gScaledFx, gScaledFy, gScaledCx, gScaledCy;  // 缩放后的内参
double timeStamp;
bool slamInitialized = false;

std::vector<ORB_SLAM2::MapPoint*> vMPs;
std::vector<cv::KeyPoint> vKeys;

// 用于vMPs和vKeys线程安全访问的互斥锁
std::mutex gMapPointsMutex;

int gLoadedMapPointCount = 0;            // 加载的地图点数量

// 点云显示开关（同时控制绿色和蓝色点云）
bool gEnablePointCloudDisplay = true;  // 默认启用点云显示

// SLAM 开关控制
bool gEnableSLAM = true;  // 默认启用 SLAM


// SLAM丢失自动重置相关变量
double lastOkTime = 0.0;            // 上次SLAM正常工作的时间
bool wasLost = false;                // 上一帧是否处于LOST状态
const double LOST_RESET_TIMEOUT = ORB_SLAM2::LOST_RESET_TIMEOUT; // LOST状态持续3秒后重置

// ========== AR 锚点 ==========
AR::ArAnchor gAnchor;
std::map<int, AR::ArAnchor> gMapAnchors;
// 渲染层对齐滞回状态（与 SLAM 核心 mbHaveMapAlign 解耦，由 AR_RenderFrame 维护）
AR::AlignHoldState gAlignHold;
const int ALIGN_HOLD_FRAMES = 6;   // raw 对齐丢失后仍按"对齐帧"渲染的保持帧数（约0.1s@60fps）

// 多地图支持
std::mutex gMapDataMutex;

// ========== SLAM 系统访问的读写锁优化 ==========
static std::mutex gSlamPtrLock;                    // 仅保护 slamSys 指针（极短临界区）
static std::atomic<int> gProcessingFrames{0};      // 正在处理的帧数（用于写操作协调）
static std::condition_variable gCvProcessingFrames; // gProcessingFrames 归零时通知写操作
static std::mutex gTcwLock;                        // 保护 Tcw 缓存
static cv::Mat gCachedTcw;                         // 线程安全的 Tcw 缓存
int gActiveMapId = 0;
int gMapSwitchCounter = 0;
const int MAP_SWITCH_THRESHOLD = ORB_SLAM2::MAP_SWITCH_THRESHOLD; // 至少连续3帧识别到新地图才切换

// AR对象渲染状态
bool gShouldDrawArObject = false;
float gArObjectScale = ORB_SLAM2::AR_OBJECT_SCALE_DEFAULT;  // 默认缩放
float gCurrentModelMatrix[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
float gCurrentViewMatrix[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};

// ========== AR 锚点生命周期事件 ==========

// 重置渲染层对齐滞回状态
static void AR_ResetAlignHold() {
    gAlignHold.Reset();
}

/**
 * 事件：用户放置 AR 物体（detect 检测到平面后调用）
 */
static void AR_OnArPlaced(Plane* detected, bool whileAligned) {
    std::lock_guard<std::mutex> lk(gMapDataMutex);
    gAnchor.Reset();
    gAnchor.plane.reset(detected);   // 接管所有权；旧锚点自动释放
    gAnchor.frame = whileAligned ? AR::AnchorFrame::kMap : AR::AnchorFrame::kSlam;
    gAnchor.isFromLoadedMap = false;
    gAnchor.valid = true;
    AR_ResetAlignHold();             // 新锚点必须重新按当前帧渲染，避免冻结旧锚点的 lastGood
    if (gAnchor.plane) {
        getRUBModelMatrixFromRDF(gAnchor.plane->glTpw, gCurrentModelMatrix);
    }
}

/**
 * 事件：加载地图的 AR 数据
 */
static void AR_OnMapDataLoaded(int mapId, AR::ArAnchor loaded) {
    std::lock_guard<std::mutex> lk(gMapDataMutex);
    gMapAnchors[mapId] = std::move(loaded);   // 替换缓存，旧 Plane 由 unique_ptr 自动释放

    if (mapId == gActiveMapId) {
        if (!gMapAnchors[mapId].objects.empty()) {
            gAnchor = gMapAnchors[mapId].Clone();
            gAnchor.isFromLoadedMap = (gAnchor.plane != nullptr);
            gAnchor.frame = (gAnchor.plane != nullptr) ? AR::AnchorFrame::kMap : AR::AnchorFrame::kSlam;
            gAnchor.valid = (gAnchor.plane != nullptr || !gAnchor.objects.empty());
            AR_ResetAlignHold();   // 地图锚点必须重新对齐后才显示
            if (gAnchor.plane) {
                getRUBModelMatrixFromRDF(gAnchor.plane->glTpw, gCurrentModelMatrix);
            }
            LOGD("加载平面和AR信息：更新地图%d的当前显示状态", mapId);
        } else {
            LOGD("加载平面和AR信息：地图%d无AR物体，保留本地AR物体", mapId);
        }
    }
}

/**
 * 事件：确认切换到另一地图（processImage 切换阈值确认后调用）。
 * 保存当前锚点到旧地图缓存；目标地图自带 AR 物体 → 接管；否则保留本地锚点。
 */
static void AR_OnMapSwitched(int oldId, int newId) {
    std::lock_guard<std::mutex> lk(gMapDataMutex);
    gMapAnchors[oldId] = gAnchor.Clone();      // 保存当前锚点（含平面+物体+标志）到旧地图

    gActiveMapId = newId;

    if (gMapAnchors.count(newId) && !gMapAnchors[newId].objects.empty()) {
        gAnchor = gMapAnchors[newId].Clone();
        gAnchor.isFromLoadedMap = (gAnchor.plane != nullptr);
        gAnchor.frame = (gAnchor.plane != nullptr) ? AR::AnchorFrame::kMap : AR::AnchorFrame::kSlam;
        gAnchor.valid = (gAnchor.plane != nullptr || !gAnchor.objects.empty());
        AR_ResetAlignHold();                   // 目标为地图锚点时须重新对齐
        if (gAnchor.plane) {
            getRUBModelMatrixFromRDF(gAnchor.plane->glTpw, gCurrentModelMatrix);
        }
        LOGD("恢复地图%d的AR上下文", newId);
    }
    // 目标地图无 AR 物体：保留当前本地锚点
}

/**
 * 事件：SLAM 开关切换（setEnableSLAM 调用）。
 * 关闭 → 隐藏 AR 并重置滞回；几何保留，重新开启后恢复。
 */
static void AR_OnSlamToggle(bool enable) {
    if (!enable) {
        std::lock_guard<std::mutex> lk(gMapDataMutex);
        gShouldDrawArObject = false;
        AR_ResetAlignHold();
    }
}

/**
 * 每帧渲染管线：由 processImage 在 status==2 时调用。
 * @return 是否应绘制 AR 物体
 */
static bool AR_RenderFrame(const cv::Mat& localTcw, bool trackingOk) {
    std::lock_guard<std::mutex> lk(gMapDataMutex);
    const bool rawAligned = (slamSys && slamSys->HasMapAlignment());

    // ---- 1) 滞回：raw 对齐抖动不下穿 ----
    if (rawAligned) {
        gAlignHold.effAligned = true;
        gAlignHold.dropHold = 0;
    } else if (gAlignHold.effAligned && gAlignHold.dropHold < ALIGN_HOLD_FRAMES) {
        gAlignHold.dropHold++;   // 保持"对齐帧"，冻结 lastGood
    } else {
        gAlignHold.effAligned = false;
    }
    const bool usingHold = gAlignHold.effAligned && !rawAligned && gAlignHold.hasLastGood;

    // ---- 2) View：与 Model 严格同帧 ----
    float view[16];
    if (usingHold) {
        memcpy(view, gAlignHold.lastView, sizeof(view));   // 冻结最后对齐视图
    } else {
        cv::Mat TcwForAR = (gAlignHold.effAligned && rawAligned)
                               ? slamSys->GetMapAlignedPose(localTcw)
                               : localTcw;
        float tmp[16];
        getColMajorMatrixFromMat(tmp, TcwForAR);
        getRUBViewMatrixFromRDF(tmp, view);
    }

    // ---- 3) Model + 绘制门控 ----
    float model[16];
    setIdentityM(model);
    bool draw = trackingOk && gAnchor.valid && gAnchor.plane;
    if (draw) {
        if (usingHold) {
            memcpy(model, gAlignHold.lastModel, sizeof(model));   // 冻结最后对齐模型
        } else if (gAnchor.frame == AR::AnchorFrame::kMap) {
            if (gAlignHold.effAligned && rawAligned) {
                getRUBModelMatrixFromRDF(gAnchor.plane->glTpw, model);   // 地图锚点原始即地图帧
            } else {
                draw = false;   // 地图锚点无对齐时不可调和 → 隐藏而非画错位置
            }
        } else { // kSlam 本地锚点：两种帧都可画（view/model 同帧 → 数学上不变量成立）
            if (gAlignHold.effAligned && rawAligned) {
                cv::Mat alignedTpw = slamSys->GetMapAlignedPose(gAnchor.plane->Tpw);
                float tmp[16];
                getColMajorMatrixFromMat(tmp, alignedTpw);
                getRUBModelMatrixFromRDF(tmp, model);
            } else {
                getRUBModelMatrixFromRDF(gAnchor.plane->glTpw, model);
            }
        }
    }

    // ---- 4) 真对齐时缓存 lastGood（view/model 都在地图帧） ----
    if (gAlignHold.effAligned && rawAligned) {
        memcpy(gAlignHold.lastView, view, sizeof(view));
        memcpy(gAlignHold.lastModel, model, sizeof(model));
        gAlignHold.hasLastGood = true;
    }

    // ---- 5) 发布（nativeGetMVP / getV 仍读这两个缓存） ----
    memcpy(gCurrentViewMatrix, view, sizeof(view));
    memcpy(gCurrentModelMatrix, model, sizeof(model));
    return draw;
}


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
    const uint32_t magic = 0x4152494E; // 'ARIN'（AR信息）
    const uint32_t version = 1;
    ofs.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
    ofs.write(reinterpret_cast<const char*>(&version), sizeof(version));
    
    // 保存平面信息
    Plane* plane = gAnchor.plane.get();
    uint8_t hasPlane = (plane != nullptr) ? 1 : 0;
    ofs.write(reinterpret_cast<const char*>(&hasPlane), sizeof(hasPlane));

    if (plane)
    {
        // 保存平面的原点坐标和法向量
        float o3[3] = {plane->o.at<float>(0), plane->o.at<float>(1), plane->o.at<float>(2)};
        float n3[3] = {plane->n.at<float>(0), plane->n.at<float>(1), plane->n.at<float>(2)};
        ofs.write(reinterpret_cast<const char*>(o3), sizeof(o3));
        ofs.write(reinterpret_cast<const char*>(n3), sizeof(n3));
        ofs.write(reinterpret_cast<const char*>(&plane->rang), sizeof(plane->rang));
    }

    // 保存AR对象
    auto numObjects = static_cast<uint32_t>(gAnchor.objects.size());
    ofs.write(reinterpret_cast<const char*>(&numObjects), sizeof(numObjects));

    for (const auto& obj : gAnchor.objects)
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

    if (magic != 0x4152494E)
    {
        LOGE("加载平面和AR信息：错误的魔数");
        ifs.close();
        return;
    }

    // 解析出完整的 AR 锚点（坐标位于目标/地图坐标系）
    AR::ArAnchor loaded;

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
            loaded.plane = std::unique_ptr<Plane>(new Plane(n3[0], n3[1], n3[2], o3[0], o3[1], o3[2]));
            loaded.plane->rang = rang;
            loaded.frame = AR::AnchorFrame::kMap;   // 加载平面的坐标位于目标/地图坐标系
            LOGD("加载平面和AR信息：为地图%d加载平面", mapId);
        }
    }

    // 加载AR对象
    uint32_t numObjects = 0;
    ifs.read(reinterpret_cast<char*>(&numObjects), sizeof(numObjects));
    loaded.objects.reserve(numObjects);
    for(uint32_t i=0; i<numObjects; i++) {
        AR::ArObject obj;
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
        loaded.objects.push_back(obj);
    }
    ifs.close();
    loaded.valid = (loaded.plane != nullptr || !loaded.objects.empty());
    LOGD("加载平面和AR信息：为地图%d加载%d个AR对象", numObjects, mapId);

    // 生命周期事件：更新缓存与当前显示
    AR_OnMapDataLoaded(mapId, std::move(loaded));
}

int processImage(cv::Mat& image, cv::Mat& outputImage, int statusBuf[])
{
    VT_PROFILE_FUNCTION(); // 跟踪主图像处理循环
    timeStamp += 1.0 / ORB_SLAM2::SYSTEM_FPS;
    
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

            // 线程安全地缓存跟踪结果，供 detect 等 JNI 函数无锁读取
            {
                std::lock_guard<std::mutex> _tcwLock(gTcwLock);
                gCachedTcw = localTcw.clone();
            }
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
                LOGD("检测到并确认地图切换：%d -> %d", gActiveMapId, currentMapId);
                // 生命周期事件：统一处理"保存旧锚点 / 接管新锚点"规则
                AR_OnMapSwitched(gActiveMapId, currentMapId);
                gMapSwitchCounter = 0;
            }
        } else {
            gMapSwitchCounter = 0;
        }

        // 如果SLAM正在跟踪，更新AR对象视图/模型矩阵
        if(!localTcw.empty()) {
            gShouldDrawArObject = AR_RenderFrame(localTcw, (status == 2));
        } else {
            gShouldDrawArObject = false;
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
                    //LOGD("SLAM已丢失%.1f秒，执行轻量重置（保留加载的地图）...", lostDuration);
                    slamSys->Reset(true);  // 保留地图的重置
                    wasLost = false;
                    lastOkTime = timeStamp;
                    //LOGD("SLAM轻量重置完成，已加载的地图数据已保留");
                }
            }
        }
        
        // AR模式：显示完整地图点云（绿色）
        if(status==2) {
            // 一旦对齐成功，立即显示完整地图点云，无需等待新点数量
            bool hasAlignment = slamSys->HasMapAlignment();

            if(hasAlignment)
            {
                // 获取对齐后的相机位姿（在地图坐标系下）
                cv::Mat TcwForProjection = slamSys->GetMapAlignedPose(localTcw);
                
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
    }

    //cv::imwrite(modelPath+"/lala2.jpg",outputImage);
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

    // 投影矩阵由 nativeGetMVP 每帧现场计算，无需预存全局（gCurrentProjectionMatrix 已删除）

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
    int slamWidth = cameraWidth / 2;
    int slamHeight = cameraHeight / 2;
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

    int slamWidth = cameraWidth / 2;
    int slamHeight = cameraHeight / 2;
    if (slamWidth < 1) slamWidth = 1;
    if (slamHeight < 1) slamHeight = 1;

    // 投影矩阵由 nativeGetMVP 每帧现场计算，无需预存全局（gCurrentProjectionMatrix 已删除）

    // 动态同步更新SLAM核心模块内的焦距与投影内参，防止尺度不匹配引发跟踪丢失
    if (slamSys) {
        slamSys->UpdateCalibration(gScaledFx, gScaledFy, gScaledCx, gScaledCy);
    }

    //      cameraWidth, cameraHeight, slamWidth, slamHeight,
    //      gScaledFx, gScaledFy, gScaledCx, gScaledCy);
}

JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_saveMap(JNIEnv* env, jobject instance, jstring path_)
{
    const char* path = env->GetStringUTFChars(path_, nullptr);

    if (slamSys)
    {

        auto t0 = static_cast<double>(cv::getTickCount());
        slamSys->SaveMap(std::string(path));
        SavePlaneAndArInfo(std::string(path)); // 保存平面和AR信息

        auto t1 = static_cast<double>(cv::getTickCount());
        double ms = (t1 - t0) * 1000.0 / cv::getTickFrequency();
        
        //     slamSys->GetNumKeyFrames(), slamSys->GetNumMapPoints());
    }
    
    env->ReleaseStringUTFChars(path_, path);
}

JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_loadMap(JNIEnv* env, jobject instance, jstring path_)
{
    const char* path = env->GetStringUTFChars(path_, nullptr);

    if (slamSys)
    {
        // 写锁协议：
        // 1. 持有 gSlamPtrLock → 阻止新的帧处理开始
        // 2. 等待 gProcessingFrames 归零 → 等待正在处理中的帧完成
        // 注意：此锁只在用户手动加载地图时短暂持有一两秒，不影响正常跟踪流程
        std::unique_lock<std::mutex> lock(gSlamPtrLock);

        // 等待正在处理中的帧完成（它们在 lock 获取前就已开始了 TrackMonocular）
        // 这些帧完成后会在 gSlamPtrLock 锁外自动递减 gProcessingFrames，
        // 因此即使我们持有 gSlamPtrLock，它们也能正常完成
        gCvProcessingFrames.wait(lock, []{
            return gProcessingFrames.load(std::memory_order_acquire) == 0;
        });
        // 此时：gProcessingFrames == 0，gSlamPtrLock 被持有
        // 新的 processImage 被阻塞在 gSlamPtrLock 上

        LOGD("JNI加载地图开始：%s", path);

        auto t0 = static_cast<double>(cv::getTickCount());
        slamSys->LoadMap(std::string(path), 0, false); // 默认ID=0，覆盖模式
        LoadPlaneAndArInfo(std::string(path), 0); // 加载平面和AR信息

        auto t1 = static_cast<double>(cv::getTickCount());
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
        
        //     slamSys->GetNumKeyFrames(), slamSys->GetNumMapPoints(), gLoadedMapPointCount);
        
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
        
        // 如果不是追加模式，清理旧的全局数据（含当前锚点，修复"加载新地图后残留旧地图平面"的陈旧 bug）
        if (!append) {
             std::lock_guard<std::mutex> lock(gMapDataMutex);
             gMapAnchors.clear();
             gAnchor.Reset();
             gActiveMapId = mapId; // 强制设置活跃地图ID
             AR_ResetAlignHold();
             // 注意：System::LoadMap(append=false) 会清理地图点，这里同步清理关联的 AR 数据
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
    JNIEnv* env, jobject instance, jlong matGrPtr, jlong matRgbaPtr, jintArray statusBuf_)
{
    jint* statusBuf = env->GetIntArrayElements(statusBuf_, nullptr);

    cv::Mat& mGr = *(cv::Mat*)matGrPtr;
    cv::Mat& mRgba = *(cv::Mat*)matRgbaPtr;

    // statusBuf: [0]=tracking, [1]=shouldDraw, [2]=scaleBits
    statusBuf[0] = processImage(mGr, mRgba, statusBuf);
    statusBuf[1] = gShouldDrawArObject ? 1 : 0;
    statusBuf[2] = *reinterpret_cast<jint*>(&gArObjectScale);

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

        Plane* detected = detectPlane(TcwForPlane, vMPs, ORB_SLAM2::PLANE_DETECT_RANSAC_ITERS);
        if(detected && slamSys->MapChanged())
            detected->Recompute();
        statusBuf[1]=detected? ORB_SLAM2::PLANE_DETECTED : ORB_SLAM2::PLANE_NOT_DETECTED;

        // 检测到平面 → 生命周期事件：接管为当前锚点
        if(detected) {
            dataLock.unlock();   // 先释放本函数持有的 gMapDataMutex，避免 AR_OnArPlaced 重入死锁
            AR_OnArPlaced(detected, slamSys->HasMapAlignment());
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

    // Model + View
    {
        std::lock_guard<std::mutex> lock(gMapDataMutex);
        for(int i=0; i<16; i++) modelM[i] = gCurrentModelMatrix[i];
        for(int i=0; i<16; i++) viewM[i]  = gCurrentViewMatrix[i];
    }
    // Projection
    frustumM_RUB(imageWidth/2, imageHeight/2, fx, fy, cx, cy,
                 ORB_SLAM2::PROJECTION_ZNEAR, ORB_SLAM2::PROJECTION_ZFAR, projM);

    env->ReleaseFloatArrayElements(modelM_, modelM, 0);
    env->ReleaseFloatArrayElements(viewM_,  viewM,  0);
    env->ReleaseFloatArrayElements(projM_,  projM,  0);
}

// getV保留供WebServer使用
JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_getV(JNIEnv *env, jobject instance, jfloatArray viewM_) {
    jfloat *viewM = env->GetFloatArrayElements(viewM_, nullptr);
    {
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
        stats[2] = (gAnchor.plane != nullptr) ? 1 : 0;
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
    // 直接读取 slamSys（Map 内部已有 mMutexMap 保护 GetAllMapPoints）
    // 无需 gSlamStateMutex，不再阻塞跟踪线程
    if(!slamSys) {
        return env->NewFloatArray(0);
    }
    std::vector<ORB_SLAM2::MapPoint*> v = slamSys->GetAllMapPoints();
    
    std::vector<float> out;
    size_t total = v.size();
    
    // 使用 Top-K 算法获取最新的 maxPoints 个点（按 ID 降序）
    
    if (total > 0) {
        if (total > (size_t)maxPoints) {
            // 使用 partial_sort 找出前 maxPoints 个最大的（最新的）点
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
    data.push_back((float)gAnchor.objects.size());

    for(const auto& obj : gAnchor.objects) {
        if(!obj.isValid) continue;
        for(float m : obj.modelMatrix) {
            data.push_back(m);
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
    float zoomFac = (scaleFactor - 1.0f) / 5.0f;
    gArObjectScale += zoomFac;
    gArObjectScale = fmax(0.03f, gArObjectScale);  // 最小缩放
    //LOGD("AR对象缩放已更新：%.3f", gArObjectScale);
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
    //LOGD("点云显示模式：%s", gEnablePointCloudDisplay ? "启用" : "禁用");
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
        // 生命周期事件：隐藏 AR 并重置对齐滞回
        AR_OnSlamToggle(false);
    } else if(!wasEnabled && gEnableSLAM) {
        // SLAM 从关闭变为开启
        //LOGD("SLAM 已开启");
    }
}

// 获取 SLAM 启用状态
JNIEXPORT jboolean JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_isEnableSLAM(JNIEnv *env, jobject instance) {
    return (jboolean)gEnableSLAM;
}

// ========== 共享内存帧持久映射（仅 attach 一次，避免每帧 mmap/munmap 与 fd 传输） ==========
static void* gSharedFramePtr = nullptr;
static int gSharedFrameSize = 0;
static std::mutex gSharedFrameLock;

// 持久映射共享内存帧缓冲。仅在缓冲（重新）创建后由 attachFrameBuffer 调用。
JNIEXPORT jboolean JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_nativeAttachFrameBuffer(
    JNIEnv* env, jobject instance, jint fd, jint size)
{
    std::lock_guard<std::mutex> lock(gSharedFrameLock);
    if (gSharedFramePtr) {
        munmap(gSharedFramePtr, gSharedFrameSize);
        gSharedFramePtr = nullptr;
        gSharedFrameSize = 0;
    }
    if (fd < 0 || size <= 0) return JNI_FALSE;

    void* mappedPtr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (mappedPtr == MAP_FAILED) {
        LOGE("nativeAttachFrameBuffer: mmap 映射内存失败");
        return JNI_FALSE;
    }
    gSharedFramePtr = mappedPtr;
    gSharedFrameSize = size;
    return JNI_TRUE;
}

// 解除持久映射（服务销毁/解绑时调用）
JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_nativeDetachFrameBuffer(JNIEnv* env, jobject instance)
{
    std::lock_guard<std::mutex> lock(gSharedFrameLock);
    if (gSharedFramePtr) {
        munmap(gSharedFramePtr, gSharedFrameSize);
        gSharedFramePtr = nullptr;
        gSharedFrameSize = 0;
    }
}

// 处理持久映射缓冲中的最新一帧：SLAM 结果（绿/蓝点云）直接绘制回该共享内存供主界面读取。
// statusBuf[0]=tracking, statusBuf[1]=shouldDraw（沿用 processImage 内 gShouldDrawArObject
// 语义：需跟踪正常 + 平面存在 + 对齐成功，避免未检测平面时误绘制）。
JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_nativeProcessFrameSharedMem(
    JNIEnv* env, jobject instance, jint width, jint height, jintArray statusBuf_)
{
    if (width <= 0 || height <= 0) return;
    jint* statusBuf = env->GetIntArrayElements(statusBuf_, nullptr);
    if (!statusBuf) return;

    std::lock_guard<std::mutex> lock(gSharedFrameLock);
    if (!gSharedFramePtr || gSharedFrameSize < width * height * 4) {
        LOGE("nativeProcessFrameSharedMem: 共享内存未映射或尺寸不足");
        statusBuf[0] = 0;
        statusBuf[1] = 0;
        env->ReleaseIntArrayElements(statusBuf_, statusBuf, 0);
        return;
    }

    cv::Mat mRgba(height, width, CV_8UC4, gSharedFramePtr);
    cv::Mat mGr;
    cv::cvtColor(mRgba, mGr, cv::COLOR_RGBA2GRAY);

    int tmpStatus[3] = {0};
    int trackingResult = processImage(mGr, mRgba, tmpStatus);
    statusBuf[0] = trackingResult;
    statusBuf[1] = gShouldDrawArObject ? 1 : 0;

    env->ReleaseIntArrayElements(statusBuf_, statusBuf, 0);
}

}

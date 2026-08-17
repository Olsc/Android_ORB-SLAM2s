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
#include <sched.h>
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

// 点云显示开关（同时控制绿色和蓝色点云）：由 binder/UI 线程写、SLAM 线程读，保持原子量
std::atomic<bool> gEnablePointCloudDisplay{true};  // 默认启用点云显示

// SLAM丢失自动重置相关变量
double lastOkTime = 0.0;            // 上次SLAM正常工作的时间
bool wasLost = false;                // 上一帧是否处于LOST状态
const double LOST_RESET_TIMEOUT = ORB_SLAM2::LOST_RESET_TIMEOUT; // 名义超时（秒），仅用于换算帧数
int gLostFrameCount = 0;             // 连续丢失帧计数

// AR 锚点
AR::ArAnchor gAnchor;
std::map<int, AR::ArAnchor> gMapAnchors;
// 渲染层对齐滞回状态（与 SLAM 核心 mbHaveMapAlign 解耦，由 AR_RenderFrame 维护）
AR::AlignHoldState gAlignHold;
const int ALIGN_HOLD_FRAMES = 6;   // raw 对齐丢失后仍按"对齐帧"渲染的保持帧数（约0.1s@60fps）

// 多地图支持
std::mutex gMapDataMutex;

// SLAM 系统访问的读写锁
static std::mutex gSlamPtrLock;                    // 仅保护 slamSys 指针（极短临界区）
static std::atomic<int> gProcessingFrames{0};      // 正在处理的帧数（用于写操作协调）
static std::condition_variable gCvProcessingFrames; // gProcessingFrames 归零时通知写操作
static std::mutex gTcwLock;                        // 保护 Tcw 缓存
static cv::Mat gCachedTcw;                         // 线程安全的 Tcw 缓存
// 跟踪状态快照原子量：GL/Web 线程读、SLAM 线程写
static std::atomic<int> gCachedTrackingState{0};   // 最近跟踪状态快照（0=NO_IMAGES_YET）
int gActiveMapId = 0;
int gMapSwitchCounter = 0;
const int MAP_SWITCH_THRESHOLD = ORB_SLAM2::MAP_SWITCH_THRESHOLD; // 至少连续3帧识别到新地图才切换

// AR对象渲染状态（跨线程读写：SLAM 线程产、binder/GL 线程读——保持原子）
static std::atomic<bool> gShouldDrawArObject{false};
static std::atomic<float> gArObjectScale{ORB_SLAM2::AR_OBJECT_SCALE_DEFAULT};  // 默认缩放
float gCurrentModelMatrix[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
float gCurrentViewMatrix[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};

// AR 锚点生命周期事件

// 重置渲染层对齐滞回状态
static void AR_ResetAlignHold() {
    gAlignHold.Reset();
}

// 事件：用户放置 AR 物体（detect 检测到平面后调用）
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
        AR::ArObject obj;
        memcpy(obj.modelMatrix, gCurrentModelMatrix, sizeof(obj.modelMatrix));
        obj.scale = gArObjectScale.load(std::memory_order_relaxed);
        obj.isValid = true;
        obj.objectId = "default";
        gAnchor.objects.push_back(obj);
    }
}

// 事件：加载地图的 AR 数据
static void AR_OnMapDataLoaded(int mapId, AR::ArAnchor loaded) {
    std::lock_guard<std::mutex> lk(gMapDataMutex);
    gMapAnchors[mapId] = std::move(loaded);   // 替换缓存，旧 Plane 由 unique_ptr 自动释放

    if (mapId == gActiveMapId) {
        if (gMapAnchors[mapId].valid || gMapAnchors[mapId].plane != nullptr || !gMapAnchors[mapId].objects.empty()) {
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

// 事件：确认切换到另一地图（processImage 切换阈值确认后调用）
// 保存当前锚点到旧地图缓存；目标地图自带 AR 物体则接管，否则保留本地锚点
static void AR_OnMapSwitched(int oldId, int newId) {
    std::lock_guard<std::mutex> lk(gMapDataMutex);
    gMapAnchors[oldId] = gAnchor.Clone();      // 保存当前锚点（含平面+物体+标志）到旧地图

    gActiveMapId = newId;

    if (gMapAnchors.count(newId) && (gMapAnchors[newId].valid || gMapAnchors[newId].plane != nullptr || !gMapAnchors[newId].objects.empty())) {
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

// 每帧渲染管线：由 processImage 在 status==2 时调用，返回是否应绘制 AR 物体
static bool AR_RenderFrame(const cv::Mat& localTcw, bool trackingOk) {
    std::lock_guard<std::mutex> lk(gMapDataMutex);
    const bool rawAligned = (slamSys && slamSys->HasMapAlignment());

    // 1) 滞回：raw 对齐抖动不下穿
    if (rawAligned) {
        gAlignHold.effAligned = true;
        gAlignHold.dropHold = 0;
    } else if (gAlignHold.effAligned && gAlignHold.dropHold < ALIGN_HOLD_FRAMES) {
        gAlignHold.dropHold++;   // 保持"对齐帧"，冻结 lastGood
    } else {
        gAlignHold.effAligned = false;
    }
    const bool usingHold = gAlignHold.effAligned && !rawAligned && gAlignHold.hasLastGood;

    // 2) View：与 Model 严格同帧
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

    // 3) Model + 绘制门控
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

    // 4) 真对齐时缓存 lastGood（view/model 都在地图帧）
    if (gAlignHold.effAligned && rawAligned) {
        memcpy(gAlignHold.lastView, view, sizeof(view));
        memcpy(gAlignHold.lastModel, model, sizeof(model));
        gAlignHold.hasLastGood = true;
    }

    // 5) 发布
    memcpy(gCurrentViewMatrix, view, sizeof(view));
    memcpy(gCurrentModelMatrix, model, sizeof(model));
    return draw;
}

// 保存平面和AR对象信息到 .arinfo 文件，与SLAM地图配套保存
void SavePlaneAndArInfo(const std::string& filename)
{
    std::lock_guard<std::mutex> lock(gMapDataMutex);

    std::string infoFile = filename + ".arinfo";
    std::ofstream ofs(infoFile, std::ios::binary);
    if (!ofs.is_open()) return;

    // 文件头：魔数和版本号
    const uint32_t magic = ORB_SLAM2::AR_INFO_FILE_MAGIC;
    const uint32_t version = ORB_SLAM2::AR_INFO_FILE_VERSION;
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
    std::vector<AR::ArObject> objectsToSave = gAnchor.objects;
    if (objectsToSave.empty() && plane) {
        AR::ArObject defaultObj;
        getRUBModelMatrixFromRDF(plane->glTpw, defaultObj.modelMatrix);
        defaultObj.scale = gArObjectScale.load(std::memory_order_relaxed);
        defaultObj.isValid = true;
        defaultObj.objectId = "default";
        objectsToSave.push_back(defaultObj);
    }
    auto numObjects = static_cast<uint32_t>(objectsToSave.size());
    ofs.write(reinterpret_cast<const char*>(&numObjects), sizeof(numObjects));

    for (const auto& obj : objectsToSave)
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
            loaded.plane = std::unique_ptr<Plane>(new Plane(n3[0], n3[1], n3[2], o3[0], o3[1], o3[2], rang));
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
    VT_PROFILE_FUNCTION();
    timeStamp += 1.0 / ORB_SLAM2::SYSTEM_FPS;

    int status = 0;

    const float DOWNSCALE = ORB_SLAM2::IMAGE_DOWNSCALE_FACTOR;
    static thread_local cv::Mat imgSmall;
    if (image.empty()) {
        LOGE("processImage: 输入图像为空，跳帧处理");
        return 0;
    }
    const int scaledW = cvRound(static_cast<double>(image.cols) / DOWNSCALE);
    const int scaledH = cvRound(static_cast<double>(image.rows) / DOWNSCALE);
    cv::resize(image, imgSmall, cv::Size(scaledW, scaledH), 0, 0, cv::INTER_LINEAR);

    // 确保内参与实际 SLAM 分辨率匹配
    static int sLastSlamW = 0, sLastSlamH = 0;
    if (imgSmall.cols != sLastSlamW || imgSmall.rows != sLastSlamH) {
        const float calScaleX = (float)imgSmall.cols / ORB_SLAM2::BASE_SLAM_WIDTH;
        const float calScaleY = (float)imgSmall.rows / ORB_SLAM2::BASE_SLAM_HEIGHT;
        gScaledFx = gBaseFx * calScaleX;
        gScaledFy = gBaseFy * calScaleY;
        gScaledCx = gBaseCx * calScaleX;
        gScaledCy = gBaseCy * calScaleY;
        fx = gScaledFx; fy = gScaledFy; cx = gScaledCx; cy = gScaledCy;
        if (slamSys) {
            slamSys->UpdateCalibration(fx, fy, cx, cy);
        }
        sLastSlamW = imgSmall.cols;
        sLastSlamH = imgSmall.rows;
        LOGD("SLAM 内参校准: 分辨率=%dx%d fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
             imgSmall.cols, imgSmall.rows, fx, fy, cx, cy);
    }

    ORB_SLAM2::System* currentSlamSys = nullptr;
    {
        std::lock_guard<std::mutex> _ptrLock(gSlamPtrLock);
        currentSlamSys = slamSys;
        if (currentSlamSys)
            gProcessingFrames.fetch_add(1, std::memory_order_relaxed);
    }

    // RAII 引用计数：无论从哪个 return 退出，都在"本函数完全结束"时才递减并唤醒
    // 等待的写操作（LoadMap 等）。原先在中途递减后仍继续访问 slamSys，与写方
    // "计数归零即可安全写"的协议存在悬空窗口。
    const bool bHoldsFrameRef = (currentSlamSys != nullptr);
    struct FrameRefGuard {
        const bool armed;
        explicit FrameRefGuard(bool a) : armed(a) {}
        ~FrameRefGuard() {
            if (armed) {
                gProcessingFrames.fetch_sub(1, std::memory_order_release);
                gCvProcessingFrames.notify_one();
            }
        }
    } frameRefGuard(bHoldsFrameRef);

    // 使用线程局部 Tcw，避免全局 Tcw 的数据竞争
    cv::Mat localTcw;
    if(currentSlamSys) {
        localTcw = currentSlamSys->TrackMonocular(imgSmall, timeStamp);
        int localStatus = currentSlamSys->GetTrackingState();

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
        // 注意：gProcessingFrames 的递减已移至 processImage 完全结束处。
        // 原先在这里递减后函数仍继续访问 slamSys（GetCurrentMapId/HasMapAlignment/
        // GetAllMapPoints），与 LoadMap 的"等待计数归零后写地图"协议存在悬空窗口。
    } else {
        {
            std::lock_guard<std::mutex> _tcwLock(gTcwLock);
            gCachedTcw = cv::Mat();
        }
        status = 0;
    }

    // 确保 vMPs 在任何情况下都处于安全状态

    // 使用锁内快照 currentSlamSys（持有帧引用计数期间对象保证存活），
    // 原先在此无锁读取全局 slamSys 属数据竞争
    if(!currentSlamSys) {
        return status;
    }

    // 检查是否切换了地图
    int currentMapId = currentSlamSys->GetCurrentMapId();
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

    bool isLost = (status == ORB_SLAM2::Tracking::LOST);
    const int LOST_RESET_FRAMES = (int)(LOST_RESET_TIMEOUT * ORB_SLAM2::SYSTEM_FPS);

    if(!isLost) {
        lastOkTime = timeStamp;
        wasLost = false;
        gLostFrameCount = 0;
    } else {
        // SLAM处于LOST状态
        if(++gLostFrameCount >= LOST_RESET_FRAMES) {
            LOGD("SLAM连续丢失 %d 帧，执行轻量重置（保留加载的地图）...", gLostFrameCount);
            currentSlamSys->Reset(true);  // 保留地图的重置
            wasLost = false;
            lastOkTime = timeStamp;
            gLostFrameCount = 0;
            LOGD("SLAM轻量重置完成，已加载的地图数据已保留");
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

    fx = ORB_SLAM2::CAMERA_FX;
    fy = ORB_SLAM2::CAMERA_FY;
    cx = ORB_SLAM2::CAMERA_CX;
    cy = ORB_SLAM2::CAMERA_CY;
    gBaseFx = fx;
    gBaseFy = fy;
    gBaseCx = cx;
    gBaseCy = cy;

    gScaledFx = fx;
    gScaledFy = fy;
    gScaledCx = cx;
    gScaledCy = cy;

    timeStamp = 0.0;

    VT_PROFILE_INITIALIZE(std::string(path) + "/mentha_profile.bin");
    LOGD("Create SLAM System...");
    // 进程亲和性诊断：SLAM 独立进程（:slam_process）可能被分配受限 cpuset，
    // 启动时打印可用核数便于排查调度受限问题
    {
        char cpuset[128] = {0};
        FILE* f = fopen("/proc/self/cpuset", "r");
        if (f) {
            size_t n = fread(cpuset, 1, sizeof(cpuset) - 1, f);
            cpuset[n] = 0;
            fclose(f);
        }
        cpu_set_t cs;
        CPU_ZERO(&cs);
        if (sched_getaffinity(0, sizeof(cs), &cs) == 0) {
            int nCpu = 0;
            for (int i = 0; i < CPU_SETSIZE; ++i) if (CPU_ISSET(i, &cs)) nCpu++;
            LOGD("SLAM 进程 cpuset=%s 可用核数=%d", cpuset, nCpu);
        } else {
            LOGD("SLAM 进程 cpuset=%s 可用核数=未知", cpuset);
        }
    }

    // 构造与首次校准在 gSlamPtrLock 内发布：均为启动期一次性操作，
    // 此时相机/GL 尚未调用其他 JNI 入口，持锁无争用
    {
        std::lock_guard<std::mutex> ptrLock(gSlamPtrLock);
        slamSys = new ORB_SLAM2::System("", ORB_SLAM2::System::MONOCULAR);
        slamSys->UpdateCalibration(fx, fy, cx, cy);
    }
}

void updateScaledIntrinsics(int cameraWidth, int cameraHeight) {
    if (cameraWidth <= 0 || cameraHeight <= 0) return;

    int slamWidth = cvRound((float)cameraWidth / ORB_SLAM2::IMAGE_DOWNSCALE_FACTOR);
    int slamHeight = cvRound((float)cameraHeight / ORB_SLAM2::IMAGE_DOWNSCALE_FACTOR);
    if (slamWidth < 1) slamWidth = 1;
    if (slamHeight < 1) slamHeight = 1;

    float scaleX = (float)slamWidth / ORB_SLAM2::BASE_SLAM_WIDTH;
    float scaleY = (float)slamHeight / ORB_SLAM2::BASE_SLAM_HEIGHT;

    gScaledFx = gBaseFx * scaleX;
    gScaledFy = gBaseFy * scaleY;
    gScaledCx = gBaseCx * scaleX;
    gScaledCy = gBaseCy * scaleY;

    fx = gScaledFx;
    fy = gScaledFy;
    cx = gScaledCx;
    cy = gScaledCy;
}

JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_nativeUpdateResolution(JNIEnv* env, jobject instance,
                                                               jint cameraWidth, jint cameraHeight) {
    updateScaledIntrinsics(cameraWidth, cameraHeight);

    int slamWidth = cvRound((float)cameraWidth / ORB_SLAM2::IMAGE_DOWNSCALE_FACTOR);
    int slamHeight = cvRound((float)cameraHeight / ORB_SLAM2::IMAGE_DOWNSCALE_FACTOR);
    if (slamWidth < 1) slamWidth = 1;
    if (slamHeight < 1) slamHeight = 1;

    // 动态同步更新SLAM核心模块内的焦距与投影内参，防止尺度不匹配引发跟踪丢失
    {
        // 快路径调用持 gSlamPtrLock，与 nativeShutdown 的 delete 互斥
        std::lock_guard<std::mutex> ptrLock(gSlamPtrLock);
        if (slamSys) {
            slamSys->UpdateCalibration(gScaledFx, gScaledFy, gScaledCx, gScaledCy);
        }
    }
}

JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_saveMap(JNIEnv* env, jobject instance, jstring path_)
{
    const char* path = env->GetStringUTFChars(path_, nullptr);

    // 锁内快照：saveMap 为 UI 线程长操作，不在 gSlamPtrLock 内执行，
    // 避免长时间阻塞帧处理线程的指针快照
    ORB_SLAM2::System* sys = nullptr;
    {
        std::lock_guard<std::mutex> ptrLock(gSlamPtrLock);
        sys = slamSys;
    }

    if (sys)
    {
        auto t0 = static_cast<double>(cv::getTickCount());
        sys->SaveMap(std::string(path)); // 使用默认 SYSTEM_MAX_MPS_SAVE 上限
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
        std::unique_lock<std::mutex> lock(gSlamPtrLock);
        gCvProcessingFrames.wait(lock, []{
            return gProcessingFrames.load(std::memory_order_acquire) == 0;
        });

        auto t0 = static_cast<double>(cv::getTickCount());

        if (!append) {
             std::lock_guard<std::mutex> lock(gMapDataMutex);
             gMapAnchors.clear();
             gAnchor.Reset();
             gActiveMapId = mapId;
             AR_ResetAlignHold();
        }

        slamSys->LoadMap(std::string(path), mapId, append);
        LoadPlaneAndArInfo(std::string(path), mapId);

        auto t1 = static_cast<double>(cv::getTickCount());
        double ms = (t1 - t0) * 1000.0 / cv::getTickFrequency();
    }
    env->ReleaseStringUTFChars(path_, path);
}

JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_detect(JNIEnv *env, jobject instance,
                                               jintArray statusBuf_) {
    jint *statusBuf = env->GetIntArrayElements(statusBuf_, nullptr);

    // 锁内快照 System*：本函数与 nativeShutdown 同为 UI 线程，不并发，
    // 快照消除对全局指针的未同步读；锁序 gSlamPtrLock → gMapDataMutex
    ORB_SLAM2::System* sys = nullptr;
    {
        std::lock_guard<std::mutex> ptrLock(gSlamPtrLock);
        sys = slamSys;
    }

    // 从线程安全缓存读取最新 Tcw，无需阻塞跟踪线程
    cv::Mat currentTcw;
    {
        std::lock_guard<std::mutex> tcwLock(gTcwLock);
        currentTcw = gCachedTcw.clone();
    }

    std::unique_lock<std::mutex> dataLock(gMapDataMutex, std::try_to_lock);
    if(!dataLock.owns_lock() || currentTcw.empty() || !sys){
        statusBuf[1] = ORB_SLAM2::PLANE_NOT_DETECTED;
        env->ReleaseIntArrayElements(statusBuf_, statusBuf, 0);
        return;
    }
    if(!currentTcw.empty()){
        cv::Mat TcwForPlane = currentTcw;
        if(sys->HasMapAlignment()) {
            TcwForPlane = sys->GetMapAlignedPose(currentTcw);
        }

        Plane* detected = detectPlane(TcwForPlane, vMPs, ORB_SLAM2::PLANE_DETECT_RANSAC_ITERS);
        if(detected && sys->MapChanged())
            detected->Recompute();
        statusBuf[1]=detected? ORB_SLAM2::PLANE_DETECTED : ORB_SLAM2::PLANE_NOT_DETECTED;

        if(detected) {
            dataLock.unlock();
            // 使用锁内快照 sys，避免裸读全局 slamSys
            AR_OnArPlaced(detected, sys->HasMapAlignment());
        }
    }
    env->ReleaseIntArrayElements(statusBuf_, statusBuf, 0);
}

JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_nativeGetMVP(JNIEnv *env, jobject instance,
    jfloatArray modelM_, jfloatArray viewM_, jfloatArray projM_, jint imageWidth, jint imageHeight)
{
    jfloat *modelM = env->GetFloatArrayElements(modelM_, nullptr);
    jfloat *viewM  = env->GetFloatArrayElements(viewM_, nullptr);
    jfloat *projM  = env->GetFloatArrayElements(projM_, nullptr);

    {
        std::lock_guard<std::mutex> lock(gMapDataMutex);
        for(int i=0; i<16; i++) modelM[i] = gCurrentModelMatrix[i];
        for(int i=0; i<16; i++) viewM[i]  = gCurrentViewMatrix[i];
    }
    // 视图矩阵
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
            // 对齐查询持 gSlamPtrLock（快路径），消除与 nativeShutdown delete 的竞态；
            // GetMapAlignedPose 仅短暂持 Tracking 内部锁，无反向锁序
            {
                std::lock_guard<std::mutex> ptrLock(gSlamPtrLock);
                if(slamSys && slamSys->HasMapAlignment())
                    TcwForView = slamSys->GetMapAlignedPose(TcwForView);
            }
            float tmp[16]; getColMajorMatrixFromMat(tmp, TcwForView);
            getRUBViewMatrixFromRDF(tmp, viewM);
        } else {
            std::lock_guard<std::mutex> lk(gMapDataMutex);
            for(int i=0; i<16; i++) viewM[i] = gCurrentViewMatrix[i];
        }
    }
    // 投影矩阵（输入未变时直接复用缓存，仅在分辨率/内参变化时重算）
    {
        static std::mutex sProjMutex;
        static int sW = -1, sH = -1;
        static float sFx = -1, sFy = -1, sCx = -1, sCy = -1;
        static float sProj[16] = {0};
        const int w = (int)(imageWidth/ORB_SLAM2::IMAGE_DOWNSCALE_FACTOR);
        const int h = (int)(imageHeight/ORB_SLAM2::IMAGE_DOWNSCALE_FACTOR);
        std::lock_guard<std::mutex> lk(sProjMutex);
        if(w != sW || h != sH || fx != sFx || fy != sFy || cx != sCx || cy != sCy)
        {
            frustumM_RUB(w, h, fx, fy, cx, cy,
                         ORB_SLAM2::PROJECTION_ZNEAR, ORB_SLAM2::PROJECTION_ZFAR, sProj);
            sW = w; sH = h; sFx = fx; sFy = fy; sCx = cx; sCy = cy;
        }
        memcpy(projM, sProj, sizeof(float)*16);
    }

    env->ReleaseFloatArrayElements(modelM_, modelM, 0);
    env->ReleaseFloatArrayElements(viewM_,  viewM, 0);
    env->ReleaseFloatArrayElements(projM_,  projM, 0);
}

JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_getV(JNIEnv *env, jobject instance, jfloatArray viewM_) {
    jfloat *viewM = env->GetFloatArrayElements(viewM_, nullptr);
    bool useSlam = false;
    cv::Mat TcwForView;
    {
        std::lock_guard<std::mutex> tcwLock(gTcwLock);
        if(gCachedTrackingState.load(std::memory_order_relaxed)==2 && !gCachedTcw.empty()) {
            useSlam = true; TcwForView = gCachedTcw.clone();
        }
    }
    if(useSlam) {
        // 与 nativeGetMVP 一致：持 gSlamPtrLock 的快路径对齐查询
        {
            std::lock_guard<std::mutex> ptrLock(gSlamPtrLock);
            if(slamSys && slamSys->HasMapAlignment())
                TcwForView = slamSys->GetMapAlignedPose(TcwForView);
        }
        float tmp[16]; getColMajorMatrixFromMat(tmp, TcwForView);
        getRUBViewMatrixFromRDF(tmp, viewM);
    } else {
        std::lock_guard<std::mutex> lk(gMapDataMutex);
        for(int i=0; i<16; i++) viewM[i] = gCurrentViewMatrix[i];
    }
    env->ReleaseFloatArrayElements(viewM_, viewM, 0);
}

JNIEXPORT jintArray JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_getMapStats(JNIEnv *env, jobject instance) {
    // 锁内快照（锁序 gSlamPtrLock → gMapDataMutex，不得反向嵌套）
    ORB_SLAM2::System* sys = nullptr;
    {
        std::lock_guard<std::mutex> ptrLock(gSlamPtrLock);
        sys = slamSys;
    }
    std::lock_guard<std::mutex> lock(gMapDataMutex);
    jintArray result = env->NewIntArray(3);
    if(sys) {
        jint stats[3];
        stats[0] = sys->GetNumKeyFrames();
        stats[1] = sys->GetNumMapPoints();
        stats[2] = (gAnchor.plane != nullptr) ? 1 : 0;
        env->SetIntArrayRegion(result, 0, 3, stats);
    }
    return result;
}

JNIEXPORT jfloatArray JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_getMiniMapPoints(JNIEnv *env, jobject instance, jint maxPoints) {
    // 取点与解引用全程持 gSlamPtrLock，阻塞 nativeShutdown 直至采样结束，消除 UAF；
    // 有界快路径持锁亚毫秒级，GetWorldPos 仅持 MapPoint 内部锁，无反向锁序
    std::vector<float> out;
    {
        std::lock_guard<std::mutex> ptrLock(gSlamPtrLock);
        if(!slamSys) {
            return env->NewFloatArray(0);
        }
        std::vector<ORB_SLAM2::MapPoint*> v = slamSys->GetAllMapPoints();
        size_t total = v.size();

        // 全地图均匀采样，确保全物体/多视角点云均匀保留，不丢弃旧视角点
        if (total > 0) {
            size_t limit = (maxPoints > 0 && (size_t)maxPoints < total) ? (size_t)maxPoints : total;
            size_t step = (total > limit) ? (total / limit) : 1;

            out.reserve(limit * 3);
            for(size_t i=0; i<total && out.size() < limit * 3; i += step) {
                ORB_SLAM2::MapPoint* p = v[i];
                if(!p || p->isBad()) continue;
                // 栈版读取
                cv::Point3f Pw;
                p->GetWorldPos(Pw);
                out.push_back(Pw.x);
                out.push_back(Pw.y);
                out.push_back(Pw.z);
            }
        }
    }

    jfloatArray arr = env->NewFloatArray((jsize)out.size());
    if(arr && !out.empty()) env->SetFloatArrayRegion(arr, 0, (jsize)out.size(), out.data());
    return arr;
}

JNIEXPORT jfloatArray JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_getTrackedPoints(JNIEnv *env, jobject instance, jint maxPoints) {
    std::vector<float> out;

    std::vector<ORB_SLAM2::MapPoint*> localMPs;
    {
        std::lock_guard<std::mutex> lock(gMapPointsMutex);
        localMPs = vMPs;
    }

    size_t total = localMPs.size();

    size_t limit = (maxPoints > 0 && (size_t)maxPoints < total) ? (size_t)maxPoints : total;

    out.reserve(limit * 3);
    for(size_t i=0; i<limit; ++i) {
        ORB_SLAM2::MapPoint* p = localMPs[i];
        if(!p || p->isBad()) continue;
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

JNIEXPORT jfloatArray JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_getAllArObjectsData(JNIEnv *env, jobject instance) {
    std::lock_guard<std::mutex> lock(gMapDataMutex);
    std::vector<float> data;
    if (!gAnchor.objects.empty()) {
        data.push_back((float)gAnchor.objects.size());
        for(const auto& obj : gAnchor.objects) {
            if(!obj.isValid) continue;
            for(float m : obj.modelMatrix) {
                data.push_back(m);
            }
            data.push_back(obj.scale);
        }
    } else if (gAnchor.valid && gAnchor.plane) {
        data.push_back(1.0f);
        for(int i=0; i<16; i++) {
            data.push_back(gCurrentModelMatrix[i]);
        }
        data.push_back(gArObjectScale.load(std::memory_order_relaxed));
    } else {
        data.push_back(0.0f);
    }

    jfloatArray arr = env->NewFloatArray((jsize)data.size());
    if(arr && !data.empty()) env->SetFloatArrayRegion(arr, 0, (jsize)data.size(), data.data());
    return arr;
}

JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_updateArObjectScale(JNIEnv *env, jobject instance, jfloat scaleFactor) {
    float zoomFac = (scaleFactor - 1.0f) / 5.0f;
    float current = gArObjectScale.load(std::memory_order_relaxed);
    float updated = fmax(0.03f, current + zoomFac);
    gArObjectScale.store(updated, std::memory_order_relaxed);
}

JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_setPointCloudDisplay(JNIEnv *env, jobject instance, jboolean enable) {
    gEnablePointCloudDisplay = (bool)enable;
}

JNIEXPORT jboolean JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_isPointCloudDisplayEnabled(JNIEnv *env, jobject instance) {
    return (jboolean)gEnablePointCloudDisplay;
}

// 共享内存帧持久映射
#define SH_HEADER_SIZE 256
#define SH_OFF_FRAME_W 8
#define SH_OFF_FRAME_H 12
#define SH_OFF_UI_WRITE_SEQ 16
#define SH_OFF_SLAM_DONE_SEQ 20
#define SH_OFF_TRACKING_STATE 24
#define SH_OFF_DRAW_FLAG 28
#define SH_OFF_POINTCLOUD_BYTES 32
#define SH_OFF_MVP 40
#define SH_POINTCLOUD_MAX_BYTES (96 * 1024)

static void* gSharedFramePtr = nullptr;
static int gSharedFrameSize = 0;
static int gSharedW = 0;
static int gSharedH = 0;
static std::mutex gSharedFrameLock;

static inline int32_t* shIntPtr(int off) {
    return reinterpret_cast<int32_t*>(static_cast<char*>(gSharedFramePtr) + off);
}
static inline float* shFloatPtr(int off) {
    return reinterpret_cast<float*>(static_cast<char*>(gSharedFramePtr) + off);
}
static inline int shYOffset(int bufIndex) {
    return SH_HEADER_SIZE + (bufIndex & 1) * (gSharedW * gSharedH);
}
static inline int shPointCloudOffset() {
    return SH_HEADER_SIZE + 2 * (gSharedW * gSharedH);
}

JNIEXPORT jboolean JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_nativeAttachFrameBuffer(
    JNIEnv* env, jobject instance, jint fd, jint size)
{
    std::lock_guard<std::mutex> lock(gSharedFrameLock);
    if (gSharedFramePtr) {
        munmap(gSharedFramePtr, gSharedFrameSize);
        gSharedFramePtr = nullptr;
        gSharedFrameSize = 0;
        gSharedW = gSharedH = 0;
    }
    if (fd < 0 || size <= 0) return JNI_FALSE;

    void* mappedPtr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (mappedPtr == MAP_FAILED) {
        LOGE("nativeAttachFrameBuffer: mmap 映射内存失败");
        return JNI_FALSE;
    }
    gSharedFramePtr = mappedPtr;
    gSharedFrameSize = size;
    gSharedW = *shIntPtr(SH_OFF_FRAME_W);
    gSharedH = *shIntPtr(SH_OFF_FRAME_H);
    if (gSharedW <= 0 || gSharedH <= 0 || gSharedW > 4096 || gSharedH > 4096) {
        gSharedW = 0;
        gSharedH = 0;
    }
    return JNI_TRUE;
}

JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_nativeDetachFrameBuffer(JNIEnv* env, jobject instance)
{
    std::lock_guard<std::mutex> lock(gSharedFrameLock);
    if (gSharedFramePtr) {
        munmap(gSharedFramePtr, gSharedFrameSize);
        gSharedFramePtr = nullptr;
        gSharedFrameSize = 0;
        gSharedW = gSharedH = 0;
    }
}

static void writeResultToSharedMemory(int seq, int tracking, bool draw, int w, int h) {
    if (!gSharedFramePtr) return;
    *shIntPtr(SH_OFF_TRACKING_STATE) = tracking;
    *shIntPtr(SH_OFF_DRAW_FLAG) = draw ? 1 : 0;

    float* mvp = shFloatPtr(SH_OFF_MVP);
    {
        std::lock_guard<std::mutex> lock(gMapDataMutex);
        memcpy(mvp,      gCurrentModelMatrix, 16 * sizeof(float));
        memcpy(mvp + 16, gCurrentViewMatrix,  16 * sizeof(float));
    }
    const int projW = cvRound((float)w / ORB_SLAM2::IMAGE_DOWNSCALE_FACTOR);
    const int projH = cvRound((float)h / ORB_SLAM2::IMAGE_DOWNSCALE_FACTOR);
    frustumM_RUB(projW, projH, fx, fy, cx, cy,
                 ORB_SLAM2::PROJECTION_ZNEAR, ORB_SLAM2::PROJECTION_ZFAR, mvp + 32);

    *shIntPtr(SH_OFF_SLAM_DONE_SEQ) = seq;
}

static void writePointCloudToSharedMemory() {
    if (!gSharedFramePtr || !slamSys) {
        if (gSharedFramePtr) *shIntPtr(SH_OFF_POINTCLOUD_BYTES) = 0;
        return;
    }
    if (!gEnablePointCloudDisplay) {
        *shIntPtr(SH_OFF_POINTCLOUD_BYTES) = 0;
        return;
    }

    static int sPointCloudFrameCounter = 0;
    if ((++sPointCloudFrameCounter & 1) != 0) {
        return;
    }

    const int maxFloats = SH_POINTCLOUD_MAX_BYTES / 4;
    float* dst = shFloatPtr(shPointCloudOffset());
    int n = 0;

    std::vector<ORB_SLAM2::MapPoint*> localMPs;
    {
        std::lock_guard<std::mutex> lock(gMapPointsMutex);
        localMPs = vMPs;
    }

    const size_t NMPs = localMPs.size();
    for (size_t i = 0; i < NMPs; ++i) {
        ORB_SLAM2::MapPoint* pMP = localMPs[i];
        if (!pMP || pMP->isBad()) continue;
        cv::Point3f Pw;
        pMP->GetWorldPos(Pw);
        if (n * 7 + 7 > maxFloats) break;

        dst[n*7+0] = Pw.x;
        dst[n*7+1] = Pw.y;
        dst[n*7+2] = Pw.z;
        if (pMP->mbFromLoadedMap) {
            dst[n*7+3] = 0.0f; dst[n*7+4] = 1.0f; dst[n*7+5] = 0.0f;       // 绿色
        } else {
            dst[n*7+3] = 31.0f/255.0f; dst[n*7+4] = 188.0f/255.0f; dst[n*7+5] = 210.0f/255.0f; // 青色
        }
        dst[n*7+6] = 8.0f;
        n++;
    }

    const int status = slamSys->GetTrackingState();
    if (status == 2 && slamSys->HasMapAlignment()) {
        std::vector<ORB_SLAM2::MapPoint*> allMPs = slamSys->GetAllMapPoints();
        int count = 0;
        const int maxDrawPoints = 3000;
        for (auto pMP : allMPs) {
            if (!pMP || pMP->isBad() || !pMP->mbFromLoadedMap) continue;

            cv::Point3f Pw;
            pMP->GetWorldPos(Pw);
            if (n * 7 + 7 > maxFloats) break;

            dst[n*7+0] = Pw.x;
            dst[n*7+1] = Pw.y;
            dst[n*7+2] = Pw.z;
            dst[n*7+3] = 0.0f; dst[n*7+4] = 1.0f; dst[n*7+5] = 0.0f;
            dst[n*7+6] = 4.0f;
            n++;

            count++;
            if (count >= maxDrawPoints) break;
        }
    }

    *shIntPtr(SH_OFF_POINTCLOUD_BYTES) = n * 7 * (int)sizeof(float);
}

JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_nativeProcessFrameSharedMem(
    JNIEnv* env, jobject instance, jint bufIndex, jint seq, jint width, jint height, jintArray statusBuf_)
{
    if (width <= 0 || height <= 0) return;
    jint* statusBuf = env->GetIntArrayElements(statusBuf_, nullptr);
    if (!statusBuf) return;

    std::lock_guard<std::mutex> lock(gSharedFrameLock);
    if (!gSharedFramePtr) {
        statusBuf[0] = 0;
        statusBuf[1] = 0;
        env->ReleaseIntArrayElements(statusBuf_, statusBuf, 0);
        return;
    }

    gSharedW = width;
    gSharedH = height;

    const int yOff = shYOffset(bufIndex);
    const int frameBytes = width * height;
    if (gSharedFrameSize < yOff + frameBytes) {
        LOGE("nativeProcessFrameSharedMem: 共享内存尺寸不足");
        statusBuf[0] = 0;
        statusBuf[1] = 0;
        env->ReleaseIntArrayElements(statusBuf_, statusBuf, 0);
        return;
    }

    cv::Mat mGr(height, width, CV_8UC1, static_cast<char*>(gSharedFramePtr) + yOff);
    cv::Mat dummyOut;

    // writePointCloudToSharedMemory 在 processImage 返回后仍解引用 slamSys 与 MapPoint，
    // 此处持有第二条帧引用覆盖写回段，与 nativeShutdown 的归零再 delete 协议一致，消除 UAF
    bool shmHoldsFrameRef = false;
    {
        std::lock_guard<std::mutex> ptrLock(gSlamPtrLock);
        if (slamSys) {
            gProcessingFrames.fetch_add(1, std::memory_order_relaxed);
            shmHoldsFrameRef = true;
        }
    }
    struct ShmFrameRefGuard {
        const bool armed;
        explicit ShmFrameRefGuard(bool a) : armed(a) {}
        ~ShmFrameRefGuard() {
            if (armed) {
                gProcessingFrames.fetch_sub(1, std::memory_order_release);
                gCvProcessingFrames.notify_one();
            }
        }
    } shmFrameRefGuard(shmHoldsFrameRef);

    int tmpStatus[3] = {0};
    const int trackingResult = processImage(mGr, dummyOut, tmpStatus);
    const bool draw = gShouldDrawArObject;
    statusBuf[0] = trackingResult;
    statusBuf[1] = draw ? 1 : 0;

    writePointCloudToSharedMemory();
    writeResultToSharedMemory(seq, trackingResult, draw, width, height);

    env->ReleaseIntArrayElements(statusBuf_, statusBuf, 0);
}

// 关停并释放 SLAM 系统：等待帧处理归零后 join 全部工作线程再 delete。
// 由 Activity.onDestroy 调用，确保三条常驻线程（LM/LC/GlobalReloc）全部退出。
JNIEXPORT void JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_nativeShutdown(JNIEnv* env, jobject instance)
{
    LOGD("nativeShutdown: 开始关停 SLAM 系统");
    {
        // 写锁协议：与 loadMapWithId 一致，等待正在处理的帧完全结束
        std::unique_lock<std::mutex> lock(gSlamPtrLock);
        gCvProcessingFrames.wait(lock, []{
            return gProcessingFrames.load(std::memory_order_acquire) == 0;
        });

        if(slamSys){
            // Shutdown 内部：StopGlobalRelocThread → RequestStopGBA(join) → join LM/LC
            slamSys->Shutdown();
            delete slamSys;   // ~System 释放各子模块与全部子地图
            slamSys = nullptr;
        }
        slamInitialized = false;
        timeStamp = 0.0;
        lastOkTime = 0.0;
        wasLost = false;
        gLostFrameCount = 0;
    }

    // 清空 JNI 侧缓存与 AR 上下文
    {
        std::lock_guard<std::mutex> lock(gMapPointsMutex);
        vMPs.clear();
        vKeys.clear();
    }
    {
        std::lock_guard<std::mutex> lock(gTcwLock);
        gCachedTcw = cv::Mat();
    }
    gCachedTrackingState.store(0, std::memory_order_relaxed);
    gShouldDrawArObject.store(false);
    {
        std::lock_guard<std::mutex> lock(gMapDataMutex);
        gAnchor.Reset();
        gMapAnchors.clear();
        gActiveMapId = 0;
        gMapSwitchCounter = 0;
    }
    LOGD("nativeShutdown: SLAM 系统已完全释放");
}

}
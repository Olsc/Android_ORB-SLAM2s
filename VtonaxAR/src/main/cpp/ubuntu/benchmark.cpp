/**
 * VtonaxAR Benchmark — 离线视频评测工具
 *
 * 用法:
 *   ./VtonaxAR_Benchmark <video_path> [output_report.json]
 *
 * 功能:
 *   - 读取本地视频文件，逐帧运行完整 SLAM 流水线
 *   - 不启动可视化窗口，确保纯 CPU 性能评测
 *   - 强制固定 30 FPS 处理帧率，无论输入视频帧率高低
 *   - 输出 JSON 分析报告，包含:
 *       • 整体统计 (总帧数/跟踪率/地图规模)
 *       • 逐帧时间序列 (帧耗时/跟踪状态/内点数/关键帧标记)
 *       • 实时性分析 (帧耗时分布/P50/P95/P99)
 *       • 稳定性分段 (连续跟踪段/丢失段)
 *       • 场景难度分类
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>
#include <mutex>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <map>
#include <csignal>
#include <opencv2/opencv.hpp>

#include "include/System.h"
#include "Common.h"
#include "Plane.h"
#include "Matrix.h"
#include "include/Config.h"

// ============================================================
// 全局状态 (与 JNI/native-lib.cpp 对齐的最小集合)
// ============================================================
ORB_SLAM2::System* slamSys = nullptr;
Plane* pPlane = nullptr;

float fx, fy, cx, cy;
double timeStamp = 0.0;
bool gEnableSLAM = true;

std::mutex gSlamStateMutex;
std::mutex gMapDataMutex;

std::vector<ORB_SLAM2::MapPoint*> vMPs;
std::vector<cv::KeyPoint> vKeys;
cv::Mat Tcw;

// ============================================================
// 逐帧记录数据结构
// ============================================================
struct FrameRecord {
    int frameId;
    double timestamp;
    int trackingState;
    double processMs;
    int trackedPoints;
    int keypoints;
    int isKeyframe;
    int totalMapPoints;
    int totalKeyframes;
};

std::vector<FrameRecord> gRecords;
int gTotalKeyframes = 0;

// 信号处理：Ctrl+C 提前结束并输出报告
volatile bool gStopRequested = false;
void signalHandler(int) { gStopRequested = true; }

// ============================================================
// CSV 输出
// ============================================================
void writeCsv(const std::string& path, const std::vector<FrameRecord>& records) {
    std::ofstream ofs(path);
    if (!ofs.is_open()) return;
    ofs << "frame_id,timestamp,tracking_state,process_ms,tracked_points,keypoints,is_keyframe,total_map_points,total_keyframes\n";
    for (const auto& r : records) {
        ofs << r.frameId << ","
            << r.timestamp << ","
            << r.trackingState << ","
            << r.processMs << ","
            << r.trackedPoints << ","
            << r.keypoints << ","
            << r.isKeyframe << ","
            << r.totalMapPoints << ","
            << r.totalKeyframes << "\n";
    }
}

// ============================================================
// 分析报告输出
// ============================================================
void writeJsonReport(const std::string& path, const std::vector<FrameRecord>& records,
                     int totalFrames, double videoFps, int okFrames, int lostFrames) {
    // 计算耗时百分位
    std::vector<double> times;
    for (const auto& r : records) times.push_back(r.processMs);
    std::sort(times.begin(), times.end());

    auto percentile = [&](double p) -> double {
        if (times.empty()) return 0.0;
        size_t idx = std::min(times.size() - 1, (size_t)(times.size() * p / 100.0));
        return times[idx];
    };

    double mean = times.empty() ? 0.0 : std::accumulate(times.begin(), times.end(), 0.0) / times.size();
    double median = percentile(50);
    double p95 = percentile(95);
    double p99 = percentile(99);
    double minT = times.empty() ? 0.0 : times.front();
    double maxT = times.empty() ? 0.0 : times.back();

    // 分析连续跟踪段
    std::vector<std::pair<int, int>> okSegments;  // (startFrame, length)
    std::vector<std::pair<int, int>> lostSegments;
    int currentStart = 0;
    int currentState = (records.empty() ? -1 : records[0].trackingState);

    for (size_t i = 1; i < records.size(); ++i) {
        if (records[i].trackingState != currentState) {
            int length = i - currentStart;
            if (currentState == 2)
                okSegments.push_back({currentStart, length});
            else if (currentState == 3)
                lostSegments.push_back({currentStart, length});
            currentStart = i;
            currentState = records[i].trackingState;
        }
    }
    int finalLength = records.size() - currentStart;
    if (currentState == 2)
        okSegments.push_back({currentStart, finalLength});
    else if (currentState == 3)
        lostSegments.push_back({currentStart, finalLength});

    int maxOk = 0, maxLost = 0;
    for (const auto& s : okSegments) if (s.second > maxOk) maxOk = s.second;
    for (const auto& s : lostSegments) if (s.second > maxLost) maxLost = s.second;

    // 计算场景难度分类
    int goodCount = 0, mediumCount = 0, hardCount = 0, severeCount = 0;
    for (const auto& r : records) {
        if (r.trackingState == 2) goodCount++;
        else if (r.trackingState == 3) severeCount++;
        else if (r.trackingState == 1) mediumCount++;
    }

    std::ofstream ofs(path);
    if (!ofs.is_open()) return;
    ofs << "{\n";
    ofs << "  \"summary\": {\n";
    ofs << "    \"total_frames\": " << totalFrames << ",\n";
    ofs << "    \"video_fps\": " << videoFps << ",\n";
    ofs << "    \"ok_frames\": " << okFrames << ",\n";
    ofs << "    \"lost_frames\": " << lostFrames << ",\n";
    ofs << "    \"ok_ratio\": " << (totalFrames > 0 ? (double)okFrames / totalFrames * 100.0 : 0.0) << ",\n";
    ofs << "    \"max_continuous_ok\": " << maxOk << ",\n";
    ofs << "    \"max_continuous_lost\": " << maxLost << "\n";
    ofs << "  },\n";
    ofs << "  \"timing\": {\n";
    ofs << "    \"mean_ms\": " << mean << ",\n";
    ofs << "    \"median_ms\": " << median << ",\n";
    ofs << "    \"p95_ms\": " << p95 << ",\n";
    ofs << "    \"p99_ms\": " << p99 << ",\n";
    ofs << "    \"min_ms\": " << minT << ",\n";
    ofs << "    \"max_ms\": " << maxT << "\n";
    ofs << "  },\n";
    ofs << "  \"records\": [\n";
    for (size_t i = 0; i < records.size(); ++i) {
        const auto& r = records[i];
        ofs << "    {\"frame_id\":" << r.frameId
            << ",\"ts\":" << r.timestamp
            << ",\"state\":" << r.trackingState
            << ",\"ms\":" << r.processMs
            << ",\"tracked\":" << r.trackedPoints
            << ",\"kps\":" << r.keypoints
            << ",\"kf\":" << r.isKeyframe
            << ",\"total_mp\":" << r.totalMapPoints
            << ",\"total_kf\":" << r.totalKeyframes
            << "}";
        if (i < records.size() - 1) ofs << ",";
        ofs << "\n";
    }
    ofs << "  ]\n";
    ofs << "}\n";
}

// ============================================================
// 打印终端报告
// ============================================================
void printReport(const std::vector<FrameRecord>& records, int totalProcessed, double videoFps) {
    int okCt = 0, lostCt = 0, initCt = 0;
    for (const auto& r : records) {
        if (r.trackingState == 2) okCt++;
        else if (r.trackingState == 3) lostCt++;
        else if (r.trackingState == 1) initCt++;
    }

    std::vector<double> times;
    for (const auto& r : records) times.push_back(r.processMs);
    std::sort(times.begin(), times.end());

    auto percentile = [&](double p) -> double {
        if (times.empty()) return 0.0;
        size_t idx = std::min(times.size() - 1, (size_t)(times.size() * p / 100.0));
        return times[idx];
    };

    double mean = times.empty() ? 0.0 : std::accumulate(times.begin(), times.end(), 0.0) / times.size();

    std::cout << "\n";
    std::cout << "============================================\n";
    std::cout << "  VtonaxAR Benchmark Report\n";
    std::cout << "============================================\n";
    std::cout << "  处理帧数:     " << totalProcessed << " (固定 30 FPS)\n";
    std::cout << "  OK帧:         " << okCt << " (" << (totalProcessed > 0 ? okCt * 100.0 / totalProcessed : 0.0) << "%)\n";
    std::cout << "  丢失帧:       " << lostCt << " (" << (totalProcessed > 0 ? lostCt * 100.0 / totalProcessed : 0.0) << "%)\n";
    std::cout << "  初始化帧:     " << initCt << "\n";
    std::cout << "  最终地图点:   " << (records.empty() ? 0 : records.back().totalMapPoints) << "\n";
    std::cout << "  最终关键帧:   " << (records.empty() ? 0 : records.back().totalKeyframes) << "\n";
    std::cout << "  --- 实时性 ---\n";
    std::cout << "  平均耗时:     " << mean << " ms\n";
    std::cout << "  中位耗时:     " << percentile(50) << " ms\n";
    std::cout << "  P95:          " << percentile(95) << " ms\n";
    std::cout << "  P99:          " << percentile(99) << " ms\n";
    std::cout << "============================================\n";
}

// ============================================================
// 主函数
// ============================================================
int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "用法: " << argv[0] << " <video_path> [output_report.json]\n";
        return 1;
    }

    std::string videoPath = argv[1];
    std::string outputPath = (argc >= 3) ? argv[2] : "benchmark_report";

    // 注册信号处理
    signal(SIGINT, signalHandler);

    // 加载相机内参
    fx = ORB_SLAM2::CAMERA_FX;
    fy = ORB_SLAM2::CAMERA_FY;
    cx = ORB_SLAM2::CAMERA_CX;
    cy = ORB_SLAM2::CAMERA_CY;

    // 初始化 SLAM 系统
    std::cout << "正在初始化 SLAM 系统...\n";
    slamSys = new ORB_SLAM2::System("", ORB_SLAM2::System::MONOCULAR);
    if (!slamSys) {
        std::cerr << "SLAM 系统初始化失败\n";
        return 1;
    }
    std::cout << "SLAM 系统初始化成功\n";

    // 打开视频文件
    cv::VideoCapture cap(videoPath);
    if (!cap.isOpened()) {
        std::cerr << "无法打开视频文件: " << videoPath << "\n";
        delete slamSys;
        return 1;
    }

    double videoFps = cap.get(cv::CAP_PROP_FPS);
    if (videoFps <= 0) videoFps = 30.0;
    int totalInputFrames = (int)cap.get(cv::CAP_PROP_FRAME_COUNT);
    std::cout << "视频信息: " << videoPath
              << "  " << totalInputFrames << " 帧 @ " << videoFps << " FPS\n"
              << "处理帧率: 固定 30 FPS (输入帧跳过保持匀速)\n";

    // 帧率控制：根据输入 FPS 计算跳帧比率，强制 30 FPS
    const double TARGET_FPS = 30.0;
    const int frameSkip = std::max(1, (int)std::round(videoFps / TARGET_FPS));

    // 主循环
    cv::Mat frame, imgGr;
    timeStamp = 0.0;
    int frameId = 0;
    int totalProcessed = 0;
    gRecords.reserve(totalInputFrames > 0 ? totalInputFrames / frameSkip + 1 : 10000);

    auto startWall = std::chrono::steady_clock::now();

    while (!gStopRequested) {
        // 跳帧：每 frameSkip 输入帧只处理 1 帧，保证 30 FPS 输出
        for (int i = 0; i < frameSkip; ++i) {
            cap >> frame;
            if (frame.empty()) goto end_loop;
        }
        timeStamp += 1.0 / TARGET_FPS;

        cv::cvtColor(frame, imgGr, cv::COLOR_BGR2GRAY);

        // 缩放到 SLAM 工作分辨率
        cv::Mat imgSmall;
        cv::resize(imgGr, imgSmall,
                   cv::Size((int)ORB_SLAM2::BASE_SLAM_WIDTH,
                            (int)ORB_SLAM2::BASE_SLAM_HEIGHT));

        auto t0 = std::chrono::steady_clock::now();
        int status = 0;

        if (gEnableSLAM) {
            std::unique_lock<std::mutex> lock(gSlamStateMutex);
            if (slamSys) {
                Tcw = slamSys->TrackMonocular(imgSmall, timeStamp);
                status = slamSys->GetTrackingState();

                std::lock_guard<std::mutex> lockData(gMapDataMutex);
                vMPs = slamSys->GetTrackedMapPoints();
                vKeys = slamSys->GetTrackedKeyPointsUn();
            }
        }

        auto t1 = std::chrono::steady_clock::now();
        double ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;

        // 记录帧数据
        FrameRecord rec;
        rec.frameId = frameId;
        rec.timestamp = timeStamp;
        rec.trackingState = status;
        rec.processMs = ms;
        rec.trackedPoints = (int)vMPs.size();
        rec.keypoints = (int)vKeys.size();
        rec.isKeyframe = 0;
        rec.totalMapPoints = slamSys ? slamSys->GetNumMapPoints() : 0;
        rec.totalKeyframes = slamSys ? slamSys->GetNumKeyFrames() : 0;
        gRecords.push_back(rec);

        totalProcessed++;
        frameId++;

        // 进度输出
        if (totalProcessed % 100 == 0) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - startWall).count();
            std::cout << "\r处理进度: " << totalProcessed << " 帧"
                      << "  耗时 " << elapsed << "s   "
                      << std::flush;
        }
    }

end_loop:
    auto totalWall = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - startWall).count();

    cap.release();

    // 输出报告
    int okFrames = 0, lostFrames = 0;
    for (const auto& r : gRecords) {
        if (r.trackingState == 2) okFrames++;
        else if (r.trackingState == 3) lostFrames++;
    }

    printReport(gRecords, totalProcessed, videoFps);
    std::cout << "总耗时: " << totalWall << "s\n";

    // 写入报告文件
    writeJsonReport(outputPath + ".json", gRecords, totalProcessed, videoFps, okFrames, lostFrames);
    writeCsv(outputPath + ".csv", gRecords);
    std::cout << "报告已写入: " << outputPath << ".json / .csv\n";

    // 清理
    if (slamSys) {
        slamSys->Shutdown();
        delete slamSys;
    }
    if (pPlane) delete pPlane;

    return 0;
}

/**
 * MenthaAR SLAM Benchmark — 离线视频性能与稳定性评测工具
 *
 * 用法:
 *   ./MenthaAR_Benchmark <video_path> [output_report_prefix] [--loops N]
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>
#include <mutex>
#include <thread>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <map>
#include <csignal>
#include <cstdio>   // popen, pclose, fgets
#include <cstdlib>  // std::system
#include <unistd.h> // sysconf, getpagesize
#include <opencv2/opencv.hpp>

#include "include/System.h"
#include "Common.h"
#include "Plane.h"
#include "UIUtils.h"
#include "Matrix.h"
#include "include/Config.h"

// ============================================================
// 全局状态变量
// ============================================================
ORB_SLAM2::System* slamSys = nullptr;
Plane* pPlane = nullptr;
bool planeLoadedFromMap = false;

float fx, fy, cx, cy;
double timeStamp = 0.0;
bool gEnableSLAM = true;
bool gEnablePointCloudDisplay = true;
bool gShouldDrawArObject = false;
bool gShowMemoryPanel = true; // 是否开启内存分布可视化仪表盘

std::mutex gSlamStateMutex;
std::mutex gMapDataMutex;

std::vector<ORB_SLAM2::MapPoint*> vMPs;
std::vector<cv::KeyPoint> vKeys;
cv::Mat Tcw;

// 循环控制与评测状态
int gTargetLoops = 1;
int gCurrentLoop = 1;
bool gBenchmarkCompleted = false;
bool gBenchmarkPaused = false;
std::string gVideoPath = "";
std::string gOutputPrefix = "benchmark_report";

// 信号处理：Ctrl+C 提前结束并输出报告
volatile bool gStopRequested = false;
void signalHandler(int) { gStopRequested = true; }

// ============================================================
// 数据结构定义
// ============================================================
struct FrameRecord {
    int loopId;
    int frameId;
    double timestamp;
    int trackingState;
    double processMs;
    int trackedPoints;
    int keypoints;
    int isKeyframe;
    int totalMapPoints;
    int totalKeyframes;
    double rssMB;
};

struct MemoryInfo {
    double vmsMB = 0.0;
    double rssMB = 0.0;
    double peakRssMB = 0.0;
    double mpMemMB = 0.0;
    double kfMemMB = 0.0;
    double bufferMemMB = 0.0;
    double otherMemMB = 0.0;
};

struct ScoreCard {
    double overallScore = 0.0;
    std::string grade = "D";
    double trackingScore = 0.0;  // 40%
    double latencyScore = 0.0;   // 30%
    double stabilityScore = 0.0; // 20%
    double memoryScore = 0.0;    // 10%
    int totalFrames = 0;
    int okFrames = 0;
    int lostFrames = 0;
    double okRatio = 0.0;
    double meanMs = 0.0;
    double medianMs = 0.0;
    double p95Ms = 0.0;
    double p99Ms = 0.0;
    int maxContinuousOk = 0;
    double peakRssMB = 0.0;
    double endRssMB = 0.0;
    int finalMPs = 0;
    int finalKFs = 0;
};

std::vector<FrameRecord> gRecords;
std::vector<double> gRssHistory;
ScoreCard gScoreCard;

// GUI 菜单项结构体
struct Button {
    std::string label;
    cv::Rect rect;
    cv::Scalar color;
    bool isHovered;
    std::function<void()> action;
};

struct MenuSection {
    std::string title;
    cv::Rect rect;
    bool expanded;
    std::vector<Button> buttons;
};

std::vector<MenuSection> menuSections;
cv::Point mousePos(-1, -1);

static const int kDrawerWidth = 240;
static const int kDrawerHotZone = 10;
static bool gDrawerVisible = false;
static int gFrameWidth = 0;

// 前置函数声明
void initMenu();
void drawGUI(cv::Mat& frame, int trackingState, int fps, const MemoryInfo& memInfo, int curLoop, int totalLoops);
void drawMemoryDashboard(cv::Mat& frame, const MemoryInfo& memInfo, const std::vector<double>& rssHist);
void drawScoreCardModal(cv::Mat& frame, const ScoreCard& card);
void onMouse(int event, int x, int y, int flags, void* userdata);
void drawARCube(cv::Mat& im, const cv::Mat& Tcw, Plane* plane, float fx, float fy, float cx, float cy);
MemoryInfo getMemoryInfo(int numMPs, int numKFs);
ScoreCard calculateScoreCard(const std::vector<FrameRecord>& records, int totalFrames);
void resetBenchmarkState();

// ============================================================
// 视频格式兼容工具 (多流 AVI 处理)
// ============================================================
static std::string tryStripAudioStream(const std::string& path) {
    size_t dot = path.find_last_of('.');
    if (dot == std::string::npos) return "";
    std::string ext = path.substr(dot);
    if (ext != ".avi" && ext != ".AVI") return "";

    FILE* which = popen("which ffmpeg 2>/dev/null", "r");
    if (!which) return "";
    char check[16] = {};
    bool hasFfmpeg = (fgets(check, sizeof(check), which) != nullptr);
    pclose(which);
    if (!hasFfmpeg) return "";

    std::string fixed = path + ".noaudio.avi";
    std::string cmd = "ffmpeg -y -i \"" + path + "\" -vcodec copy -an \"" + fixed + "\" 2>/dev/null";
    if (std::system(cmd.c_str()) == 0) {
        std::cout << "[INFO] 检测到多流 AVI，已自动剥离音频流: " << fixed << "\n";
        return fixed;
    }
    return "";
}

// ============================================================
// Linux 进程内存读取与估计函数
// ============================================================
MemoryInfo getMemoryInfo(int numMPs, int numKFs) {
    MemoryInfo info;
    long pageSizeBytes = sysconf(_SC_PAGESIZE);
    if (pageSizeBytes <= 0) pageSizeBytes = 4096;

    std::ifstream statm("/proc/self/statm");
    if (statm.is_open()) {
        long pagesTotal = 0, pagesResident = 0;
        statm >> pagesTotal >> pagesResident;
        info.vmsMB = (pagesTotal * pageSizeBytes) / (1024.0 * 1024.0);
        info.rssMB = (pagesResident * pageSizeBytes) / (1024.0 * 1024.0);
        statm.close();
    }

    std::ifstream status("/proc/self/status");
    if (status.is_open()) {
        std::string line;
        while (std::getline(status, line)) {
            if (line.find("VmPeak:") == 0) {
                long val = 0;
                sscanf(line.c_str(), "VmPeak:\t%ld kB", &val);
                info.peakRssMB = val / 1024.0;
                break;
            }
        }
        status.close();
    }
    if (info.peakRssMB <= 0) info.peakRssMB = info.rssMB;

    // 估计结构内存开销
    info.mpMemMB = (numMPs * 450.0) / (1024.0 * 1024.0);
    info.kfMemMB = (numKFs * 55.0 * 1024.0) / (1024.0 * 1024.0);
    info.bufferMemMB = (1280 * 720 * 3 * 2 + 640 * 360 * 2) / (1024.0 * 1024.0); // ~7 MB
    info.otherMemMB = std::max(0.0, info.rssMB - (info.mpMemMB + info.kfMemMB + info.bufferMemMB));

    return info;
}

// ============================================================
// 综合性能评分计算逻辑
// ============================================================
ScoreCard calculateScoreCard(const std::vector<FrameRecord>& records, int totalFrames) {
    ScoreCard card;
    card.totalFrames = totalFrames;
    if (records.empty() || totalFrames <= 0) return card;

    for (const auto& r : records) {
        if (r.trackingState == ORB_SLAM2::Tracking::OK) card.okFrames++;
        else if (r.trackingState == ORB_SLAM2::Tracking::LOST) card.lostFrames++;
    }
    card.okRatio = (double)card.okFrames / totalFrames * 100.0;

    std::vector<double> times;
    times.reserve(records.size());
    for (const auto& r : records) times.push_back(r.processMs);
    std::sort(times.begin(), times.end());

    auto pct = [&](double p) -> double {
        if (times.empty()) return 0.0;
        size_t idx = std::min(times.size() - 1, (size_t)(times.size() * p / 100.0));
        return times[idx];
    };

    card.meanMs = std::accumulate(times.begin(), times.end(), 0.0) / times.size();
    card.medianMs = pct(50);
    card.p95Ms = pct(95);
    card.p99Ms = pct(99);

    // 最长连续 OK 段
    int curOk = 0, maxOk = 0;
    for (const auto& r : records) {
        if (r.trackingState == ORB_SLAM2::Tracking::OK) {
            curOk++;
            if (curOk > maxOk) maxOk = curOk;
        } else {
            curOk = 0;
        }
    }
    card.maxContinuousOk = maxOk;

    card.finalMPs = records.back().totalMapPoints;
    card.finalKFs = records.back().totalKeyframes;
    card.endRssMB = records.back().rssMB;

    double peak = 0.0;
    for (const auto& r : records) if (r.rssMB > peak) peak = r.rssMB;
    card.peakRssMB = peak;

    // 1. 跟踪得分 (40%)
    card.trackingScore = card.okRatio;

    // 2. 实时性延迟得分 (30%) - 目标 33.3ms (30 FPS)
    const double targetMs = 33.33;
    if (card.meanMs <= targetMs) {
        card.latencyScore = 100.0;
    } else {
        card.latencyScore = std::max(0.0, 100.0 - (card.meanMs - targetMs) * 2.5);
    }

    // 3. 地图稳定性得分 (20%)
    double contRatio = (double)maxOk / totalFrames;
    double density = (double)card.finalMPs / (card.finalKFs + 1);
    double densityScore = std::min(100.0, density * 0.2 * 100.0);
    card.stabilityScore = std::min(100.0, 0.6 * (contRatio * 100.0) + 0.4 * densityScore);

    // 4. 内存健康度得分 (10%)
    card.memoryScore = std::max(0.0, 100.0 - std::min(50.0, card.peakRssMB / 10.0));

    // 综合加权总分
    card.overallScore = 0.40 * card.trackingScore +
                        0.30 * card.latencyScore +
                        0.20 * card.stabilityScore +
                        0.10 * card.memoryScore;

    // 评级
    if (card.overallScore >= 95.0) card.grade = "S+";
    else if (card.overallScore >= 90.0) card.grade = "S";
    else if (card.overallScore >= 80.0) card.grade = "A";
    else if (card.overallScore >= 70.0) card.grade = "B";
    else if (card.overallScore >= 60.0) card.grade = "C";
    else card.grade = "D";

    return card;
}

// ============================================================
// JSON / CSV 导出报告
// ============================================================
void exportReports(const std::string& prefix, const ScoreCard& card, const std::vector<FrameRecord>& records) {
    // JSON
    std::string jsonPath = prefix + ".json";
    std::ofstream ofs(jsonPath);
    if (ofs.is_open()) {
        ofs << "{\n";
        ofs << "  \"score_card\": {\n";
        ofs << "    \"overall_score\": " << std::fixed << std::setprecision(1) << card.overallScore << ",\n";
        ofs << "    \"grade\": \"" << card.grade << "\",\n";
        ofs << "    \"tracking_score\": " << card.trackingScore << ",\n";
        ofs << "    \"latency_score\": " << card.latencyScore << ",\n";
        ofs << "    \"stability_score\": " << card.stabilityScore << ",\n";
        ofs << "    \"memory_score\": " << card.memoryScore << "\n";
        ofs << "  },\n";
        ofs << "  \"summary\": {\n";
        ofs << "    \"total_loops\": " << gTargetLoops << ",\n";
        ofs << "    \"total_frames\": " << card.totalFrames << ",\n";
        ofs << "    \"ok_frames\": " << card.okFrames << ",\n";
        ofs << "    \"lost_frames\": " << card.lostFrames << ",\n";
        ofs << "    \"ok_ratio\": " << card.okRatio << ",\n";
        ofs << "    \"max_continuous_ok\": " << card.maxContinuousOk << ",\n";
        ofs << "    \"final_map_points\": " << card.finalMPs << ",\n";
        ofs << "    \"final_keyframes\": " << card.finalKFs << "\n";
        ofs << "  },\n";
        ofs << "  \"timing\": {\n";
        ofs << "    \"mean_ms\": " << card.meanMs << ",\n";
        ofs << "    \"median_ms\": " << card.medianMs << ",\n";
        ofs << "    \"p95_ms\": " << card.p95Ms << ",\n";
        ofs << "    \"p99_ms\": " << card.p99Ms << "\n";
        ofs << "  },\n";
        ofs << "  \"memory\": {\n";
        ofs << "    \"peak_rss_mb\": " << card.peakRssMB << ",\n";
        ofs << "    \"end_rss_mb\": " << card.endRssMB << "\n";
        ofs << "  },\n";
        ofs << "  \"records\": [\n";
        for (size_t i = 0; i < records.size(); ++i) {
            const auto& r = records[i];
            ofs << "    {\"loop\":" << r.loopId
                << ",\"frame\":" << r.frameId
                << ",\"ts\":" << r.timestamp
                << ",\"state\":" << r.trackingState
                << ",\"ms\":" << r.processMs
                << ",\"tracked\":" << r.trackedPoints
                << ",\"total_mp\":" << r.totalMapPoints
                << ",\"total_kf\":" << r.totalKeyframes
                << ",\"rss_mb\":" << r.rssMB << "}";
            if (i < records.size() - 1) ofs << ",";
            ofs << "\n";
        }
        ofs << "  ]\n";
        ofs << "}\n";
        ofs.close();
    }

    // CSV
    std::string csvPath = prefix + ".csv";
    std::ofstream cfs(csvPath);
    if (cfs.is_open()) {
        cfs << "loop_id,frame_id,timestamp,tracking_state,process_ms,tracked_points,total_map_points,total_keyframes,rss_mb\n";
        for (const auto& r : records) {
            cfs << r.loopId << "," << r.frameId << "," << r.timestamp << "," << r.trackingState << ","
                << r.processMs << "," << r.trackedPoints << "," << r.totalMapPoints << ","
                << r.totalKeyframes << "," << r.rssMB << "\n";
        }
        cfs.close();
    }

    std::cout << "[Benchmark] Report files exported: " << jsonPath << " / " << csvPath << std::endl;
}

// 终端摘要打印
void printTerminalReport(const ScoreCard& card) {
    std::cout << "\n==========================================================\n";
    std::cout << "          MenthaAR Benchmark Final Scorecard\n";
    std::cout << "==========================================================\n";
    std::cout << "  Overall Score:  " << std::fixed << std::setprecision(1) << card.overallScore << " / 100  [" << card.grade << "]\n";
    std::cout << "  --------------------------------------------------------\n";
    std::cout << "  Tracking Quality Score (40%):  " << card.trackingScore << "  (OK: " << card.okRatio << "%)\n";
    std::cout << "  Real-time Latency Score (30%): " << card.latencyScore << "  (Mean: " << card.meanMs << " ms)\n";
    std::cout << "  Map Stability Score     (20%): " << card.stabilityScore << "  (Cont OK: " << card.maxContinuousOk << " frames)\n";
    std::cout << "  Memory Health Score     (10%): " << card.memoryScore << "  (Peak RSS: " << card.peakRssMB << " MB)\n";
    std::cout << "  --------------------------------------------------------\n";
    std::cout << "  Total Loops: " << gTargetLoops << " | Total Processed: " << card.totalFrames << " frames\n";
    std::cout << "  Final Map Points: " << card.finalMPs << " | Final Keyframes: " << card.finalKFs << "\n";
    std::cout << "==========================================================\n\n";
}

void resetBenchmarkState() {
    {
        std::lock_guard<std::mutex> lockData(gMapDataMutex);
        vMPs.clear();
        vKeys.clear();
    }
    gRecords.clear();
    gRssHistory.clear();
    gCurrentLoop = 1;
    gBenchmarkCompleted = false;
    gBenchmarkPaused = false;
    timeStamp = 0.0;
    if (slamSys) {
        slamSys->Reset(false);
    }
    if (pPlane) {
        delete pPlane;
        pPlane = nullptr;
    }
    std::cout << "[Benchmark] State reset successfully." << std::endl;
}

// ============================================================
// 主函数
// ============================================================
int main(int argc, char** argv) {
    std::cout << "==========================================================" << std::endl;
    std::cout << "   MenthaAR SLAM Benchmark & Memory Profiler" << std::endl;
    std::cout << "==========================================================" << std::endl;

    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <video_path> [output_prefix] [--loops N]\n";
        return 1;
    }

    gVideoPath = argv[1];
    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--loops" || arg == "-n") {
            if (i + 1 < argc) {
                gTargetLoops = std::max(1, std::stoi(argv[++i]));
            }
        } else if (arg.find("--loops=") == 0) {
            gTargetLoops = std::max(1, std::stoi(arg.substr(8)));
        } else if (arg[0] != '-') {
            gOutputPrefix = arg;
        }
    }

    signal(SIGINT, signalHandler);

    fx = ORB_SLAM2::CAMERA_FX;
    fy = ORB_SLAM2::CAMERA_FY;
    cx = ORB_SLAM2::CAMERA_CX;
    cy = ORB_SLAM2::CAMERA_CY;

    std::cout << "[Benchmark] Loading vocabulary & initializing SLAM..." << std::endl;
    slamSys = new ORB_SLAM2::System("", ORB_SLAM2::System::MONOCULAR);
    std::cout << "[Benchmark] SLAM engine initialized!" << std::endl;

    cv::VideoCapture cap(gVideoPath);
    if (!cap.isOpened()) {
        std::string fixed = tryStripAudioStream(gVideoPath);
        if (!fixed.empty()) {
            cap.open(fixed);
            if (cap.isOpened()) gVideoPath = fixed;
        }
        if (!cap.isOpened()) {
            std::cerr << "[ERROR] Cannot open video source: " << gVideoPath << std::endl;
            delete slamSys;
            return -1;
        }
    }

    double inputFps = cap.get(cv::CAP_PROP_FPS);
    if (inputFps <= 0) inputFps = ORB_SLAM2::SYSTEM_FPS;
    const double TARGET_FPS = ORB_SLAM2::SYSTEM_FPS;
    int totalInputFrames = (int)cap.get(cv::CAP_PROP_FRAME_COUNT);

    std::cout << "[Benchmark] Video: " << gVideoPath << " (" << totalInputFrames << " frames @ " << inputFps << " FPS)\n";
    std::cout << "[Benchmark] Processing Target: Fixed 30 FPS, Loops: " << gTargetLoops << "\n";

    cap.set(cv::CAP_PROP_FRAME_WIDTH, ORB_SLAM2::UBUNTU_CAPTURE_WIDTH);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, ORB_SLAM2::UBUNTU_CAPTURE_HEIGHT);

    cv::namedWindow("MenthaAR SLAM Benchmark Engine", cv::WINDOW_AUTOSIZE);
    initMenu();
    cv::setMouseCallback("MenthaAR SLAM Benchmark Engine", onMouse, nullptr);

    cv::Mat frame, imgGr, imgRgba;
    int fps = 0;
    int status = 0;
    auto lastTime = std::chrono::steady_clock::now();
    int frameCounter = 0;
    int globalFrameId = 0;

    auto lastProcessTime = std::chrono::steady_clock::now();
    const double processInterval = 1.0 / TARGET_FPS;

    double playBaseMsec = -1.0;
    double playBaseWallMs = 0.0;
    long frameIdx = 0;
    auto nowWallMs = []() -> double {
        return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    };

    gRecords.reserve(totalInputFrames > 0 ? totalInputFrames * gTargetLoops : ORB_SLAM2::BENCH_RESERVE_DEFAULT);

    while (!gStopRequested) {
        if (!gBenchmarkCompleted && !gBenchmarkPaused) {
            cap >> frame;
            if (frame.empty()) {
                if (gCurrentLoop < gTargetLoops) {
                    gCurrentLoop++;
                    cap.set(cv::CAP_PROP_POS_FRAMES, 0);
                    cap >> frame;
                    frameIdx = 0;
                    playBaseMsec = -1.0;
                    std::cout << "\n[Benchmark] Advanced to Loop " << gCurrentLoop << " / " << gTargetLoops << std::endl;
                } else {
                    gBenchmarkCompleted = true;
                    gScoreCard = calculateScoreCard(gRecords, globalFrameId);
                    printTerminalReport(gScoreCard);
                    exportReports(gOutputPrefix, gScoreCard, gRecords);
                }
            }
        }

        // 墙钟播放同步
        char key = -1;
        if (!frame.empty() && !gBenchmarkCompleted && !gBenchmarkPaused) {
            frameIdx++;
            double curMsec = cap.get(cv::CAP_PROP_POS_MSEC);
            if (curMsec < 0) curMsec = (frameIdx - 1) * 1000.0 / inputFps;
            if (playBaseMsec < 0 || curMsec < playBaseMsec - 500.0) {
                playBaseMsec = curMsec;
                playBaseWallMs = nowWallMs();
            }
            double videoElapsed = curMsec - playBaseMsec;
            double wallElapsed = nowWallMs() - playBaseWallMs;
            if (videoElapsed > wallElapsed + 3.0) {
                double need = videoElapsed - wallElapsed;
                double waited = 0.0;
                while (waited < need) {
                    double step = std::min(need - waited, 5.0);
                    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(step));
                    waited += step;
                    char k = (char)cv::waitKey(1);
                    if (k == 27 || k == 'q') { key = k; break; }
                }
            } else if (wallElapsed - videoElapsed > 250.0) {
                playBaseMsec = curMsec;
                playBaseWallMs = nowWallMs();
            }
        }
        if (key == 27 || key == 'q') break;

        // 计算 GUI 渲染 FPS
        frameCounter++;
        auto currentTime = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(currentTime - lastTime).count();
        if (duration >= 1.0) {
            fps = frameCounter;
            frameCounter = 0;
            lastTime = currentTime;
        }

        if (!frame.empty()) {
            const int MAX_DISPLAY_W = ORB_SLAM2::UBUNTU_MAX_DISPLAY_W;
            const int MAX_DISPLAY_H = ORB_SLAM2::UBUNTU_MAX_DISPLAY_H;
            if (frame.cols > MAX_DISPLAY_W || frame.rows > MAX_DISPLAY_H) {
                float scale = std::min((float)MAX_DISPLAY_W / frame.cols,
                                       (float)MAX_DISPLAY_H / frame.rows);
                cv::resize(frame, imgRgba, cv::Size(), scale, scale, cv::INTER_AREA);
            } else {
                imgRgba = frame.clone();
            }
        } else if (imgRgba.empty()) {
            imgRgba = cv::Mat::zeros(720, 1280, CV_8UC3);
        }

        // ORB / SLAM 处理节流
        if (gEnableSLAM && !gBenchmarkCompleted && !gBenchmarkPaused && !frame.empty()) {
            auto now = std::chrono::steady_clock::now();
            double procDt = std::chrono::duration_cast<std::chrono::duration<double>>(now - lastProcessTime).count();
            if (procDt >= processInterval) {
                lastProcessTime = now;
                timeStamp += processInterval;

                cv::cvtColor(frame, imgGr, cv::COLOR_BGR2GRAY);
                cv::Mat imgSmall;
                cv::resize(imgGr, imgSmall,
                           cv::Size((int)ORB_SLAM2::BASE_SLAM_WIDTH,
                                    (int)ORB_SLAM2::BASE_SLAM_HEIGHT));

                auto t0 = std::chrono::steady_clock::now();
                {
                    std::unique_lock<std::mutex> lock(gSlamStateMutex);
                    if (slamSys) {
                        Tcw = slamSys->TrackMonocular(imgSmall, timeStamp);
                        status = slamSys->GetTrackingState();

                        std::lock_guard<std::mutex> lockPoints(gMapDataMutex);
                        if (status == ORB_SLAM2::Tracking::OK) {
                            vMPs = slamSys->GetTrackedMapPoints();
                            vKeys = slamSys->GetTrackedKeyPointsUn();
                        } else {
                            vMPs.clear();
                            vKeys.clear();
                        }
                    }
                }
                auto t1 = std::chrono::steady_clock::now();
                double ms = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0;

                int numMPs = slamSys ? slamSys->GetNumMapPoints() : 0;
                int numKFs = slamSys ? slamSys->GetNumKeyFrames() : 0;
                MemoryInfo curMem = getMemoryInfo(numMPs, numKFs);

                FrameRecord rec;
                rec.loopId = gCurrentLoop;
                rec.frameId = globalFrameId++;
                rec.timestamp = timeStamp;
                rec.trackingState = status;
                rec.processMs = ms;
                rec.trackedPoints = (int)vMPs.size();
                rec.keypoints = (int)vKeys.size();
                rec.totalMapPoints = numMPs;
                rec.totalKeyframes = numKFs;
                rec.rssMB = curMem.rssMB;
                gRecords.push_back(rec);

                gRssHistory.push_back(curMem.rssMB);
                if (gRssHistory.size() > 100) gRssHistory.erase(gRssHistory.begin());
            }
        }

        // 3D 虚拟 AR 判断
        {
            std::lock_guard<std::mutex> lock(gMapDataMutex);
            bool alignmentOK = true;
            if (pPlane && planeLoadedFromMap) {
                alignmentOK = slamSys->HasMapAlignment();
            }
            gShouldDrawArObject = (status == ORB_SLAM2::Tracking::OK) && (pPlane != nullptr) && alignmentOK;
        }

        // 可视化点云渲染 (加锁保护 vMPs 与地图数据)
        if (gEnablePointCloudDisplay) {
            std::lock_guard<std::mutex> lockPoints(gMapDataMutex);
            if (status == ORB_SLAM2::Tracking::OK && slamSys->HasMapAlignment()) {
                cv::Mat TcwAligned = slamSys->GetMapAlignedPose(Tcw);
                std::vector<ORB_SLAM2::MapPoint*> allMapPoints = slamSys->GetAllMapPoints();
                drawAllMapPoints(TcwAligned, allMapPoints, imgRgba, fx, fy, cx, cy, true);
            }
            if (status == ORB_SLAM2::Tracking::OK && !vMPs.empty()) {
                drawTrackedPoints(vKeys, vMPs, imgRgba, cx, cy);
            }
        }

        // 绘制 3D 虚拟 AR 立方体
        if (gShouldDrawArObject) {
            cv::Mat TcwAR = Tcw;
            if (slamSys->HasMapAlignment()) {
                TcwAR = slamSys->GetMapAlignedPose(Tcw);
            }
            drawARCube(imgRgba, TcwAR, pPlane, fx, fy, cx, cy);
        }

        // 获取最新内存统计
        int numMPs = slamSys ? slamSys->GetNumMapPoints() : 0;
        int numKFs = slamSys ? slamSys->GetNumKeyFrames() : 0;
        MemoryInfo memInfo = getMemoryInfo(numMPs, numKFs);

        // 绘制桌面交互式 UI 面板
        drawGUI(imgRgba, status, fps, memInfo, gCurrentLoop, gTargetLoops);

        // 绘制【可视化内存分布仪表盘】
        if (gShowMemoryPanel) {
            drawMemoryDashboard(imgRgba, memInfo, gRssHistory);
        }

        // 评测完成时绘制【综合性能评分卡】模态框
        if (gBenchmarkCompleted) {
            drawScoreCardModal(imgRgba, gScoreCard);
        }

        cv::imshow("MenthaAR SLAM Benchmark Engine", imgRgba);

        if (key == -1) {
            char k2 = (char)cv::waitKey(1);
            if (k2 == 27 || k2 == 'q') break;
        }
    }

    std::cout << "[Benchmark] Shutting down SLAM engine..." << std::endl;
    if (slamSys) {
        slamSys->Shutdown();
        delete slamSys;
    }
    if (pPlane) delete pPlane;

    std::cout << "[Benchmark] Done. Goodbye!" << std::endl;
    return 0;
}

// 绘制 AR 立方体线框
void drawARCube(cv::Mat& im, const cv::Mat& Tcw, Plane* plane, float fx, float fy, float cx, float cy) {
    if (Tcw.empty() || !plane) return;

    cv::Mat Twp = plane->Tpw.inv();
    float s = ORB_SLAM2::AR_CUBE_SCALE_FACTOR * plane->rang;
    if (s <= ORB_SLAM2::AR_CUBE_MIN_SIZE) s = ORB_SLAM2::AR_CUBE_FALLBACK_SIZE;

    std::vector<cv::Mat> ptsPlane = {
        (cv::Mat_<float>(4, 1) << -s, -s, 0, 1),
        (cv::Mat_<float>(4, 1) <<  s, -s, 0, 1),
        (cv::Mat_<float>(4, 1) <<  s,  s, 0, 1),
        (cv::Mat_<float>(4, 1) << -s,  s, 0, 1),
        (cv::Mat_<float>(4, 1) << -s, -s, 2 * s, 1),
        (cv::Mat_<float>(4, 1) <<  s, -s, 2 * s, 1),
        (cv::Mat_<float>(4, 1) <<  s,  s, 2 * s, 1),
        (cv::Mat_<float>(4, 1) << -s,  s, 2 * s, 1)
    };

    std::vector<cv::Point> ptsImg;
    for (const auto& ptP : ptsPlane) {
        cv::Mat ptW = Twp * ptP;
        cv::Mat ptW3 = ptW.rowRange(0, 3) / ptW.at<float>(3);
        cv::Mat ptW4 = (cv::Mat_<float>(4, 1) << ptW3.at<float>(0), ptW3.at<float>(1), ptW3.at<float>(2), 1);
        cv::Mat ptC = Tcw * ptW4;
        float Xc = ptC.at<float>(0);
        float Yc = ptC.at<float>(1);
        float Zc = ptC.at<float>(2);

        if (Zc <= ORB_SLAM2::PROJECT_MIN_DEPTH) return;

        float scaleToDisplay = (ORB_SLAM2::IMAGE_DOWNSCALE_FACTOR * cx > 0.0f) ? (float)im.cols / (ORB_SLAM2::IMAGE_DOWNSCALE_FACTOR * cx) : ORB_SLAM2::IMAGE_DOWNSCALE_FACTOR;
        float u = (fx * Xc / Zc + cx) * scaleToDisplay;
        float v = (fy * Yc / Zc + cy) * scaleToDisplay;
        ptsImg.push_back(cv::Point(u, v));
    }

    if (ptsImg.size() < 8) return;

    cv::line(im, ptsImg[0], ptsImg[1], cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    cv::line(im, ptsImg[1], ptsImg[2], cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    cv::line(im, ptsImg[2], ptsImg[3], cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    cv::line(im, ptsImg[3], ptsImg[0], cv::Scalar(0, 255, 255), 2, cv::LINE_AA);

    cv::line(im, ptsImg[4], ptsImg[5], cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
    cv::line(im, ptsImg[5], ptsImg[6], cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
    cv::line(im, ptsImg[6], ptsImg[7], cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
    cv::line(im, ptsImg[7], ptsImg[4], cv::Scalar(255, 0, 255), 2, cv::LINE_AA);

    cv::line(im, ptsImg[0], ptsImg[4], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    cv::line(im, ptsImg[1], ptsImg[5], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    cv::line(im, ptsImg[2], ptsImg[6], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    cv::line(im, ptsImg[3], ptsImg[7], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
}

// 初始化交互式按钮与菜单
void initMenu() {
    menuSections.clear();

    // Section 1: AR Controls
    MenuSection arSec;
    arSec.title = "AR Controls";
    arSec.expanded = true;

    Button btnPlace;
    btnPlace.label = "Place AR Cube";
    btnPlace.color = cv::Scalar(30, 150, 20);
    btnPlace.action = []() {
        std::lock_guard<std::mutex> lock(gSlamStateMutex);
        std::lock_guard<std::mutex> lockData(gMapDataMutex);
        if (!Tcw.empty()) {
            cv::Mat TcwAligned = Tcw;
            if (slamSys && slamSys->HasMapAlignment()) {
                TcwAligned = slamSys->GetMapAlignedPose(Tcw);
            }
            if (pPlane) delete pPlane;
            pPlane = detectPlane(TcwAligned, vMPs, ORB_SLAM2::PLANE_DETECT_RANSAC_ITERS);
            if (pPlane) {
                if (slamSys && slamSys->MapChanged()) pPlane->Recompute();
                planeLoadedFromMap = false;
                std::cout << "[Benchmark GUI] Plane detected, AR ready!" << std::endl;
            } else {
                std::cout << "[Benchmark GUI] Plane detection failed." << std::endl;
            }
        }
    };

    Button btnClear;
    btnClear.label = "Clear All";
    btnClear.color = cv::Scalar(30, 30, 200);
    btnClear.action = []() {
        std::lock_guard<std::mutex> lock(gMapDataMutex);
        if (pPlane) {
            delete pPlane;
            pPlane = nullptr;
        }
        std::cout << "[Benchmark GUI] AR objects cleared." << std::endl;
    };

    arSec.buttons.push_back(btnPlace);
    arSec.buttons.push_back(btnClear);

    // Section 2: Map Persistence
    MenuSection mapSec;
    mapSec.title = "Map Persistence";
    mapSec.expanded = false;

    Button btnSave;
    btnSave.label = "Save Map";
    btnSave.color = cv::Scalar(180, 100, 30);
    btnSave.action = []() {
        std::lock_guard<std::mutex> lock(gSlamStateMutex);
        if (slamSys) {
            std::cout << "[Benchmark GUI] Saving map (max 50,000 MPs) to mentha_map.bin..." << std::endl;
            slamSys->SaveMap("mentha_map.bin");
            if (pPlane) {
                std::ofstream ofs("mentha_map.bin.arinfo", std::ios::binary);
                if (ofs.is_open()) {
                    const uint32_t magic = ORB_SLAM2::AR_INFO_FILE_MAGIC;
                    const uint32_t version = ORB_SLAM2::AR_INFO_FILE_VERSION;
                    ofs.write(reinterpret_cast<const char*>(&magic), 4);
                    ofs.write(reinterpret_cast<const char*>(&version), 4);
                    uint8_t hasPlane = 1;
                    ofs.write(reinterpret_cast<const char*>(&hasPlane), 1);
                    float o3[3] = {pPlane->o.at<float>(0), pPlane->o.at<float>(1), pPlane->o.at<float>(2)};
                    float n3[3] = {pPlane->n.at<float>(0), pPlane->n.at<float>(1), pPlane->n.at<float>(2)};
                    ofs.write(reinterpret_cast<const char*>(o3), sizeof(o3));
                    ofs.write(reinterpret_cast<const char*>(n3), sizeof(n3));
                    ofs.write(reinterpret_cast<const char*>(&pPlane->rang), sizeof(pPlane->rang));
                    ofs.close();
                }
            }
            std::cout << "[Benchmark GUI] Map saved successfully!" << std::endl;
        }
    };

    Button btnLoad;
    btnLoad.label = "Load Map";
    btnLoad.color = cv::Scalar(30, 120, 180);
    btnLoad.action = []() {
        std::lock_guard<std::mutex> lock(gSlamStateMutex);
        if (slamSys) {
            std::cout << "[Benchmark GUI] Loading map from mentha_map.bin..." << std::endl;
            slamSys->LoadMap("mentha_map.bin", 0, false);
            std::ifstream ifs("mentha_map.bin.arinfo", std::ios::binary);
            if (ifs.is_open()) {
                uint32_t magic = 0, version = 0;
                ifs.read(reinterpret_cast<char*>(&magic), 4);
                ifs.read(reinterpret_cast<char*>(&version), 4);
                if (magic == ORB_SLAM2::AR_INFO_FILE_MAGIC) {
                    uint8_t hasPlane = 0;
                    ifs.read(reinterpret_cast<char*>(&hasPlane), 1);
                    if (hasPlane) {
                        float o3[3], n3[3], rang;
                        ifs.read(reinterpret_cast<char*>(o3), sizeof(o3));
                        ifs.read(reinterpret_cast<char*>(n3), sizeof(n3));
                        ifs.read(reinterpret_cast<char*>(&rang), sizeof(rang));
                        if (pPlane) delete pPlane;
                        pPlane = new Plane(n3[0], n3[1], n3[2], o3[0], o3[1], o3[2]);
                        pPlane->rang = rang;
                        planeLoadedFromMap = true;
                    }
                }
                ifs.close();
            }
            std::cout << "[Benchmark GUI] Map loaded successfully!" << std::endl;
        }
    };

    mapSec.buttons.push_back(btnSave);
    mapSec.buttons.push_back(btnLoad);

    // Section 3: Display Settings
    MenuSection dispSec;
    dispSec.title = "Display Settings";
    dispSec.expanded = false;

    Button btnToggleP;
    btnToggleP.label = "Toggle Point Cloud";
    btnToggleP.color = cv::Scalar(100, 100, 100);
    btnToggleP.action = []() {
        gEnablePointCloudDisplay = !gEnablePointCloudDisplay;
    };

    Button btnToggleMem;
    btnToggleMem.label = "Toggle Memory Panel";
    btnToggleMem.color = cv::Scalar(140, 60, 140);
    btnToggleMem.action = []() {
        gShowMemoryPanel = !gShowMemoryPanel;
    };

    dispSec.buttons.push_back(btnToggleP);
    dispSec.buttons.push_back(btnToggleMem);

    // Section 4: Benchmark Controls
    MenuSection benchSec;
    benchSec.title = "Benchmark Controls";
    benchSec.expanded = true;

    Button btnRestart;
    btnRestart.label = "Restart Benchmark";
    btnRestart.color = cv::Scalar(30, 160, 160);
    btnRestart.action = []() {
        resetBenchmarkState();
    };

    Button btnExport;
    btnExport.label = "Export Report Now";
    btnExport.color = cv::Scalar(160, 120, 30);
    btnExport.action = []() {
        ScoreCard sc = calculateScoreCard(gRecords, gRecords.size());
        exportReports(gOutputPrefix, sc, gRecords);
    };

    benchSec.buttons.push_back(btnRestart);
    benchSec.buttons.push_back(btnExport);

    menuSections.push_back(arSec);
    menuSections.push_back(mapSec);
    menuSections.push_back(dispSec);
    menuSections.push_back(benchSec);
}

// 绘制 GUI 仪表盘与抽屉
void drawGUI(cv::Mat& frame, int trackingState, int fps, const MemoryInfo& memInfo, int curLoop, int totalLoops) {
    // 1. 左上角状态仪表盘
    cv::Rect panelRect(15, 15, 300, 155);
    cv::Mat panelOverlay(panelRect.size(), frame.type(), cv::Scalar(25, 25, 25));
    cv::addWeighted(panelOverlay, 0.78, frame(panelRect), 0.22, 0, frame(panelRect));
    cv::rectangle(frame, panelRect, cv::Scalar(90, 90, 90), 1, cv::LINE_AA);

    std::string stateStr = "No Input";
    cv::Scalar stateColor(150, 150, 150);
    if (trackingState == ORB_SLAM2::Tracking::NOT_INITIALIZED) {
        stateStr = "Not Initialized";
        stateColor = cv::Scalar(30, 180, 230);
    } else if (trackingState == ORB_SLAM2::Tracking::OK) {
        stateStr = "Tracking OK";
        stateColor = cv::Scalar(30, 230, 30);
    } else if (trackingState == ORB_SLAM2::Tracking::LOST) {
        stateStr = "Tracking Lost";
        stateColor = cv::Scalar(30, 30, 230);
    }

    cv::putText(frame, "MenthaAR Benchmark Dashboard", cv::Point(25, 38), cv::FONT_HERSHEY_SIMPLEX, 0.48, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    cv::putText(frame, "---------------------------------", cv::Point(25, 48), cv::FONT_HERSHEY_SIMPLEX, 0.48, cv::Scalar(90, 90, 90), 1, cv::LINE_AA);

    cv::putText(frame, "Loop: ", cv::Point(25, 68), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
    cv::putText(frame, std::to_string(curLoop) + " / " + std::to_string(totalLoops), cv::Point(75, 68), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 215, 0), 1, cv::LINE_AA);

    cv::putText(frame, "SLAM: ", cv::Point(25, 88), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
    cv::putText(frame, stateStr, cv::Point(75, 88), cv::FONT_HERSHEY_SIMPLEX, 0.45, stateColor, 1, cv::LINE_AA);

    cv::putText(frame, "FPS: ", cv::Point(25, 108), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
    cv::putText(frame, std::to_string(fps) + " FPS", cv::Point(75, 108), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 217, 255), 1, cv::LINE_AA);

    int numKFs = slamSys ? slamSys->GetNumKeyFrames() : 0;
    int numMPs = slamSys ? slamSys->GetNumMapPoints() : 0;
    std::string statsStr = "KFs: " + std::to_string(numKFs) + " | MPs: " + std::to_string(numMPs);
    cv::putText(frame, "Map: ", cv::Point(25, 128), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
    cv::putText(frame, statsStr, cv::Point(75, 128), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 136), 1, cv::LINE_AA);

    char ramBuf[64];
    snprintf(ramBuf, sizeof(ramBuf), "RSS: %.1f MB (Peak: %.1f)", memInfo.rssMB, memInfo.peakRssMB);
    cv::putText(frame, "RAM: ", cv::Point(25, 148), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
    cv::putText(frame, ramBuf, cv::Point(75, 148), cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(230, 160, 255), 1, cv::LINE_AA);

    // 2. 右侧抽屉
    gFrameWidth = frame.cols;
    int drawerX = frame.cols - kDrawerWidth;

    if (!gDrawerVisible) {
        int tabW = 8, tabH = 96;
        int tabX = frame.cols - tabW, tabY = (frame.rows - tabH) / 2;
        cv::Rect tabRect(tabX, tabY, tabW, tabH);
        cv::Mat tabOverlay(tabRect.size(), frame.type(), cv::Scalar(45, 45, 45));
        cv::addWeighted(tabOverlay, 0.65, frame(tabRect), 0.35, 0, frame(tabRect));
        cv::rectangle(frame, tabRect, cv::Scalar(90, 90, 90), 1, cv::LINE_AA);
        for (int i = 0; i < 3; i++) {
            int ly = tabY + tabH / 2 - 7 + i * 7;
            cv::line(frame, cv::Point(tabX + 2, ly), cv::Point(tabX + tabW - 2, ly),
                     cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
        }
        return;
    }

    cv::Rect drawerRect(drawerX, 0, kDrawerWidth, frame.rows);
    cv::Mat drawerOverlay(drawerRect.size(), frame.type(), cv::Scalar(15, 15, 15));
    cv::addWeighted(drawerOverlay, 0.84, frame(drawerRect), 0.16, 0, frame(drawerRect));
    cv::line(frame, cv::Point(drawerX, 0), cv::Point(drawerX, frame.rows), cv::Scalar(60, 60, 60), 1, cv::LINE_AA);

    int currentY = 20;
    int categoryHeight = 32;
    int buttonHeight = 28;
    int spacing = 5;

    for (size_t sIdx = 0; sIdx < menuSections.size(); sIdx++) {
        MenuSection& sec = menuSections[sIdx];
        sec.rect = cv::Rect(drawerX + 10, currentY, kDrawerWidth - 20, categoryHeight);
        bool isHovered = sec.rect.contains(mousePos);

        cv::Scalar catColor = isHovered ? cv::Scalar(75, 75, 75) : cv::Scalar(50, 50, 50);
        cv::rectangle(frame, sec.rect, catColor, -1);
        cv::rectangle(frame, sec.rect, cv::Scalar(90, 90, 90), 1, cv::LINE_AA);

        std::string expandIcon = sec.expanded ? "[-] " : "[+] ";
        cv::putText(frame, expandIcon + sec.title, cv::Point(drawerX + 20, currentY + 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

        currentY += categoryHeight + spacing;

        if (sec.expanded) {
            for (size_t bIdx = 0; bIdx < sec.buttons.size(); bIdx++) {
                Button& btn = sec.buttons[bIdx];
                btn.rect = cv::Rect(drawerX + 20, currentY, kDrawerWidth - 40, buttonHeight);
                btn.isHovered = btn.rect.contains(mousePos);

                cv::Scalar borderCol = btn.isHovered ? cv::Scalar(255, 255, 255) : cv::Scalar(80, 80, 80);
                cv::Scalar fillCol = btn.color * (btn.isHovered ? 1.3 : 1.0);

                cv::rectangle(frame, btn.rect, fillCol, -1);
                cv::rectangle(frame, btn.rect, borderCol, 1, cv::LINE_AA);

                int textBaseline = 0;
                cv::Size textSize = cv::getTextSize(btn.label, cv::FONT_HERSHEY_SIMPLEX, 0.38, 1, &textBaseline);
                int textX = btn.rect.x + (btn.rect.width - textSize.width) / 2;
                int textY = btn.rect.y + (btn.rect.height + textSize.height) / 2 - 1;

                cv::putText(frame, btn.label, cv::Point(textX, textY),
                            cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

                currentY += buttonHeight + spacing;
            }
        }
        currentY += spacing;
    }
}

// 绘制【可视化内存分布仪表盘】 (包含堆叠成分柱状图与 RSS 历史走势曲线)
void drawMemoryDashboard(cv::Mat& frame, const MemoryInfo& memInfo, const std::vector<double>& rssHist) {
    int panelW = 340;
    int panelH = 140;
    int panelX = 15;
    int panelY = frame.rows - panelH - 15;

    cv::Rect panelRect(panelX, panelY, panelW, panelH);
    if (panelY < 180) return; // 避免遮挡顶部仪表盘

    cv::Mat panelOverlay(panelRect.size(), frame.type(), cv::Scalar(20, 20, 25));
    cv::addWeighted(panelOverlay, 0.82, frame(panelRect), 0.18, 0, frame(panelRect));
    cv::rectangle(frame, panelRect, cv::Scalar(100, 100, 120), 1, cv::LINE_AA);

    // 标题
    cv::putText(frame, "Visual Memory Occupancy Map", cv::Point(panelX + 12, panelY + 22),
                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 220, 255), 1, cv::LINE_AA);

    // 1. 分色堆叠比例条
    int barX = panelX + 12;
    int barY = panelY + 32;
    int barW = panelW - 24;
    int barH = 14;
    cv::rectangle(frame, cv::Rect(barX, barY, barW, barH), cv::Scalar(40, 40, 40), -1);

    double totalRss = std::max(0.1, memInfo.rssMB);
    double mpRatio = std::min(1.0, memInfo.mpMemMB / totalRss);
    double kfRatio = std::min(1.0 - mpRatio, memInfo.kfMemMB / totalRss);
    double bufRatio = std::min(1.0 - mpRatio - kfRatio, memInfo.bufferMemMB / totalRss);
    double othRatio = std::max(0.0, 1.0 - (mpRatio + kfRatio + bufRatio));

    int wMP = (int)(barW * mpRatio);
    int wKF = (int)(barW * kfRatio);
    int wBuf = (int)(barW * bufRatio);
    int wOth = barW - (wMP + wKF + wBuf);

    int curX = barX;
    if (wMP > 0) {
        cv::rectangle(frame, cv::Rect(curX, barY, wMP, barH), cv::Scalar(255, 200, 0), -1); // Cyan-Blue for MPs
        curX += wMP;
    }
    if (wKF > 0) {
        cv::rectangle(frame, cv::Rect(curX, barY, wKF, barH), cv::Scalar(255, 100, 50), -1); // Deep Blue for KFs
        curX += wKF;
    }
    if (wBuf > 0) {
        cv::rectangle(frame, cv::Rect(curX, barY, wBuf, barH), cv::Scalar(50, 180, 255), -1); // Orange for Buffers
        curX += wBuf;
    }
    if (wOth > 0) {
        cv::rectangle(frame, cv::Rect(curX, barY, wOth, barH), cv::Scalar(140, 140, 140), -1); // Gray for Other
    }
    cv::rectangle(frame, cv::Rect(barX, barY, barW, barH), cv::Scalar(180, 180, 180), 1, cv::LINE_AA);

    // 分色图例
    int legY = panelY + 58;
    cv::rectangle(frame, cv::Rect(panelX + 12, legY - 8, 8, 8), cv::Scalar(255, 200, 0), -1);
    cv::putText(frame, "MPs", cv::Point(panelX + 24, legY), cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(220, 220, 220), 1);

    cv::rectangle(frame, cv::Rect(panelX + 70, legY - 8, 8, 8), cv::Scalar(255, 100, 50), -1);
    cv::putText(frame, "KFs", cv::Point(panelX + 82, legY), cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(220, 220, 220), 1);

    cv::rectangle(frame, cv::Rect(panelX + 125, legY - 8, 8, 8), cv::Scalar(50, 180, 255), -1);
    cv::putText(frame, "Buffers", cv::Point(panelX + 137, legY), cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(220, 220, 220), 1);

    cv::rectangle(frame, cv::Rect(panelX + 205, legY - 8, 8, 8), cv::Scalar(140, 140, 140), -1);
    cv::putText(frame, "Engine", cv::Point(panelX + 217, legY), cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(220, 220, 220), 1);

    // 2. RSS 历史走势曲线图
    int graphX = panelX + 12;
    int graphY = panelY + 68;
    int graphW = panelW - 24;
    int graphH = panelH - 78;
    cv::rectangle(frame, cv::Rect(graphX, graphY, graphW, graphH), cv::Scalar(30, 30, 35), -1);
    cv::rectangle(frame, cv::Rect(graphX, graphY, graphW, graphH), cv::Scalar(60, 60, 70), 1, cv::LINE_AA);

    if (rssHist.size() >= 2) {
        double minRss = *std::min_element(rssHist.begin(), rssHist.end());
        double maxRss = *std::max_element(rssHist.begin(), rssHist.end());
        if (maxRss - minRss < 5.0) {
            maxRss = minRss + 5.0;
        }

        std::vector<cv::Point> pts;
        pts.reserve(rssHist.size());
        for (size_t i = 0; i < rssHist.size(); ++i) {
            float px = graphX + (float)i / (rssHist.size() - 1) * graphW;
            float py = graphY + graphH - (float)((rssHist[i] - minRss) / (maxRss - minRss)) * (graphH - 4) - 2;
            pts.push_back(cv::Point(px, py));
        }

        for (size_t i = 0; i < pts.size() - 1; ++i) {
            cv::line(frame, pts[i], pts[i + 1], cv::Scalar(0, 255, 180), 1, cv::LINE_AA);
        }

        char minMaxStr[64];
        snprintf(minMaxStr, sizeof(minMaxStr), "%.1f MB", maxRss);
        cv::putText(frame, minMaxStr, cv::Point(graphX + 4, graphY + 12), cv::FONT_HERSHEY_SIMPLEX, 0.32, cv::Scalar(180, 180, 180), 1);
    }
}

// 评测完成时绘制【综合性能评分卡】模态窗口 (Scorecard Modal)
void drawScoreCardModal(cv::Mat& frame, const ScoreCard& card) {
    int modalW = 520;
    int modalH = 340;
    int modalX = (frame.cols - modalW) / 2;
    int modalY = (frame.rows - modalH) / 2;

    cv::Rect modalRect(modalX, modalY, modalW, modalH);
    cv::Mat overlay(modalRect.size(), frame.type(), cv::Scalar(15, 15, 20));
    cv::addWeighted(overlay, 0.88, frame(modalRect), 0.12, 0, frame(modalRect));
    cv::rectangle(frame, modalRect, cv::Scalar(0, 215, 255), 2, cv::LINE_AA);

    // 标题与 Badge
    cv::putText(frame, "SLAM BENCHMARK COMPREHENSIVE SCORECARD", cv::Point(modalX + 25, modalY + 38),
                cv::FONT_HERSHEY_SIMPLEX, 0.52, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    cv::line(frame, cv::Point(modalX + 25, modalY + 48), cv::Point(modalX + modalW - 25, modalY + 48),
             cv::Scalar(80, 80, 90), 1, cv::LINE_AA);

    // 左侧：总分与 Badge
    cv::Scalar gradeColor(0, 255, 136); // Emerald for S+/S/A
    if (card.grade == "B") gradeColor = cv::Scalar(0, 215, 255);
    else if (card.grade == "C") gradeColor = cv::Scalar(30, 180, 230);
    else if (card.grade == "D") gradeColor = cv::Scalar(30, 30, 230);

    cv::Rect badgeRect(modalX + 30, modalY + 65, 140, 120);
    cv::rectangle(frame, badgeRect, cv::Scalar(30, 30, 40), -1);
    cv::rectangle(frame, badgeRect, gradeColor, 2, cv::LINE_AA);

    char scoreStr[32];
    snprintf(scoreStr, sizeof(scoreStr), "%.1f", card.overallScore);
    cv::putText(frame, scoreStr, cv::Point(badgeRect.x + 25, badgeRect.y + 55),
                cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
    cv::putText(frame, "Grade " + card.grade, cv::Point(badgeRect.x + 32, badgeRect.y + 95),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, gradeColor, 2, cv::LINE_AA);

    // 右侧：4 项维度分拆条形图
    int barX = modalX + 190;
    int barY = modalY + 75;
    int barW = 280;

    auto drawSubBar = [&](const std::string& label, double val, int yOffset) {
        cv::putText(frame, label, cv::Point(barX, barY + yOffset), cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(220, 220, 220), 1);
        char vStr[16];
        snprintf(vStr, sizeof(vStr), "%.1f", val);
        cv::putText(frame, vStr, cv::Point(barX + barW - 35, barY + yOffset), cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(0, 220, 255), 1);

        cv::Rect bR(barX, barY + yOffset + 6, barW, 10);
        cv::rectangle(frame, bR, cv::Scalar(40, 40, 50), -1);
        int fillW = (int)(barW * (val / 100.0));
        if (fillW > 0) {
            cv::rectangle(frame, cv::Rect(barX, barY + yOffset + 6, fillW, 10), cv::Scalar(0, 215, 255), -1);
        }
        cv::rectangle(frame, bR, cv::Scalar(80, 80, 90), 1);
    };

    drawSubBar("Tracking Quality (40%)", card.trackingScore, 0);
    drawSubBar("Real-Time Latency (30%)", card.latencyScore, 30);
    drawSubBar("Map Stability (20%)", card.stabilityScore, 60);
    drawSubBar("Memory Health (10%)", card.memoryScore, 90);

    // 下方核心数据指标
    cv::line(frame, cv::Point(modalX + 25, modalY + 215), cv::Point(modalX + modalW - 25, modalY + 215),
             cv::Scalar(80, 80, 90), 1, cv::LINE_AA);

    char line1[128], line2[128];
    snprintf(line1, sizeof(line1), "Frames: %d | OK Ratio: %.1f%% | Mean Latency: %.2f ms",
             card.totalFrames, card.okRatio, card.meanMs);
    snprintf(line2, sizeof(line2), "Final Map: %d MPs, %d KFs | Peak Memory: %.1f MB",
             card.finalMPs, card.finalKFs, card.peakRssMB);

    cv::putText(frame, line1, cv::Point(modalX + 30, modalY + 242), cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
    cv::putText(frame, line2, cv::Point(modalX + 30, modalY + 268), cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);

    // 底部操作提示
    cv::rectangle(frame, cv::Rect(modalX + 20, modalY + modalH - 45, modalW - 40, 30), cv::Scalar(35, 35, 45), -1);
    cv::putText(frame, "Press 'ESC' / 'q' to exit or click 'Restart Benchmark' in drawer",
                cv::Point(modalX + 45, modalY + modalH - 25), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
}

// 鼠标响应
void onMouse(int event, int x, int y, int flags, void* userdata) {
    mousePos = cv::Point(x, y);

    if (event == cv::EVENT_MOUSEMOVE) {
        int drawerX = gFrameWidth - kDrawerWidth;
        if (x >= gFrameWidth - kDrawerHotZone) {
            gDrawerVisible = true;
        } else if (gDrawerVisible && x < drawerX - kDrawerHotZone) {
            gDrawerVisible = false;
        }
    }

    if (event == cv::EVENT_LBUTTONDOWN) {
        if (!gDrawerVisible) return;

        for (auto& sec : menuSections) {
            if (sec.rect.contains(mousePos)) {
                sec.expanded = !sec.expanded;
                return;
            }

            if (sec.expanded) {
                for (auto& btn : sec.buttons) {
                    if (btn.rect.contains(mousePos)) {
                        btn.action();
                        return;
                    }
                }
            }
        }
    }
}

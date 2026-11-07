/*
 * 离线回放基准：在设备上用固定的灰度帧序列驱动 ORB_SLAM2::System，
 * 复刻 native-lib.cpp::processImage 的帧路径（缩放 1/2 → TrackMonocular）。
 *
 * 用法：
 *   slam_bench <frames.raw> <width> <height> <out.txt> [maxFrames] [sync=1] [repeat=1]
 *
 * 输出（每帧一行）：
 *   idx state nTracked ms nKF nMP  r00 r01 r02 t0 r10 r11 r12 t1 r20 r21 r22 t2   (12 个 float 的十六进制位模式)
 *   （sync 模式下 nKF/nMP 是后端处理完本帧之后的数量）
 * 末尾汇总：帧数、关键帧数、地图点数、各状态帧数、Tracking 耗时统计。
 *
 * sync=1：每帧之后等待局部建图与闭环线程空闲，使结果与线程调度无关，可逐位比较。
 */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <vector>
#include <string>
#include <chrono>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "System.h"
#include "Config.h"

static inline uint32_t f2u(float f) { uint32_t u; std::memcpy(&u, &f, 4); return u; }

int main(int argc, char** argv)
{
    if (argc < 5) {
        std::fprintf(stderr, "usage: %s <frames.raw> <w> <h> <out.txt> [maxFrames] [sync] [repeat]\n", argv[0]);
        return 2;
    }
    const char* rawPath = argv[1];
    const int W = std::atoi(argv[2]);
    const int H = std::atoi(argv[3]);
    const char* outPath = argv[4];
    const long maxFrames = argc > 5 ? std::atol(argv[5]) : -1;
    const bool sync = argc > 6 ? std::atoi(argv[6]) != 0 : true;
    const int repeat = argc > 7 ? std::atoi(argv[7]) : 1;

    FILE* fin = std::fopen(rawPath, "rb");
    if (!fin) { std::perror("open frames"); return 1; }
    FILE* fout = std::fopen(outPath, "w");
    if (!fout) { std::perror("open out"); return 1; }

    const size_t frameBytes = (size_t)W * H;
    std::vector<unsigned char> buf(frameBytes);

    std::fseek(fin, 0, SEEK_END);
    const long fileBytes = std::ftell(fin);
    std::fseek(fin, 0, SEEK_SET);
    long nFrames = fileBytes / (long)frameBytes;
    if (maxFrames > 0 && maxFrames < nFrames) nFrames = maxFrames;

    // 与 processImage 相同的缩放与内参处理
    const float DOWNSCALE = ORB_SLAM2::IMAGE_DOWNSCALE_FACTOR;
    const int sw = (int)std::lround((double)W / DOWNSCALE);
    const int sh = (int)std::lround((double)H / DOWNSCALE);
    const float sx = (float)sw / ORB_SLAM2::BASE_SLAM_WIDTH;
    const float sy = (float)sh / ORB_SLAM2::BASE_SLAM_HEIGHT;

    ORB_SLAM2::System slam("", ORB_SLAM2::System::MONOCULAR);
    slam.UpdateCalibration(ORB_SLAM2::CAMERA_FX * sx, ORB_SLAM2::CAMERA_FY * sy,
                           ORB_SLAM2::CAMERA_CX * sx, ORB_SLAM2::CAMERA_CY * sy);

    // 环境变量：SLAM_LOAD_MAP=<file> 回放前加载地图（复刻 App 的"加载地图后继续扫描"路径）；
    //           SLAM_SAVE_MAP=<file> 回放结束后保存地图，用于生成上述场景的输入
    // SLAM_LOAD_MAP_AT=<frameIdx>：在第 frameIdx 帧之前加载（默认 0，即回放开始前），用于复刻"先扫描再加载"
    // SLAM_LOAD_MAP_ID=<id>：复刻 App 的 loadMapWithId(path, mapId, ...)（默认 0）；非 0 时加载点的 mnMapId 与 Map::mnId 不同
    const char* loadMapPath = std::getenv("SLAM_LOAD_MAP");
    const char* saveMapPath = std::getenv("SLAM_SAVE_MAP");
    const char* loadAtStr = std::getenv("SLAM_LOAD_MAP_AT");
    const long loadAt = loadAtStr ? std::atol(loadAtStr) : 0;
    const char* loadMapIdStr = std::getenv("SLAM_LOAD_MAP_ID");
    const int loadMapId = loadMapIdStr ? std::atoi(loadMapIdStr) : 0;
    auto doLoadMap = [&]() {
        slam.LoadMap(loadMapPath, loadMapId, false);
        slam.WaitForBackendIdle(20000);
        std::fprintf(fout, "# loaded_map=%s at_frame=%ld map_id=%d keyframes=%d mappoints=%d\n", loadMapPath, loadAt, loadMapId,
                     slam.GetNumKeyFrames(), slam.GetNumMapPoints());
    };
    if (loadMapPath && loadMapPath[0] && loadAt <= 0) doLoadMap();

    cv::Mat imgSmall;
    double timeStamp = 0.0;
    std::vector<double> msTrack;
    msTrack.reserve((size_t)nFrames * repeat);
    int stateCount[8] = {0};
    long nSyncTimeouts = 0;
    double backendTotal = 0.0; // 同步模式下等待后端的时间（≈ 每个关键帧的 LM+LC 处理时间）
    long backendCount = 0;
    long frameIdx = 0;

    const auto tAll0 = std::chrono::steady_clock::now();
    for (int rep = 0; rep < repeat; ++rep) {
        std::fseek(fin, 0, SEEK_SET);
        for (long i = 0; i < nFrames; ++i) {
            if (std::fread(buf.data(), 1, frameBytes, fin) != frameBytes) break;
            if (loadMapPath && loadMapPath[0] && loadAt > 0 && frameIdx == loadAt) doLoadMap();
            cv::Mat gray(H, W, CV_8UC1, buf.data());
            timeStamp += 1.0 / ORB_SLAM2::SYSTEM_FPS;

            const auto t0 = std::chrono::steady_clock::now();
            cv::resize(gray, imgSmall, cv::Size(sw, sh), 0, 0, cv::INTER_LINEAR);
            cv::Mat Tcw = slam.TrackMonocular(imgSmall, timeStamp);
            const auto t1 = std::chrono::steady_clock::now();
            const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
            msTrack.push_back(ms);

            const int state = slam.GetTrackingState();
            if (state >= 0 && state < 8) stateCount[state]++;
            int nTracked = 0;
            {
                std::vector<ORB_SLAM2::MapPoint*> v = slam.GetTrackedMapPoints();
                for (auto p : v) if (p) nTracked++;
            }

            if (sync) {
                const auto tb0 = std::chrono::steady_clock::now();
                if (!slam.WaitForBackendIdle(20000)) nSyncTimeouts++;
                const double backendMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tb0).count();
                if (backendMs > 2.0) { backendTotal += backendMs; backendCount++; } // 只统计真正处理了关键帧的等待
            }

            std::fprintf(fout, "%ld %d %d %.3f %d %d", frameIdx, state, nTracked, ms,
                         slam.GetNumKeyFrames(), slam.GetNumMapPoints());
            if (!Tcw.empty() && Tcw.rows >= 3 && Tcw.cols >= 4) {
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 4; ++c)
                        std::fprintf(fout, " %08x", f2u(Tcw.at<float>(r, c)));
            } else {
                std::fprintf(fout, " -");
            }
            std::fprintf(fout, "\n");
            frameIdx++;
        }
    }
    // 等后端把最后的关键帧处理完再统计
    slam.WaitForBackendIdle(20000);
    const auto tAll1 = std::chrono::steady_clock::now();
    if (saveMapPath && saveMapPath[0]) {
        slam.SaveMap(saveMapPath);
        std::fprintf(fout, "# saved_map=%s\n", saveMapPath);
    }

    const int nKF = slam.GetNumKeyFrames();
    const int nMP = slam.GetNumMapPoints();

    std::vector<double> sorted(msTrack);
    std::sort(sorted.begin(), sorted.end());
    double sum = 0; for (double v : msTrack) sum += v;
    auto pct = [&](double p) { return sorted.empty() ? 0.0 : sorted[std::min(sorted.size() - 1, (size_t)(p * sorted.size()))]; };

    std::fprintf(fout, "# frames=%ld keyframes=%d mappoints=%d sync=%d syncTimeouts=%ld\n",
                 frameIdx, nKF, nMP, sync ? 1 : 0, nSyncTimeouts);
    std::fprintf(fout, "# states: NO_IMAGES=%d NOT_INIT=%d OK=%d LOST=%d\n",
                 stateCount[0], stateCount[1], stateCount[2], stateCount[3]);
    std::fprintf(fout, "# backend: keyframe_waits=%ld total_ms=%.1f mean_ms=%.2f\n", backendCount, backendTotal,
                 backendCount ? backendTotal / backendCount : 0.0);
    std::fprintf(fout, "# track_ms: mean=%.3f p50=%.3f p90=%.3f p99=%.3f max=%.3f total_wall_s=%.3f\n",
                 msTrack.empty() ? 0.0 : sum / msTrack.size(), pct(0.5), pct(0.9), pct(0.99),
                 sorted.empty() ? 0.0 : sorted.back(),
                 std::chrono::duration<double>(tAll1 - tAll0).count());
    std::fclose(fout);
    std::fclose(fin);

    std::printf("frames=%ld keyframes=%d mappoints=%d OK=%d LOST=%d mean_ms=%.3f p50=%.3f p90=%.3f backend_ms=%.0f wall_s=%.2f syncTimeouts=%ld\n",
                frameIdx, nKF, nMP, stateCount[2], stateCount[3],
                msTrack.empty() ? 0.0 : sum / msTrack.size(), pct(0.5), pct(0.9), backendTotal,
                std::chrono::duration<double>(tAll1 - tAll0).count(), nSyncTimeouts);

    slam.Shutdown();
    return 0;
}

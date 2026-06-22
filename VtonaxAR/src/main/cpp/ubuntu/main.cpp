/**
 * VtonaxAR SLAM
 */

#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <mutex>
#include <thread>
#include <cmath>
#include <opencv2/opencv.hpp>

#include "include/System.h"
#include "Common.h"
#include "Plane.h"
#include "UIUtils.h"
#include "Matrix.h"
#include "include/Config.h"
#include "EmbeddedResources.h"

// 定义与 JNI 类似的全局状态变量，供 Ubuntu 桌面应用程序使用
ORB_SLAM2::System* slamSys = nullptr;
Plane* pPlane = nullptr;
bool planeLoadedFromMap = false;

float fx, fy, cx, cy;
double timeStamp = 0.0;
bool gEnableSLAM = true;
bool gEnablePointCloudDisplay = true;
bool gShouldDrawArObject = false;

std::mutex gSlamStateMutex;
std::mutex gMapDataMutex;

std::vector<ORB_SLAM2::MapPoint*> vMPs;
std::vector<cv::KeyPoint> vKeys;
cv::Mat Tcw;

// 交互式 GUI 的界面状态结构体
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

// 前置函数声明
void initMenu();
void drawGUI(cv::Mat& frame, int trackingState, int fps);
void onMouse(int event, int x, int y, int flags, void* userdata);
void drawARCube(cv::Mat& im, const cv::Mat& Tcw, Plane* plane, float fx, float fy, float cx, float cy);

int main(int argc, char** argv) {
    std::cout << "==========================================================" << std::endl;
    std::cout << "   VtonaxAR 单目 SLAM" << std::endl;
    std::cout << "==========================================================" << std::endl;

    // 默认相机索引或视频文件路径
    std::string videoSource = "0";
    if (argc > 1) {
        videoSource = argv[1];
        std::cout << "[Ubuntu GUI] 使用自定义视频源: " << videoSource << std::endl;
    } else {
        std::cout << "[Ubuntu GUI] 使用默认系统相机 (设备索引: 0)" << std::endl;
    }

    // 从项目内置的 Config.h 加载相机内参
    fx = ORB_SLAM2::CAMERA_FX;
    fy = ORB_SLAM2::CAMERA_FY;
    cx = ORB_SLAM2::CAMERA_CX;
    cy = ORB_SLAM2::CAMERA_CY;

    // 采用与 Android 相同的嵌入式资源模式加载 SLAM 系统
    std::cout << "[Ubuntu GUI] 正在从程序内嵌资源中解压并加载词汇表..." << std::endl;
    slamSys = new ORB_SLAM2::System(":embedded:", "", ORB_SLAM2::System::MONOCULAR);
    std::cout << "[Ubuntu GUI] SLAM 引擎初始化成功，已就绪！" << std::endl;

    // 初始化 OpenCV 视频捕获组件
    cv::VideoCapture cap;
    if (videoSource == "0" || videoSource == "1" || videoSource == "2") {
        cap.open(std::stoi(videoSource));
    } else {
        cap.open(videoSource);
    }

    if (!cap.isOpened()) {
        std::cerr << "[错误] 无法打开指定的相机或视频文件: " << videoSource << std::endl;
        delete slamSys;
        return -1;
    }

    // 计算输入源的 FPS，用于帧率控制
    double inputFps = cap.get(cv::CAP_PROP_FPS);
    if (inputFps <= 0) inputFps = 30.0;
    bool isVideoFile = !(videoSource == "0" || videoSource == "1" || videoSource == "2");
    const double TARGET_FPS = 30.0;
    const int frameSkip = isVideoFile ? std::max(1, (int)std::round(inputFps / TARGET_FPS)) : 1;

    std::cout << "[Ubuntu GUI] 输入源 FPS: " << inputFps
              << ", 固定处理帧率: " << TARGET_FPS << " FPS"
              << (frameSkip > 1 ? " (跳帧率: " + std::to_string(frameSkip) + ")" : "")
              << std::endl;

    // 设置视频分辨率为 1280x720 以获得绝佳的高清画质
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

    cv::namedWindow("VtonaxAR SLAM Engine", cv::WINDOW_AUTOSIZE);
    initMenu();
    cv::setMouseCallback("VtonaxAR SLAM Engine", onMouse, nullptr);

    cv::Mat frame, imgGr, imgRgba;
    int fps = 0;
    auto lastTime = std::chrono::steady_clock::now();
    int frameCounter = 0;

    std::cout << "[Ubuntu GUI] 主处理线程循环已启动。按键盘 'ESC' 或 'q' 退出程序。" << std::endl;

    int skipCounter = 0;
    while (true) {
        cap >> frame;
        if (frame.empty()) {
            if (isVideoFile) {
                cap.set(cv::CAP_PROP_POS_FRAMES, 0);
                cap >> frame;          // 读取循环后的第一帧
                skipCounter = 0;       // 重置跳帧计数器
            }
            if (frame.empty()) break;
        }

        // 跳帧：仅当计数器达到 frameSkip 时才处理，其余帧丢弃
        skipCounter++;
        if (skipCounter < frameSkip) continue;
        skipCounter = 0;

        // 计算实时帧率 FPS
        frameCounter++;
        auto currentTime = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(currentTime - lastTime).count();
        if (duration >= 1.0) {
            fps = frameCounter;
            frameCounter = 0;
            lastTime = currentTime;
        }

        // 灰度化用于 SLAM 跟踪
        cv::cvtColor(frame, imgGr, cv::COLOR_BGR2GRAY);
        imgRgba = frame.clone(); // 保留彩色帧用于渲染

        // 缩放到 SLAM 工作分辨率 (640x360)
        cv::Mat imgSmall;
        cv::resize(imgGr, imgSmall,
                   cv::Size((int)ORB_SLAM2::BASE_SLAM_WIDTH,
                            (int)ORB_SLAM2::BASE_SLAM_HEIGHT));

        timeStamp += 1.0 / TARGET_FPS;
        int status = 0; // 初始状态为 NO_IMAGES_YET

        if (gEnableSLAM) {
            std::unique_lock<std::mutex> lock(gSlamStateMutex);
            if (slamSys) {
                Tcw = slamSys->TrackMonocular(imgSmall, timeStamp);
                status = slamSys->GetTrackingState();

                // 缓存跟踪的关键点和地图点数据以进行绘制
                std::lock_guard<std::mutex> lockPoints(gMapDataMutex);
                vMPs = slamSys->GetTrackedMapPoints();
                vKeys = slamSys->GetTrackedKeyPointsUn();
            }
        } else {
            status = 0;
            vMPs.clear();
            vKeys.clear();
            gShouldDrawArObject = false;
        }

        // 确定当前是否可以绘制 3D 虚拟物体
        {
            std::lock_guard<std::mutex> lock(gMapDataMutex);
            bool alignmentOK = true;
            if (pPlane && planeLoadedFromMap) {
                alignmentOK = slamSys->HasMapAlignment();
            }
            gShouldDrawArObject = (status == 2) && (pPlane != nullptr) && alignmentOK;
        }

        // 可视化显示点云（绿色表示已加载点，青色表示新建点）
        if (gEnablePointCloudDisplay) {
            if (status == 2 && slamSys->HasMapAlignment()) {
                cv::Mat TcwAligned = slamSys->GetMapAlignedPose(Tcw);
                std::vector<ORB_SLAM2::MapPoint*> allMapPoints = slamSys->GetAllMapPoints();
                drawAllMapPoints(TcwAligned, allMapPoints, imgRgba, fx, fy, cx, cy, true);
            }
            drawTrackedPoints(vKeys, vMPs, imgRgba);
        }

        // 若检测到平面，绘制 3D 虚拟 AR 立方体
        if (gShouldDrawArObject) {
            cv::Mat TcwAR = Tcw;
            if (slamSys->HasMapAlignment()) {
                TcwAR = slamSys->GetMapAlignedPose(Tcw);
            }
            drawARCube(imgRgba, TcwAR, pPlane, fx, fy, cx, cy);
        }

        // 绘制桌面交互式 UI 面板与状态仪表盘
        drawGUI(imgRgba, status, fps);

        cv::imshow("VtonaxAR SLAM Engine", imgRgba);

        // 键盘按键捕获
        char key = (char)cv::waitKey(33);  // ~30 FPS 显示刷新
        if (key == 27 || key == 'q') {
            break;
        }
    }

    // 释放资源
    std::cout << "[Ubuntu GUI] 正在关闭 SLAM 引擎并释放内存..." << std::endl;
    if (slamSys) {
        slamSys->Shutdown();
        delete slamSys;
    }
    if (pPlane) delete pPlane;

    std::cout << "[Ubuntu GUI] 程序正常退出，感谢使用！" << std::endl;
    return 0;
}

// 基于平面的原点和法向量投影并绘制 3D 交互式线框立方体
void drawARCube(cv::Mat& im, const cv::Mat& Tcw, Plane* plane, float fx, float fy, float cx, float cy) {
    if (Tcw.empty() || !plane) return;

    // 从平面局部坐标系转换到世界坐标系
    cv::Mat Twp = plane->Tpw.inv();

    // 根据平面测量的范围来自适应缩放立方体大小
    float s = 0.1f * plane->rang;
    if (s <= 0.01f) s = 0.05f;

    // 平面局部坐标系下的 8 个立方体顶点
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

        if (Zc <= 0.01f) return; // 跳过相机后方的裁剪点

        // 将相机坐标投影到图像上，显示画面为处理分辨率的 2 倍（对应 * 2.0f）
        float u = (fx * Xc / Zc + cx) * 2.0f;
        float v = (fy * Yc / Zc + cy) * 2.0f;
        ptsImg.push_back(cv::Point(u, v));
    }

    if (ptsImg.size() < 8) return;

    // 绘制平面底部轮廓 (黄色，高抗锯齿)
    cv::line(im, ptsImg[0], ptsImg[1], cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    cv::line(im, ptsImg[1], ptsImg[2], cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    cv::line(im, ptsImg[2], ptsImg[3], cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    cv::line(im, ptsImg[3], ptsImg[0], cv::Scalar(0, 255, 255), 2, cv::LINE_AA);

    // 绘制立方体顶部轮廓 (紫色，高抗锯齿)
    cv::line(im, ptsImg[4], ptsImg[5], cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
    cv::line(im, ptsImg[5], ptsImg[6], cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
    cv::line(im, ptsImg[6], ptsImg[7], cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
    cv::line(im, ptsImg[7], ptsImg[4], cv::Scalar(255, 0, 255), 2, cv::LINE_AA);

    // 绘制立方体垂直连接柱 (绿色，高抗锯齿)
    cv::line(im, ptsImg[0], ptsImg[4], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    cv::line(im, ptsImg[1], ptsImg[5], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    cv::line(im, ptsImg[2], ptsImg[6], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    cv::line(im, ptsImg[3], ptsImg[7], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
}

// 初始化可交互动作的按钮与菜单组
void initMenu() {
    menuSections.clear();

    // 第 1 组菜单: AR 互动操作
    MenuSection arSec;
    arSec.title = "AR 互动操作";
    arSec.expanded = true;

    Button btnPlace;
    btnPlace.label = "放置 AR 虚拟箱";
    btnPlace.color = cv::Scalar(30, 150, 20); // 翡翠绿
    btnPlace.action = []() {
        std::lock_guard<std::mutex> lock(gSlamStateMutex);
        std::lock_guard<std::mutex> lockData(gMapDataMutex);
        if (!Tcw.empty()) {
            cv::Mat TcwAligned = Tcw;
            if (slamSys && slamSys->HasMapAlignment()) {
                TcwAligned = slamSys->GetMapAlignedPose(Tcw);
            }
            if (pPlane) delete pPlane;
            pPlane = detectPlane(TcwAligned, vMPs, 50);
            if (pPlane) {
                if (slamSys && slamSys->MapChanged()) pPlane->Recompute();
                planeLoadedFromMap = false;
                std::cout << "[Ubuntu GUI] 成功检测到平面并生成虚拟 AR 场景！" << std::endl;
            } else {
                std::cout << "[Ubuntu GUI] 平面检测失败：特征点数量过少或未构成稳定平面。" << std::endl;
            }
        }
    };

    Button btnClear;
    btnClear.label = "清除所有物体";
    btnClear.color = cv::Scalar(30, 30, 200); // 胭脂红
    btnClear.action = []() {
        std::lock_guard<std::mutex> lock(gMapDataMutex);
        if (pPlane) {
            delete pPlane;
            pPlane = nullptr;
        }
        std::cout << "[Ubuntu GUI] 已清除屏幕上所有放置的 AR 虚拟物体。" << std::endl;
    };

    arSec.buttons.push_back(btnPlace);
    arSec.buttons.push_back(btnClear);

    // 第 2 组菜单: 地图持久化操作
    MenuSection mapSec;
    mapSec.title = "地图持久化操作";
    mapSec.expanded = false;

    Button btnSave;
    btnSave.label = "保存当前地图";
    btnSave.color = cv::Scalar(180, 100, 30); // 琥珀黄
    btnSave.action = []() {
        std::lock_guard<std::mutex> lock(gSlamStateMutex);
        if (slamSys) {
            std::cout << "[Ubuntu GUI] 正在将当前 SLAM 地图序列化输出至 vtonax_map.bin..." << std::endl;
            slamSys->SaveMap("vtonax_map.bin");
            if (pPlane) {
                std::string arFile = "vtonax_map.bin.arinfo";
                std::ofstream ofs(arFile, std::ios::binary);
                if (ofs.is_open()) {
                    const uint32_t magic = 0x4152494E; // 'ARIN'
                    const uint32_t version = 1;
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
            std::cout << "[Ubuntu GUI] 地图文件及 AR 平面数据保存成功！" << std::endl;
        }
    };

    Button btnLoad;
    btnLoad.label = "加载离线地图";
    btnLoad.color = cv::Scalar(30, 120, 180); // 晴空蓝
    btnLoad.action = []() {
        std::lock_guard<std::mutex> lock(gSlamStateMutex);
        if (slamSys) {
            std::cout << "[Ubuntu GUI] 正在从 vtonax_map.bin 文件中反序列化加载地图..." << std::endl;
            slamSys->LoadMap("vtonax_map.bin", 0, false);
            // 尝试读取关联的 AR 面元配置信息
            std::ifstream ifs("vtonax_map.bin.arinfo", std::ios::binary);
            if (ifs.is_open()) {
                uint32_t magic = 0, version = 0;
                ifs.read(reinterpret_cast<char*>(&magic), 4);
                ifs.read(reinterpret_cast<char*>(&version), 4);
                if (magic == 0x4152494E) {
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
            std::cout << "[Ubuntu GUI] 离线地图与全局环境面元加载完成！" << std::endl;
        }
    };

    mapSec.buttons.push_back(btnSave);
    mapSec.buttons.push_back(btnLoad);

    // 第 3 组菜单: SLAM 引擎控制
    MenuSection slamSec;
    slamSec.title = "SLAM 引擎控制";
    slamSec.expanded = false;

    Button btnReset;
    btnReset.label = "完全重置 SLAM";
    btnReset.color = cv::Scalar(180, 30, 180); // 紫罗兰
    btnReset.action = []() {
        std::lock_guard<std::mutex> lock(gSlamStateMutex);
        if (slamSys) {
            slamSys->Reset(false);
            if (pPlane) {
                delete pPlane;
                pPlane = nullptr;
            }
            std::cout << "[Ubuntu GUI] 已完全清空当前内存地图并重置 SLAM 跟踪回路！" << std::endl;
        }
    };

    Button btnToggleS;
    btnToggleS.label = "开启/关闭 SLAM";
    btnToggleS.color = cv::Scalar(150, 100, 30); // 橄榄绿
    btnToggleS.action = []() {
        gEnableSLAM = !gEnableSLAM;
        std::cout << "[Ubuntu GUI] SLAM 引擎控制状态切换: " << (gEnableSLAM ? "开启运行" : "暂停运行") << std::endl;
    };

    slamSec.buttons.push_back(btnReset);
    slamSec.buttons.push_back(btnToggleS);

    // 第 4 组菜单: 可视化显示设置
    MenuSection dispSec;
    dispSec.title = "可视化显示设置";
    dispSec.expanded = false;

    Button btnToggleP;
    btnToggleP.label = "切换点云显示";
    btnToggleP.color = cv::Scalar(100, 100, 100); // 经典灰
    btnToggleP.action = []() {
        gEnablePointCloudDisplay = !gEnablePointCloudDisplay;
    };

    dispSec.buttons.push_back(btnToggleP);

    menuSections.push_back(arSec);
    menuSections.push_back(mapSec);
    menuSections.push_back(slamSec);
    menuSections.push_back(dispSec);
}

// 绘制半透明状态仪表盘和右侧可收纳控制按钮板
void drawGUI(cv::Mat& frame, int trackingState, int fps) {
    // 1. 绘制左上方的半透明“状态仪表盘”
    cv::Mat overlayStatus = frame.clone();
    cv::rectangle(overlayStatus, cv::Rect(15, 15, 280, 140), cv::Scalar(30, 30, 30), -1);
    cv::addWeighted(overlayStatus, 0.75, frame, 0.25, 0, frame);

    // 绘制仪表盘的高雅描边边框
    cv::rectangle(frame, cv::Rect(15, 15, 280, 140), cv::Scalar(100, 100, 100), 1, cv::LINE_AA);

    // 跟踪状态字符映射
    std::string stateStr = "未开始图像输入";
    cv::Scalar stateColor(150, 150, 150);
    if (trackingState == 1) {
        stateStr = "未初始化 (请移动相机)";
        stateColor = cv::Scalar(30, 180, 230); // 橙黄
    } else if (trackingState == 2) {
        stateStr = "跟踪正常 (工作正常)";
        stateColor = cv::Scalar(30, 230, 30); // 绿色
    } else if (trackingState == 3) {
        stateStr = "跟踪丢失 (请缓慢对齐)";
        stateColor = cv::Scalar(30, 30, 230); // 红色
    }

    cv::putText(frame, "VtonaxAR 桌面状态面板", cv::Point(25, 38), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    cv::putText(frame, "---------------------------", cv::Point(25, 48), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100, 100, 100), 1, cv::LINE_AA);

    cv::putText(frame, "SLAM 状态: ", cv::Point(25, 68), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
    cv::putText(frame, stateStr, cv::Point(115, 68), cv::FONT_HERSHEY_SIMPLEX, 0.45, stateColor, 1, cv::LINE_AA);

    cv::putText(frame, "当前帧率: ", cv::Point(25, 88), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
    cv::putText(frame, std::to_string(fps) + " FPS", cv::Point(115, 88), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 217, 255), 1, cv::LINE_AA);

    int numKFs = slamSys ? slamSys->GetNumKeyFrames() : 0;
    int numMPs = slamSys ? slamSys->GetNumMapPoints() : 0;
    std::string statsStr = "关键帧: " + std::to_string(numKFs) + " | 地图点: " + std::to_string(numMPs);
    cv::putText(frame, "地图统计: ", cv::Point(25, 108), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
    cv::putText(frame, statsStr, cv::Point(115, 108), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 136), 1, cv::LINE_AA);

    bool alignment = slamSys ? slamSys->HasMapAlignment() : false;
    cv::putText(frame, "AR 状态: ", cv::Point(25, 128), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
    cv::putText(frame, (pPlane ? (alignment ? "已重定位平面" : "手动检测到平面") : "正在搜索可用平面..."),
                cv::Point(115, 128), cv::FONT_HERSHEY_SIMPLEX, 0.42, (pPlane ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 150, 255)), 1, cv::LINE_AA);

    // 2. 绘制右侧的“可滑动收纳按钮控制抽屉”
    int drawerW = 240;
    int drawerX = frame.cols - drawerW;
    cv::Mat overlayMenu = frame.clone();
    cv::rectangle(overlayMenu, cv::Rect(drawerX, 0, drawerW, frame.rows), cv::Scalar(15, 15, 15), -1);
    cv::addWeighted(overlayMenu, 0.82, frame, 0.18, 0, frame);

    // 绘制分隔边界线
    cv::line(frame, cv::Point(drawerX, 0), cv::Point(drawerX, frame.rows), cv::Scalar(60, 60, 60), 1, cv::LINE_AA);

    // 递归循环渲染每一个菜单栏组件和所包含的微调按钮
    int currentY = 20;
    int categoryHeight = 35;
    int buttonHeight = 30;
    int spacing = 6;

    for (size_t sIdx = 0; sIdx < menuSections.size(); sIdx++) {
        MenuSection& sec = menuSections[sIdx];
        sec.rect = cv::Rect(drawerX + 10, currentY, drawerW - 20, categoryHeight);

        bool isHovered = sec.rect.contains(mousePos);

        // 绘制折叠标题框的底色
        cv::Scalar catColor = isHovered ? cv::Scalar(75, 75, 75) : cv::Scalar(55, 55, 55);
        cv::rectangle(frame, sec.rect, catColor, -1);
        cv::rectangle(frame, sec.rect, cv::Scalar(90, 90, 90), 1, cv::LINE_AA);

        // 展开与收拢的状态符号
        std::string expandIcon = sec.expanded ? "[-] " : "[+] ";
        cv::putText(frame, expandIcon + sec.title, cv::Point(drawerX + 20, currentY + 22),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

        currentY += categoryHeight + spacing;

        if (sec.expanded) {
            for (size_t bIdx = 0; bIdx < sec.buttons.size(); bIdx++) {
                Button& btn = sec.buttons[bIdx];
                btn.rect = cv::Rect(drawerX + 20, currentY, drawerW - 40, buttonHeight);
                btn.isHovered = btn.rect.contains(mousePos);

                // 绝佳的鼠标移入高亮特效
                cv::Scalar borderCol = btn.isHovered ? cv::Scalar(255, 255, 255) : cv::Scalar(80, 80, 80);
                cv::Scalar fillCol = btn.color * (btn.isHovered ? 1.3 : 1.0);

                cv::rectangle(frame, btn.rect, fillCol, -1);
                cv::rectangle(frame, btn.rect, borderCol, 1, cv::LINE_AA);

                // 自动居中对齐渲染按钮文本
                int textBaseline = 0;
                cv::Size textSize = cv::getTextSize(btn.label, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &textBaseline);
                int textX = btn.rect.x + (btn.rect.width - textSize.width) / 2;
                int textY = btn.rect.y + (btn.rect.height + textSize.height) / 2 - 1;

                cv::putText(frame, btn.label, cv::Point(textX, textY),
                            cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

                currentY += buttonHeight + spacing;
            }
        }
        currentY += spacing;
    }
}

// 鼠标回调拦截与位置响应，保证卓越的交互灵活性
void onMouse(int event, int x, int y, int flags, void* userdata) {
    mousePos = cv::Point(x, y);

    if (event == cv::EVENT_LBUTTONDOWN) {
        for (auto& sec : menuSections) {
            if (sec.rect.contains(mousePos)) {
                sec.expanded = !sec.expanded;
                std::cout << "[Ubuntu GUI] 菜单抽屉板块 '" << sec.title << "' 收展状态已切换。" << std::endl;
                return;
            }

            if (sec.expanded) {
                for (auto& btn : sec.buttons) {
                    if (btn.rect.contains(mousePos)) {
                        std::cout << "[Ubuntu GUI] 触发按钮动作: '" << btn.label << "'" << std::endl;
                        btn.action();
                        return;
                    }
                }
            }
        }
    }
}
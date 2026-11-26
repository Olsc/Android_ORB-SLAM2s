// Created by Ads on 2017/1/6.
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

// 公共头文件：定义全局日志宏和通用包含文件

#ifndef ORB_SLAM2_AR_COMMON_H
#define ORB_SLAM2_AR_COMMON_H

#ifdef ANDROID
#include <android/log.h>

// Android日志标签
#define  LOG_TAG    "JNI_PART"

// 日志宏定义
#define LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG, __VA_ARGS__)   // 信息日志
#define LOGD(...)  __android_log_print(ANDROID_LOG_DEBUG,LOG_TAG, __VA_ARGS__)  // 调试日志
#define LOGW(...)  __android_log_print(ANDROID_LOG_WARN,LOG_TAG, __VA_ARGS__)   // 警告日志
#define LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG, __VA_ARGS__)  // 错误日志
#define LOGF(...)  __android_log_print(ANDROID_LOG_FATAL,LOG_TAG, __VA_ARGS__)  // 致命错误日志
#else
#include <stdio.h>
#define LOGI(...) do { printf("[INFO] "); printf(__VA_ARGS__); printf("\n"); } while(0)
#define LOGD(...) do { printf("[DEBUG] "); printf(__VA_ARGS__); printf("\n"); } while(0)
#define LOGW(...) do { printf("[WARN] "); printf(__VA_ARGS__); printf("\n"); } while(0)
#define LOGE(...) do { fprintf(stderr, "[ERROR] "); fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); } while(0)
#define LOGF(...) do { fprintf(stderr, "[FATAL] "); fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n"); } while(0)
#endif

#ifndef _WIN32
#include <unistd.h>
#endif

#if defined(__ANDROID__) || defined(__linux__)
#include <sys/resource.h>
#include <sys/syscall.h>
// 设置当前线程的 nice 值：Linux 新线程继承创建者的 nice，工作线程需显式设定，不沿用帧处理线程的高优先级
inline void SetCurrentThreadNice(int nice) { setpriority(PRIO_PROCESS, (id_t)syscall(SYS_gettid), nice); }
#else
inline void SetCurrentThreadNice(int) {}
#endif

// 工作线程统一为普通优先级：低于帧处理线程，彼此之间不分高低
// 后台重定位降到 10 时，负载下对齐加载地图会推迟数百帧
const int SLAM_NICE_WORKER = 0;

#endif // ORB_SLAM2_AR_COMMON_H
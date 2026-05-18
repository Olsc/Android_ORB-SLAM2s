//
// Created by Ads on 2017/1/6.
//

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

/**
 * 公共头文件
 * 功能描述: 定义全局日志宏和通用包含文件
 */

#ifndef ORB_SLAM2_AR_COMMON_H
#define ORB_SLAM2_AR_COMMON_H

#ifdef ANDROID
#include <android/log.h>

// Android日志标签
#define  LOG_TAG    "JNI_PART"

// 日志宏定义（按重要性从低到高）
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

#include <unistd.h>

// 启用函数跟踪
#define ENABLE_FUNCTION_TRACE
#include "DebugUtils.h"

// OpenCV 3/4 桌面向后兼容头文件 (解决 CvMat 和 CV_BGRA2GRAY 等遗留符号报错问题)
#include <opencv2/core/core_c.h>
#include <opencv2/core/types_c.h>
#include <opencv2/imgproc/imgproc_c.h>

#endif //ORB_SLAM2_AR_COMMON_H

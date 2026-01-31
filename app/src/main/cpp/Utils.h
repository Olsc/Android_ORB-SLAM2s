/**
 * Created by Ads on 2017/1/15.
 * 由Olsc于2025/8/25开始进行修改
 */

/**
 * 功能描述: 提供时间测量和性能分析相关的工具函数
 */

#ifndef UTILS_H
#define UTILS_H

#include "Common.h"

/**
 * 获取当前系统时间（毫秒）
 * @return 当前时间戳（毫秒）
 */
long getCurrentTime();

/**
 * 记录当前时间点，用于后续计算时间差
 */
void recordTime();

/**
 * 输出从recordTime()调用至今经过的时间
 */
// void logTime();

#endif //ORB_SLAM2_AR_PLANE_H

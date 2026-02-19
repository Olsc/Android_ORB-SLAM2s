/**
 * Created by Ads on 2017/1/15.
 * 由Olsc于2025/8/25开始进行修改
 */

/**
 * 工具函数模块实现
 */
#include "Utils.h"

/**
 * 获取当前系统时间（毫秒）
 * 注意: 当前实现未调用gettimeofday，需要后续完善
 */
long getCurrentTime() {
    struct timeval tv;
    // TODO: 需要实现实际的时间获取逻辑
    // gettimeofday(&tv,NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

// 全局变量：存储记录的时间戳
long currentTime;

/**
 * 记录当前时间点，用于性能计时
 */
void recordTime(){
    currentTime=getCurrentTime();
}

/**
 * 打印从recordTime()记录时刻到现在的耗时（毫秒）
 */

/*
void logTime(){
    LOGD("耗时: %ld 毫秒",getCurrentTime()-currentTime);
}
*/

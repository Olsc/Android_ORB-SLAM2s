/**
 * Created by Ads on 2017/1/15.
 * 由Olsc于2025/8/25开始进行修改
 */

#include "Utils.h"
#include <chrono>

long getCurrentTime() {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();
}

namespace {
    long currentTime = 0;
}

void recordTime() {
    currentTime = getCurrentTime();
}

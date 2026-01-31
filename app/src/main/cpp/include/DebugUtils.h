#ifndef DEBUG_UTILS_H
#define DEBUG_UTILS_H

#include <android/log.h>
#include <string>
#include <chrono>

// 使用在此定义的LOG_TAG，或者如果未定义则使用默认的
#ifndef LOG_TAG
#define LOG_TAG "NativeDebug"
#endif

// 简单的Scoped Trace类
// 仅在执行时间超过阈值（如50ms）时打印，避免日志泛滥
class ScopedTrace {
public:
    ScopedTrace(const char* funcName) : mName(funcName) {
        // 记录开始时间
        mStart = std::chrono::steady_clock::now();
    }
    
    ~ScopedTrace() {
        auto end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - mStart).count();

        // 极端情况：耗时超过50ms才打印
        if (duration > 50) {
            __android_log_print(ANDROID_LOG_WARN, LOG_TAG, "Extreme Case - Slow Function: %s took %lld ms", mName, (long long)duration);
        }
    }

private:
    const char* mName;
    std::chrono::steady_clock::time_point mStart;
};

// 宏定义
// 使用方法：在函数开头添加 FUNCTION_TRACE;
#ifdef ENABLE_FUNCTION_TRACE
    #define FUNCTION_TRACE ScopedTrace __trace__(__FUNCTION__)
#else
    #define FUNCTION_TRACE
#endif

#endif // DEBUG_UTILS_H

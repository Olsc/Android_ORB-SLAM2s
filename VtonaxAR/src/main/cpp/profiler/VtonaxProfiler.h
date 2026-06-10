#ifndef VTONAX_PROFILER_H
#define VTONAX_PROFILER_H

#include <string>
#include <chrono>
#include <thread>
#include <atomic>
#include <vector>

// 开发模式开关
#ifdef VTONAX_DEVELOP_MODE

namespace Vtonax {

enum class EventType : uint8_t {
    Begin = 0, // 开始
    End = 1    // 结束
};

class Profiler {
public:
    static Profiler& Get();

    void Initialize(const std::string& outputFile);
    void Shutdown();

    void WriteEvent(const char* name, EventType type);

    static uint64_t GetTimestampNS() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch()
        ).count();
    }

private:
    Profiler() = default;
    ~Profiler();

    // 内部实现细节
    struct Impl;
    Impl* pImpl = nullptr;
};

class Timer {
public:
    Timer(const char* name) : m_Name(name) {
        Profiler::Get().WriteEvent(m_Name, EventType::Begin);
    }
    ~Timer() {
        Profiler::Get().WriteEvent(m_Name, EventType::End);
    }
private:
    const char* m_Name;
};

} // namespace Vtonax

// 分析器宏定义
#define VT_PROFILE_INITIALIZE(path) Vtonax::Profiler::Get().Initialize(path)
#define VT_PROFILE_SHUTDOWN() Vtonax::Profiler::Get().Shutdown()
#define VT_PROFILE_BEGIN(name) Vtonax::Profiler::Get().WriteEvent(name, Vtonax::EventType::Begin)
#define VT_PROFILE_END(name) Vtonax::Profiler::Get().WriteEvent(name, Vtonax::EventType::End)
#define VT_CONCAT_IMPL(a, b) a##b
#define VT_CONCAT(a, b) VT_CONCAT_IMPL(a, b)
#define VT_PROFILE_FUNCTION() Vtonax::Timer VT_CONCAT(_timer_, __LINE__)(__PRETTY_FUNCTION__)
#define VT_PROFILE_SCOPE(name) Vtonax::Timer VT_CONCAT(_timer_, __LINE__)(name)

#else

// 非开发模式下宏定义为空
#define VT_PROFILE_INITIALIZE(path)
#define VT_PROFILE_SHUTDOWN()
#define VT_PROFILE_BEGIN(name)
#define VT_PROFILE_END(name)
#define VT_PROFILE_FUNCTION()
#define VT_PROFILE_SCOPE(name)

#endif // VTONAX_DEVELOP_MODE

#endif // VTONAX_PROFILER_H

/**
 * Copyright (C) 2025-2026 Olsc <OlscStudio@outlook.com>
 *
 * This file is part of the Android ORB-SLAM2s project (a fork of ORB-SLAM2).
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef MENTHA_PROFILER_H
#define MENTHA_PROFILER_H

#include <string>
#include <chrono>
#include <thread>
#include <atomic>
#include <vector>

// 开发模式开关
#ifdef MENTHA_DEVELOP_MODE

namespace Mentha {

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

} // namespace Mentha

// 分析器宏定义
#define VT_PROFILE_INITIALIZE(path) Mentha::Profiler::Get().Initialize(path)
#define VT_PROFILE_SHUTDOWN() Mentha::Profiler::Get().Shutdown()
#define VT_PROFILE_BEGIN(name) Mentha::Profiler::Get().WriteEvent(name, Mentha::EventType::Begin)
#define VT_PROFILE_END(name) Mentha::Profiler::Get().WriteEvent(name, Mentha::EventType::End)
#define VT_CONCAT_IMPL(a, b) a##b
#define VT_CONCAT(a, b) VT_CONCAT_IMPL(a, b)
#define VT_PROFILE_FUNCTION() Mentha::Timer VT_CONCAT(_timer_, __LINE__)(__PRETTY_FUNCTION__)
#define VT_PROFILE_SCOPE(name) Mentha::Timer VT_CONCAT(_timer_, __LINE__)(name)

#else

// 非开发模式下宏定义为空语句，避免产生悬空分号（EmptyDeclOrStmt 警告）
#define VT_PROFILE_INITIALIZE(path) ((void)0)
#define VT_PROFILE_SHUTDOWN() ((void)0)
#define VT_PROFILE_BEGIN(name) ((void)0)
#define VT_PROFILE_END(name) ((void)0)
#define VT_PROFILE_FUNCTION() ((void)0)
#define VT_PROFILE_SCOPE(name) ((void)0)

#endif // MENTHA_DEVELOP_MODE

#endif // MENTHA_PROFILER_H
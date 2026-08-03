#include "MenthaProfiler.h"
#include <fstream>
#include <mutex>
#include <unordered_map>
#include <iostream>
#include <thread>
#include <queue>
#include <condition_variable>

#ifdef MENTHA_DEVELOP_MODE

namespace Mentha {

#pragma pack(push, 1)
// 事件记录结构体
struct EventRecord {
    uint32_t nameId;    // 函数名称ID
    uint32_t threadId;  // 线程ID
    uint64_t timestamp; // 纳秒时间戳
    EventType type;     // 事件类型 (开始/结束)
};
#pragma pack(pop)

struct Profiler::Impl {
    std::ofstream outFile; // 输出文件流
    std::mutex mutex;      // 互斥锁，保护队列和映射表
    std::unordered_map<std::string, uint32_t> nameMap; // 函数名与ID的映射
    uint32_t nextNameId = 0;

    std::atomic<bool> running{false}; // 运行标志
    std::thread writerThread;         // 后台写入线程
    std::queue<EventRecord> eventQueue; // 事件缓冲区队列
    std::condition_variable cv;       // 条件变量，用于同步写入线程

    // 后台写入循环
    void WriterLoop() {
        while (running || !eventQueue.empty()) {
            std::vector<EventRecord> batch;
            {
                std::unique_lock<std::mutex> lock(mutex);
                cv.wait_for(lock, std::chrono::milliseconds(100), [this] {
                    return !eventQueue.empty() || !running;
                });

                // 批量提取事件记录，减少IO调用次数
                while (!eventQueue.empty() && batch.size() < 1000) {
                    batch.push_back(eventQueue.front());
                    eventQueue.pop();
                }
            }

            if (!batch.empty()) {
                for (const auto& event : batch) {
                    uint8_t eventMarker = 0xEE; // 事件标记
                    outFile.write(reinterpret_cast<const char*>(&eventMarker), 1);
                    outFile.write(reinterpret_cast<const char*>(&event), sizeof(EventRecord));
                }
                outFile.flush(); // 实时保存到磁盘，防止崩溃丢失
            }
        }
    }
};

Profiler& Profiler::Get() {
    static Profiler instance;
    return instance;
}

void Profiler::Initialize(const std::string& outputFile) {
    if (pImpl) return;
    pImpl = new Impl();
    pImpl->outFile.open(outputFile, std::ios::binary);

    // 写入文件头：幻数 'VPRO'，版本号 1
    uint32_t magic = 0x4F525056; // 'VPRO'
    uint32_t version = 1;
    pImpl->outFile.write(reinterpret_cast<const char*>(&magic), 4);
    pImpl->outFile.write(reinterpret_cast<const char*>(&version), 4);

    pImpl->running = true;
    pImpl->writerThread = std::thread(&Impl::WriterLoop, pImpl);
}

void Profiler::Shutdown() {
    if (!pImpl) return;
    {
        std::lock_guard<std::mutex> lock(pImpl->mutex);
        pImpl->running = false;
    }
    pImpl->cv.notify_all();
    if (pImpl->writerThread.joinable()) {
        pImpl->writerThread.join();
    }

    pImpl->outFile.close();
    delete pImpl;
    pImpl = nullptr;
}

void Profiler::WriteEvent(const char* name, EventType type) {
    if (!pImpl || !pImpl->running) return;

    // 获取当前线程ID (使用 hash 确保唯一性)
    static thread_local uint32_t tid = static_cast<uint32_t>(std::hash<std::thread::id>{}(std::this_thread::get_id()));

    uint32_t nameId;
    {
        std::lock_guard<std::mutex> lock(pImpl->mutex);
        auto it = pImpl->nameMap.find(name);
        if (it == pImpl->nameMap.end()) {
            nameId = pImpl->nextNameId++;
            pImpl->nameMap[name] = nameId;

            // 立即将字符串映射写入文件，确保崩溃后仍可解析
            uint8_t mapMarker = 0xFF; // 名称映射记录标记
            pImpl->outFile.write(reinterpret_cast<const char*>(&mapMarker), 1);
            pImpl->outFile.write(reinterpret_cast<const char*>(&nameId), 4);
            uint32_t len = strlen(name);
            pImpl->outFile.write(reinterpret_cast<const char*>(&len), 4);
            pImpl->outFile.write(name, len);
        } else {
            nameId = it->second;
        }

        pImpl->eventQueue.push({nameId, tid, GetTimestampNS(), type});
    }
    pImpl->cv.notify_one();
}

Profiler::~Profiler() {
    Shutdown();
}

} // namespace Mentha

#endif // MENTHA_DEVELOP_MODE

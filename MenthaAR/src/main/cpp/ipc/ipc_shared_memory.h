#ifndef IPC_SHARED_MEMORY_H
#define IPC_SHARED_MEMORY_H

#include <sys/mman.h>
#include <unistd.h>
#include <android/log.h>
#include <opencv2/core/core.hpp>

#define LOG_TAG_IPC "C_IPC_SharedMemory"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG_IPC, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG_IPC, __VA_ARGS__)

namespace ipc {
    class SharedMemoryReader {
    public:
        static bool processSharedMemoryFd(int fd, int size, int width, int height, cv::Mat& outGrayMat, cv::Mat& outRgbaMat);
    };
}

#endif // IPC_SHARED_MEMORY_H

#include "ipc_shared_memory.h"
#include <opencv2/imgproc/imgproc.hpp>

namespace ipc {

bool SharedMemoryReader::processSharedMemoryFd(int fd, int size, int width, int height, cv::Mat& outGrayMat, cv::Mat& outRgbaMat) {
    if (fd < 0 || size <= 0 || width <= 0 || height <= 0) {
        LOGE("无效的共享内存参数: fd=%d, size=%d, w=%d, h=%d", fd, size, width, height);
        return false;
    }

    void* mappedPtr = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (mappedPtr == MAP_FAILED) {
        LOGE("mmap 映射内存失败");
        return false;
    }

    int expectedYSize = width * height;
    if (size >= expectedYSize * 4) {
        // 直接在共享内存上包装 RGBA Mat，以便 C++ 绘制的点云与特征点能无缝写入共享内存供主界面渲染
        outRgbaMat = cv::Mat(height, width, CV_8UC4, mappedPtr);
        cv::cvtColor(outRgbaMat, outGrayMat, cv::COLOR_RGBA2GRAY);
    } else if (size >= expectedYSize) {
        outGrayMat = cv::Mat(height, width, CV_8UC1, mappedPtr).clone();
        cv::cvtColor(outGrayMat, outRgbaMat, cv::COLOR_GRAY2RGBA);
    } else {
        LOGE("共享内存尺寸不匹配: size=%d, expectedY=%d", size, expectedYSize);
        munmap(mappedPtr, size);
        return false;
    }

    munmap(mappedPtr, size);
    return true;
}

} // namespace ipc

#include "OpenCVBridge.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <android/bitmap.h>
#include <android/log.h>

#define TAG "OpenCVBridge"
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, TAG, __VA_ARGS__)

// ==================== Mat 生命周期 ====================

JNIEXPORT jlong JNICALL Java_com_orb_slam2s_slamar_OpenCVBridge_nativeCreateMat
  (JNIEnv *env, jclass cls, jint rows, jint cols, jint type)
{
    cv::Mat* mat = new cv::Mat(rows, cols, type);
    return (jlong)mat;
}

JNIEXPORT void JNICALL Java_com_orb_slam2s_slamar_OpenCVBridge_nativeReleaseMat
  (JNIEnv *env, jclass cls, jlong matAddr)
{
    cv::Mat* mat = (cv::Mat*)matAddr;
    if (mat) {
        mat->release();
        delete mat;
    }
}

// ==================== Mat 数据操作 ====================

JNIEXPORT void JNICALL Java_com_orb_slam2s_slamar_OpenCVBridge_nativePutData
  (JNIEnv *env, jclass cls, jlong matAddr, jbyteArray data)
{
    cv::Mat* mat = (cv::Mat*)matAddr;
    if (!mat || mat->empty()) return;

    jbyte* dataPtr = env->GetByteArrayElements(data, NULL);
    jsize dataLen = env->GetArrayLength(data);

    size_t matSize = mat->rows * mat->step;
    size_t copyLen = (size_t)dataLen < matSize ? (size_t)dataLen : matSize;
    memcpy(mat->data, dataPtr, copyLen);

    env->ReleaseByteArrayElements(data, dataPtr, JNI_ABORT);
}

// 直接从 ByteBuffer 写入 Mat，避免 Java byte[] 中间拷贝
JNIEXPORT void JNICALL Java_com_orb_slam2s_slamar_OpenCVBridge_nativePutBuffer
  (JNIEnv *env, jclass cls, jlong matAddr, jobject buffer)
{
    cv::Mat* mat = (cv::Mat*)matAddr;
    if (!mat || mat->empty() || !buffer) return;

    void* srcPtr = env->GetDirectBufferAddress(buffer);
    if (!srcPtr) {
        // Fallback: 非 DirectBuffer 时通过 ByteBuffer API 读取
        jclass bufClass = env->GetObjectClass(buffer);
        jmethodID arrMethod = env->GetMethodID(bufClass, "array", "()[B");
        if (!arrMethod) return;
        jbyteArray byteArr = (jbyteArray)env->CallObjectMethod(buffer, arrMethod);
        if (!byteArr) return;
        Java_com_orb_slam2s_slamar_OpenCVBridge_nativePutData(env, cls, matAddr, byteArr);
        return;
    }

    jlong bufCapacity = env->GetDirectBufferCapacity(buffer);
    size_t matSize = mat->rows * mat->step;
    size_t copyLen = (size_t)bufCapacity < matSize ? (size_t)bufCapacity : matSize;
    memcpy(mat->data, srcPtr, copyLen);
}

JNIEXPORT void JNICALL Java_com_orb_slam2s_slamar_OpenCVBridge_nativeMatSetTo
  (JNIEnv *env, jclass cls, jlong matAddr, jdouble v1, jdouble v2, jdouble v3, jdouble v4)
{
    cv::Mat* mat = (cv::Mat*)matAddr;
    if (!mat || mat->empty()) return;
    *mat = cv::Scalar(v1, v2, v3, v4);
}

// ==================== 图像处理 ====================

JNIEXPORT void JNICALL Java_com_orb_slam2s_slamar_OpenCVBridge_nativeRGBA2Gray
  (JNIEnv *env, jclass cls, jlong srcAddr, jlong dstAddr)
{
    cv::Mat* src = (cv::Mat*)srcAddr;
    cv::Mat* dst = (cv::Mat*)dstAddr;
    if (!src || src->empty() || !dst) return;
    if (src->channels() == 1) {
        // 已经是灰度图，直接拷贝
        src->copyTo(*dst);
    } else {
        cv::cvtColor(*src, *dst, cv::COLOR_RGBA2GRAY);
    }
}

// ==================== YUV420 处理 ====================

JNIEXPORT void JNICALL Java_com_orb_slam2s_slamar_OpenCVBridge_nativeYPlaneToMats
  (JNIEnv *env, jclass cls, jlong rgbaMatAddr, jlong grayMatAddr,
   jbyteArray yData, jint width, jint height)
{
    cv::Mat* rgbaMat = (cv::Mat*)rgbaMatAddr;
    cv::Mat* grayMat = (cv::Mat*)grayMatAddr;
    if (!rgbaMat || !grayMat) return;
    if (rgbaMat->empty() || grayMat->empty()) return;

    jbyte* data = env->GetByteArrayElements(yData, nullptr);
    if (!data) return;

    const uint8_t* src = (const uint8_t*)data;
    size_t pixelCount = (size_t)width * height;

    // 灰度图：直接拷贝
    memcpy(grayMat->data, src, pixelCount);

    // RGBA: 将 Y 值填入所有 4 通道 (R=G=B=Y, A=255)
    uint8_t* dst = (uint8_t*)rgbaMat->data;
    for (size_t i = 0; i < pixelCount; i++) {
        dst[i * 4]     = src[i];  // R
        dst[i * 4 + 1] = src[i];  // G
        dst[i * 4 + 2] = src[i];  // B
        dst[i * 4 + 3] = 255;     // A
    }

    env->ReleaseByteArrayElements(yData, data, JNI_ABORT);
}

JNIEXPORT void JNICALL Java_com_orb_slam2s_slamar_OpenCVBridge_nativeRotate180
  (JNIEnv *env, jclass cls, jlong matAddr)
{
    cv::Mat* mat = (cv::Mat*)matAddr;
    if (!mat || mat->empty()) return;
    cv::rotate(*mat, *mat, cv::ROTATE_180);
}

JNIEXPORT void JNICALL Java_com_orb_slam2s_slamar_OpenCVBridge_nativeFlipBoth
  (JNIEnv *env, jclass cls, jlong matAddr)
{
    cv::Mat* mat = (cv::Mat*)matAddr;
    if (!mat || mat->empty()) return;
    cv::flip(*mat, *mat, -1);  // -1 = both axes
}

// ==================== Bitmap 转换 ====================

JNIEXPORT void JNICALL Java_com_orb_slam2s_slamar_OpenCVBridge_nativeMatToBitmap
  (JNIEnv *env, jclass cls, jlong matAddr, jobject bitmap)
{
    cv::Mat* mat = (cv::Mat*)matAddr;
    if (!mat || mat->empty() || !bitmap) return;

    AndroidBitmapInfo info;
    if (AndroidBitmap_getInfo(env, bitmap, &info) != ANDROID_BITMAP_RESULT_SUCCESS) {
        LOGE("nativeMatToBitmap: AndroidBitmap_getInfo failed");
        return;
    }

    if (info.format != ANDROID_BITMAP_FORMAT_RGBA_8888 &&
        info.format != ANDROID_BITMAP_FORMAT_RGB_565) {
        LOGE("nativeMatToBitmap: unsupported bitmap format");
        return;
    }

    void* pixels = NULL;
    if (AndroidBitmap_lockPixels(env, bitmap, &pixels) != ANDROID_BITMAP_RESULT_SUCCESS) {
        LOGE("nativeMatToBitmap: AndroidBitmap_lockPixels failed");
        return;
    }

    if (info.format == ANDROID_BITMAP_FORMAT_RGBA_8888) {
        cv::Mat bmp(info.height, info.width, CV_8UC4, pixels, info.stride);
        if (mat->channels() == 4) {
            mat->copyTo(bmp);
        } else if (mat->channels() == 1) {
            cv::cvtColor(*mat, bmp, cv::COLOR_GRAY2RGBA);
        }
    } else {
        // RGB_565
        cv::Mat bmp(info.height, info.width, CV_8UC2, pixels, info.stride);
        cv::Mat temp;
        if (mat->channels() == 4) {
            cv::cvtColor(*mat, temp, cv::COLOR_RGBA2RGB);
        } else if (mat->channels() == 1) {
            cv::cvtColor(*mat, temp, cv::COLOR_GRAY2RGB);
        } else {
            mat->copyTo(temp);
        }
        cv::Mat temp565(info.height, info.width, CV_8UC2);
        cv::cvtColor(temp, temp565, cv::COLOR_RGB2BGR565);
        temp565.copyTo(bmp);
    }

    AndroidBitmap_unlockPixels(env, bitmap);
}

JNIEXPORT void JNICALL Java_com_orb_slam2s_slamar_OpenCVBridge_nativeBitmapToMat
  (JNIEnv *env, jclass cls, jobject bitmap, jlong matAddr)
{
    cv::Mat* mat = (cv::Mat*)matAddr;
    if (!mat || !bitmap) return;

    AndroidBitmapInfo info;
    if (AndroidBitmap_getInfo(env, bitmap, &info) != ANDROID_BITMAP_RESULT_SUCCESS) {
        LOGE("nativeBitmapToMat: AndroidBitmap_getInfo failed");
        return;
    }

    void* pixels = NULL;
    if (AndroidBitmap_lockPixels(env, bitmap, &pixels) != ANDROID_BITMAP_RESULT_SUCCESS) {
        LOGE("nativeBitmapToMat: AndroidBitmap_lockPixels failed");
        return;
    }

    if (info.format == ANDROID_BITMAP_FORMAT_RGBA_8888) {
        cv::Mat bmp(info.height, info.width, CV_8UC4, pixels, info.stride);
        bmp.copyTo(*mat);
    } else {
        LOGE("nativeBitmapToMat: unsupported format (only RGBA_8888)");
    }

    AndroidBitmap_unlockPixels(env, bitmap);
}

// ==================== 时间测量 ====================

JNIEXPORT jdouble JNICALL Java_com_orb_slam2s_slamar_OpenCVBridge_nativeGetTickFrequency
  (JNIEnv *env, jclass cls)
{
    return cv::getTickFrequency();
}

JNIEXPORT jlong JNICALL Java_com_orb_slam2s_slamar_OpenCVBridge_nativeGetTickCount
  (JNIEnv *env, jclass cls)
{
    return cv::getTickCount();
}

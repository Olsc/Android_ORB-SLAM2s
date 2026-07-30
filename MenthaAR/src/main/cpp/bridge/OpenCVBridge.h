#ifndef OPENCV_BRIDGE_H
#define OPENCV_BRIDGE_H

#include <jni.h>

#ifdef __cplusplus
extern "C" {
#endif

// Mat 生命周期
JNIEXPORT jlong JNICALL Java_com_orb_slam2s_slamar_OpenCVBridge_nativeCreateMat
  (JNIEnv *, jclass, jint, jint, jint);

JNIEXPORT void JNICALL Java_com_orb_slam2s_slamar_OpenCVBridge_nativeReleaseMat
  (JNIEnv *, jclass, jlong);

// Mat 数据操作
JNIEXPORT void JNICALL Java_com_orb_slam2s_slamar_OpenCVBridge_nativePutData
  (JNIEnv *, jclass, jlong, jbyteArray);

JNIEXPORT void JNICALL Java_com_orb_slam2s_slamar_OpenCVBridge_nativePutBuffer
  (JNIEnv *, jclass, jlong, jobject);

JNIEXPORT void JNICALL Java_com_orb_slam2s_slamar_OpenCVBridge_nativeMatSetTo
  (JNIEnv *, jclass, jlong, jdouble, jdouble, jdouble, jdouble);

// 图像处理
JNIEXPORT void JNICALL Java_com_orb_slam2s_slamar_OpenCVBridge_nativeRGBA2Gray
  (JNIEnv *, jclass, jlong, jlong);

JNIEXPORT void JNICALL Java_com_orb_slam2s_slamar_OpenCVBridge_nativeRotate180
  (JNIEnv *, jclass, jlong);

JNIEXPORT void JNICALL Java_com_orb_slam2s_slamar_OpenCVBridge_nativeFlipBoth
  (JNIEnv *, jclass, jlong);

// Bitmap 转换 (需要 Android Bitmap API via JNI)
JNIEXPORT void JNICALL Java_com_orb_slam2s_slamar_OpenCVBridge_nativeMatToBitmap
  (JNIEnv *, jclass, jlong, jobject);

JNIEXPORT void JNICALL Java_com_orb_slam2s_slamar_OpenCVBridge_nativeBitmapToMat
  (JNIEnv *, jclass, jobject, jlong);

// 时间测量
JNIEXPORT jdouble JNICALL Java_com_orb_slam2s_slamar_OpenCVBridge_nativeGetTickFrequency
  (JNIEnv *, jclass);

JNIEXPORT jlong JNICALL Java_com_orb_slam2s_slamar_OpenCVBridge_nativeGetTickCount
  (JNIEnv *, jclass);

#ifdef __cplusplus
}
#endif

#endif // OPENCV_BRIDGE_H

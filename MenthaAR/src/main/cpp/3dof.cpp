#include <jni.h>
#include <math.h>
#include <string.h>
#include <android/log.h>
#include "Matrix.h"
#include "include/Config.h"

#ifndef LOG_TAG
#define LOG_TAG "3DOF"
#endif

#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

extern "C" {

// 3DOF坐标系重映射（Java层已处理横屏重映射，这里直接使用）
static void getRemappedMatrix_3dof(float* outMatrix, float* inMatrix, int rotation) {
    // 传感器坐标系已在Java层通过SensorManager.remapCoordinateSystem重映射为横屏坐标系
    // 这里直接复制即可
    memcpy(outMatrix, inMatrix, 16 * sizeof(float));
}

// 计算3DOF物体插入点（在相机前方指定距离处）
JNIEXPORT jfloatArray JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_calculate3DofInsertionPoint(JNIEnv* env, jobject thiz, jfloatArray rotationMatrix, jint rotation, jfloat distance) {
    jsize len = env->GetArrayLength(rotationMatrix);
    float deviceRotation[16];

    if (len >= 16) {
        env->GetFloatArrayRegion(rotationMatrix, 0, 16, deviceRotation);
    } else {
        setIdentityM(deviceRotation);
    }

    // 获取根据屏幕旋转调整后的视图矩阵
    float viewMatrix[16];
    getRemappedMatrix_3dof(viewMatrix, deviceRotation, rotation);

    // 计算方块在世界空间中的坐标: Inv(View) * (0, 0, -distance)
    float invViewMatrix[16];
    transposeM(invViewMatrix, 0, viewMatrix, 0);

    float targetPosCameraSpace[4] = { 0.0f, 0.0f, -distance, 1.0f };
    float worldPos[4];
    multiplyMV(worldPos, 0, invViewMatrix, 0, targetPosCameraSpace, 0);

    jfloatArray result = env->NewFloatArray(3);
    float resFloats[3] = {worldPos[0], worldPos[1], worldPos[2]};
    env->SetFloatArrayRegion(result, 0, 3, resFloats);

    LOGD("3DOF插入点计算完成: [%.2f, %.2f, %.2f]", worldPos[0], worldPos[1], worldPos[2]);
    return result;
}

// 计算3DOF MVP矩阵
JNIEXPORT jfloatArray JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_compute3DofMVP(JNIEnv* env, jobject thiz, jfloatArray rotationMatrix, jint rotation, jfloat ratio, jfloatArray objectPos) {
    // 获取设备旋转数据
    jsize len = env->GetArrayLength(rotationMatrix);
    float deviceRotation[16];
    if (len >= 16) {
        env->GetFloatArrayRegion(rotationMatrix, 0, 16, deviceRotation);
    } else {
        setIdentityM(deviceRotation);
    }

    // 获取物体世界坐标
    float worldPos[3] = {0, 0, 0};
    if (objectPos != NULL) {
        env->GetFloatArrayRegion(objectPos, 0, 3, worldPos);
    }

    // 计算视图矩阵 (根据屏幕旋转重映射)
    float viewMatrix[16];
    getRemappedMatrix_3dof(viewMatrix, deviceRotation, rotation);

    // 计算投影矩阵
    float projectionMatrix[16];
    frustumM(projectionMatrix, 0, -ratio, ratio, -1, 1, ORB_SLAM2::AR_3DOF_ZNEAR, ORB_SLAM2::AR_3DOF_ZFAR);

    // 计算模型矩阵 (平移到指定的世界坐标并添加固定自转)
    float modelMatrix[16];
    setIdentityM(modelMatrix);
    translateM(modelMatrix, 0, worldPos[0], worldPos[1], worldPos[2]);
    rotateM(modelMatrix, modelMatrix, ORB_SLAM2::AR_OBJECT_SPIN_Y_DEG, 0.0f, 1.0f, 0.0f);
    rotateM(modelMatrix, modelMatrix, ORB_SLAM2::AR_OBJECT_TILT_X_DEG, 1.0f, 0.0f, 0.0f);

    // 组合变换: MVP = Projection * View * Model
    float tempMatrix[16];
    multiplyMM(tempMatrix, viewMatrix, modelMatrix);

    float mvpMatrix[16];
    multiplyMM(mvpMatrix, projectionMatrix, tempMatrix);

    jfloatArray result = env->NewFloatArray(16);
    env->SetFloatArrayRegion(result, 0, 16, mvpMatrix);
    return result;
}

}

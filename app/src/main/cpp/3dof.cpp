#include <jni.h>
#include <math.h>
#include <string.h>
#include <android/log.h>
#include "Matrix.h"

#define LOG_TAG "3DOF"
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)

extern "C" {

// 3DOF坐标系重映射（Java层已处理横屏重映射，这里直接使用）
static void getRemappedMatrix_3dof(float* outMatrix, float* inMatrix, int rotation) {
    // 传感器坐标系已在Java层通过SensorManager.remapCoordinateSystem重映射为横屏坐标系
    // 这里直接复制即可
    memcpy(outMatrix, inMatrix, 16 * sizeof(float));
}

// ========== OrientationSensor 计算功能 ==========

/**
 * 从旋转矩阵计算欧拉角（Yaw, Pitch, Roll）
 * 对应 Android SensorManager.getOrientation() 的功能
 * 
 * @param rotationMatrix 输入的旋转矩阵（3x3 或 4x4）
 * @param values 输出的欧拉角数组 [方位角, 俯仰角, 滚转角]（弧度）
 * 
 * 返回值说明：
 * - values[0]: 方位角（Azimuth）- 绕Z轴旋转，范围 [-π, π]
 * - values[1]: 俯仰角（Pitch）- 绕X轴旋转，范围 [-π/2, π/2]
 * - values[2]: 滚转角（Roll）- 绕Y轴旋转，范围 [-π, π]
 */
static void getOrientationFromMatrix_3dof(const float* rotationMatrix, float* values) {
    // 假设输入是4x4矩阵，取左上角3x3部分
    // 如果是3x3矩阵，stride应该是3
    
    // 从旋转矩阵提取欧拉角
    // 遵循Android的ZXY欧拉角约定
    
    // 方位角（Azimuth）: atan2(R[1][0], R[0][0])
    values[0] = atan2f(rotationMatrix[1 * 4 + 0], rotationMatrix[0 * 4 + 0]);
    
    // 俯仰角（Pitch）: asin(-R[2][0])
    float pitchSin = -rotationMatrix[2 * 4 + 0];
    // 限制在[-1, 1]范围内以避免asin域错误
    pitchSin = fmaxf(-1.0f, fminf(1.0f, pitchSin));
    values[1] = asinf(pitchSin);
    
    // 滚转角（Roll）: atan2(-R[2][1], R[2][2])
    values[2] = atan2f(-rotationMatrix[2 * 4 + 1], rotationMatrix[2 * 4 + 2]);
}

/**
 * 坐标系重映射（用于处理不同的设备方向）
 * 对应 Android SensorManager.remapCoordinateSystem()
 * 
 * @param inR 输入旋转矩阵
 * @param X 新的X轴对应的原轴（例如 AXIS_Y）
 * @param Y 新的Y轴对应的原轴（例如 AXIS_MINUS_X）
 * @param outR 输出重映射后的旋转矩阵
 * @return 是否成功
 */
static bool remapCoordinateSystem_3dof(const float* inR, int X, int Y, float* outR) {
    // 轴常量定义（与Android SensorManager保持一致）
    const int AXIS_X = 1;
    const int AXIS_Y = 2;
    const int AXIS_Z = 3;
    const int AXIS_MINUS_X = 129; // AXIS_X | 0x80
    const int AXIS_MINUS_Y = 130; // AXIS_Y | 0x80
    const int AXIS_MINUS_Z = 131; // AXIS_Z | 0x80
    
    // 解析轴映射
    int xAxis = X & 0x7F;
    int yAxis = Y & 0x7F;
    bool xNeg = (X & 0x80) != 0;
    bool yNeg = (Y & 0x80) != 0;
    
    // 验证输入
    if (xAxis < 1 || xAxis > 3 || yAxis < 1 || yAxis > 3 || xAxis == yAxis) {
        LOGD("无效的轴映射: X=%d, Y=%d", X, Y);
        return false;
    }
    
    // 计算Z轴（右手坐标系）
    int zAxis = 6 - xAxis - yAxis; // 剩余的轴
    
    // 构建变换矩阵
    float transform[16] = {0};
    
    // X轴映射
    int xSrcCol = xAxis - 1;
    int xDstRow = 0;
    transform[xDstRow * 4 + xSrcCol] = xNeg ? -1.0f : 1.0f;
    
    // Y轴映射
    int ySrcCol = yAxis - 1;
    int yDstRow = 1;
    transform[yDstRow * 4 + ySrcCol] = yNeg ? -1.0f : 1.0f;
    
    // Z轴映射（根据右手法则确定符号）
    int zSrcCol = zAxis - 1;
    int zDstRow = 2;
    // 计算叉积符号
    bool zNeg = ((xAxis - 1) == ((yAxis) % 3)) != (xNeg != yNeg);
    transform[zDstRow * 4 + zSrcCol] = zNeg ? -1.0f : 1.0f;
    
    // 第四行第四列
    transform[15] = 1.0f;
    
    // 执行矩阵乘法: outR = transform * inR
    multiplyMM(outR, transform, inR);
    
    return true;
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
    // 手动转置 viewMatrix 到 invViewMatrix
    for(int i=0; i<4; ++i) {
        for(int j=0; j<4; ++j) {
            invViewMatrix[i*4+j] = viewMatrix[j*4+i];
        }
    }

    float targetPosCameraSpace[4] = { 0.0f, 0.0f, -distance, 1.0f };
    float worldPos[4];
    for(int i=0; i<4; ++i) {
        worldPos[i] = 0;
        for(int j=0; j<4; ++j) {
            worldPos[i] += invViewMatrix[i+j*4] * targetPosCameraSpace[j];
        }
    }

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
    frustumM(projectionMatrix, 0, -ratio, ratio, -1, 1, 1.0f, 100.0f);
    
    // 计算模型矩阵 (平移到指定的世界坐标并添加固定自转)
    float modelMatrix[16];
    setIdentityM(modelMatrix);
    modelMatrix[12] = worldPos[0];
    modelMatrix[13] = worldPos[1];
    modelMatrix[14] = worldPos[2];
    rotateM(modelMatrix, modelMatrix, 45.0f, 0.0f, 1.0f, 0.0f);
    rotateM(modelMatrix, modelMatrix, 30.0f, 1.0f, 0.0f, 0.0f);
        
    // 组合变换: MVP = Projection * View * Model
    float tempMatrix[16];
    multiplyMM(tempMatrix, viewMatrix, modelMatrix);
        
    float mvpMatrix[16];
    multiplyMM(mvpMatrix, projectionMatrix, tempMatrix);
        
    jfloatArray result = env->NewFloatArray(16);
    env->SetFloatArrayRegion(result, 0, 16, mvpMatrix);
    return result;
}

// ========== OrientationSensor JNI 接口 ==========

/**
 * 从旋转矩阵获取欧拉角
 * Java 签名: native float[] getOrientationFromRotationMatrix(float[] rotationMatrix)
 */
JNIEXPORT jfloatArray JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_getOrientationFromRotationMatrix(JNIEnv* env, jobject thiz, jfloatArray rotationMatrix) {
    jsize len = env->GetArrayLength(rotationMatrix);
    if (len < 9) {
        LOGD("旋转矩阵大小不足: %d", len);
        return env->NewFloatArray(0);
    }
    
    float matrix[16];
    env->GetFloatArrayRegion(rotationMatrix, 0, len >= 16 ? 16 : 9, matrix);
    
    // 如果是9元素（3x3矩阵），扩展为4x4
    if (len == 9) {
        float temp[9];
        memcpy(temp, matrix, 9 * sizeof(float));
        memset(matrix, 0, 16 * sizeof(float));
        // 重新排列为4x4
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                matrix[i * 4 + j] = temp[i * 3 + j];
            }
        }
        matrix[15] = 1.0f;
    }
    
    float values[3];
    getOrientationFromMatrix_3dof(matrix, values);
    
    jfloatArray result = env->NewFloatArray(3);
    env->SetFloatArrayRegion(result, 0, 3, values);
    
    return result;
}

/**
 * 坐标系重映射
 * Java 签名: native float[] remapCoordinateSystem(float[] inR, int X, int Y)
 */
JNIEXPORT jfloatArray JNICALL
Java_com_orb_slam2s_slamar_NativeHelper_remapCoordinateSystemNative(JNIEnv* env, jobject thiz, jfloatArray inR, jint X, jint Y) {
    jsize len = env->GetArrayLength(inR);
    if (len < 9) {
        LOGD("输入矩阵大小不足: %d", len);
        return env->NewFloatArray(0);
    }
    
    float inMatrix[16] = {0};
    float outMatrix[16] = {0};
    
    env->GetFloatArrayRegion(inR, 0, len >= 16 ? 16 : 9, inMatrix);
    
    // 如果是9元素（3x3矩阵），扩展为4x4
    if (len == 9) {
        float temp[9];
        memcpy(temp, inMatrix, 9 * sizeof(float));
        memset(inMatrix, 0, 16 * sizeof(float));
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                inMatrix[i * 4 + j] = temp[i * 3 + j];
            }
        }
        inMatrix[15] = 1.0f;
    }
    
    bool success = remapCoordinateSystem_3dof(inMatrix, X, Y, outMatrix);
    
    if (!success) {
        return env->NewFloatArray(0);
    }
    
    jfloatArray result = env->NewFloatArray(16);
    env->SetFloatArrayRegion(result, 0, 16, outMatrix);
    
    return result;
}

}
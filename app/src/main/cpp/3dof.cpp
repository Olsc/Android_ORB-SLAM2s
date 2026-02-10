#include <jni.h>
#include <math.h>
#include <string.h>
#include <android/log.h>

#define LOG_TAG "3DOF"
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)

extern "C" {

// 3DOF辅助函数：设置单位矩阵
static void setIdentityM_3dof(float* sm, int smOffset) {
    for (int i=0; i<16; i++) sm[smOffset+i] = 0;
    sm[smOffset+0] = 1; sm[smOffset+5] = 1; sm[smOffset+10] = 1; sm[smOffset+15] = 1;
}

// 3DOF辅助函数：矩阵转置
static void transposeM_3dof(float* mTrans, int mTransOffset, float* m, int mOffset) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            mTrans[mTransOffset + i * 4 + j] = m[mOffset + j * 4 + i];
        }
    }
}

// 3DOF辅助函数：矩阵乘法
static void multiplyMM_3dof(float* result, int resultOffset, const float* lhs, int lhsOffset, const float* rhs, int rhsOffset) {
    float temp[16];
    for (int i = 0; i < 4; i++) {
        const float lhs0 = lhs[lhsOffset + 0*4 + i];
        const float lhs1 = lhs[lhsOffset + 1*4 + i];
        const float lhs2 = lhs[lhsOffset + 2*4 + i];
        const float lhs3 = lhs[lhsOffset + 3*4 + i];
        for (int j = 0; j < 4; j++) {
            temp[j*4+i] = lhs0 * rhs[rhsOffset + j*4 + 0]
                        + lhs1 * rhs[rhsOffset + j*4 + 1]
                        + lhs2 * rhs[rhsOffset + j*4 + 2]
                        + lhs3 * rhs[rhsOffset + j*4 + 3];
        }
    }
    memcpy(result+resultOffset, temp, 16*sizeof(float));
}

// 3DOF辅助函数：矩阵向量乘法
static void multiplyMV_3dof(float* resultVec, int resultVecOffset, const float* lhsMat, int lhsMatOffset, const float* rhsVec, int rhsVecOffset) {
    float x = rhsVec[rhsVecOffset + 0];
    float y = rhsVec[rhsVecOffset + 1];
    float z = rhsVec[rhsVecOffset + 2];
    float w = rhsVec[rhsVecOffset + 3];
    resultVec[resultVecOffset + 0] = lhsMat[lhsMatOffset + 0]*x + lhsMat[lhsMatOffset + 4]*y + lhsMat[lhsMatOffset + 8]*z + lhsMat[lhsMatOffset + 12]*w;
    resultVec[resultVecOffset + 1] = lhsMat[lhsMatOffset + 1]*x + lhsMat[lhsMatOffset + 5]*y + lhsMat[lhsMatOffset + 9]*z + lhsMat[lhsMatOffset + 13]*w;
    resultVec[resultVecOffset + 2] = lhsMat[lhsMatOffset + 2]*x + lhsMat[lhsMatOffset + 6]*y + lhsMat[lhsMatOffset + 10]*z + lhsMat[lhsMatOffset + 14]*w;
    resultVec[resultVecOffset + 3] = lhsMat[lhsMatOffset + 3]*x + lhsMat[lhsMatOffset + 7]*y + lhsMat[lhsMatOffset + 11]*z + lhsMat[lhsMatOffset + 15]*w;
}

// 3DOF辅助函数：透视投影矩阵
static void frustumM_3dof(float* m, int offset, float left, float right, float bottom, float top, float near, float far) {
    float r_width  = 1.0f / (right - left);
    float r_height = 1.0f / (top - bottom);
    float r_depth  = 1.0f / (near - far);
    float x = 2.0f * (near * r_width);
    float y = 2.0f * (near * r_height);
    float A = (right + left) * r_width;
    float B = (top + bottom) * r_height;
    float C = (near + far) * r_depth;
    float D = 2.0f * (near * far) * r_depth;
    memset(m+offset, 0, 16*sizeof(float));
    m[offset + 0] = x;
    m[offset + 5] = y;
    m[offset + 8] = A;
    m[offset + 9] = B;
    m[offset + 10] = C;
    m[offset + 11] = -1.0f;
    m[offset + 14] = D;
}

// 3DOF辅助函数：平移矩阵
static void translateM_3dof(float* m, int mOffset, float x, float y, float z) {
    for (int i=0 ; i<4 ; i++) {
        int mi = mOffset + i;
        m[12 + mi] += m[mi] * x + m[4 + mi] * y + m[8 + mi] * z;
    }
}

// 3DOF辅助函数：旋转矩阵
static void rotateM_3dof(float* m, int mOffset, float a, float x, float y, float z) {
    float temp[16];
    setIdentityM_3dof(temp, 0);
    float rad = a * 3.14159265358979323846f / 180.0f;
    float c = cos(rad);
    float s = sin(rad);
    float nc = 1.0f - c;
    float len = sqrt(x*x + y*y + z*z);
    if(len != 1.0f && len != 0.0f) {
        float rlen = 1.0f / len;
        x *= rlen; y *= rlen; z *= rlen;
    }
    float rm[16];
    setIdentityM_3dof(rm, 0);
    rm[0] = x*x*nc + c;   rm[4] = x*y*nc - z*s; rm[8] = x*z*nc + y*s;
    rm[1] = x*y*nc + z*s; rm[5] = y*y*nc + c;   rm[9] = y*z*nc - x*s;
    rm[2] = x*z*nc - y*s; rm[6] = y*z*nc + x*s; rm[10]= z*z*nc + c;
    multiplyMM_3dof(temp, 0, m, mOffset, rm, 0);
    memcpy(m+mOffset, temp, 16*sizeof(float));
}

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
    multiplyMM_3dof(outR, 0, transform, 0, inR, 0);
    
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
        setIdentityM_3dof(deviceRotation, 0);
    }

    // 获取根据屏幕旋转调整后的视图矩阵
    float viewMatrix[16];
    getRemappedMatrix_3dof(viewMatrix, deviceRotation, rotation);

    // 计算方块在世界空间中的坐标: Inv(View) * (0, 0, -distance)
    float invViewMatrix[16];
    transposeM_3dof(invViewMatrix, 0, viewMatrix, 0);

    float targetPosCameraSpace[4] = { 0.0f, 0.0f, -distance, 1.0f };
    float worldPos[4];
    multiplyMV_3dof(worldPos, 0, invViewMatrix, 0, targetPosCameraSpace, 0);

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
        setIdentityM_3dof(deviceRotation, 0);
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
    frustumM_3dof(projectionMatrix, 0, -ratio, ratio, -1, 1, 1.0f, 100.0f);
    
    // 计算模型矩阵 (平移到指定的世界坐标并添加固定自转)
    float modelMatrix[16];
    setIdentityM_3dof(modelMatrix, 0);
    translateM_3dof(modelMatrix, 0, worldPos[0], worldPos[1], worldPos[2]);
    rotateM_3dof(modelMatrix, 0, 45.0f, 0.0f, 1.0f, 0.0f);
    rotateM_3dof(modelMatrix, 0, 30.0f, 1.0f, 0.0f, 0.0f);
        
    // 组合变换: MVP = Projection * View * Model
    float tempMatrix[16];
    multiplyMM_3dof(tempMatrix, 0, viewMatrix, 0, modelMatrix, 0);
        
    float mvpMatrix[16];
    multiplyMM_3dof(mvpMatrix, 0, projectionMatrix, 0, tempMatrix, 0);
        
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

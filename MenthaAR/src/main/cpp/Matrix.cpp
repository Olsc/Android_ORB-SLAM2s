/**
 * Created by Ads on 2017/3/5.
 * 由Olsc于2025/8/25开始进行修改
 */

/**
 * 矩阵运算模块实现
 * 实现了3D图形学中常用的矩阵和四元数运算
 */

#include "Matrix.h"
#include <cmath>
#include <cstring>
#include <string>
#include <algorithm>
#include <include/Common.h>

const float PI= (const float) acos(-1);  // π常量
#define I(_i, _j) ((_j)+ 4*(_i))  // 列主序矩阵索引宏（OpenGL风格）

/**
 * 4x4矩阵乘法（列主序）
 * 实现 r = lhs * rhs
 */
void multiplyMM(float* r, const float* lhs, const float* rhs) {
    float tmp[16];
    // 第0列
    float rhs_00 = rhs[0]; float rhs_01 = rhs[1]; float rhs_02 = rhs[2]; float rhs_03 = rhs[3];
    tmp[0] = lhs[0] * rhs_00 + lhs[4] * rhs_01 + lhs[8] * rhs_02 + lhs[12] * rhs_03;
    tmp[1] = lhs[1] * rhs_00 + lhs[5] * rhs_01 + lhs[9] * rhs_02 + lhs[13] * rhs_03;
    tmp[2] = lhs[2] * rhs_00 + lhs[6] * rhs_01 + lhs[10] * rhs_02 + lhs[14] * rhs_03;
    tmp[3] = lhs[3] * rhs_00 + lhs[7] * rhs_01 + lhs[11] * rhs_02 + lhs[15] * rhs_03;

    // 第1列
    float rhs_10 = rhs[4]; float rhs_11 = rhs[5]; float rhs_12 = rhs[6]; float rhs_13 = rhs[7];
    tmp[4] = lhs[0] * rhs_10 + lhs[4] * rhs_11 + lhs[8] * rhs_12 + lhs[12] * rhs_13;
    tmp[5] = lhs[1] * rhs_10 + lhs[5] * rhs_11 + lhs[9] * rhs_12 + lhs[13] * rhs_13;
    tmp[6] = lhs[2] * rhs_10 + lhs[6] * rhs_11 + lhs[10] * rhs_12 + lhs[14] * rhs_13;
    tmp[7] = lhs[3] * rhs_10 + lhs[7] * rhs_11 + lhs[11] * rhs_12 + lhs[15] * rhs_13;

    // 第2列
    float rhs_20 = rhs[8]; float rhs_21 = rhs[9]; float rhs_22 = rhs[10]; float rhs_23 = rhs[11];
    tmp[8] =  lhs[0] * rhs_20 + lhs[4] * rhs_21 + lhs[8] * rhs_22 + lhs[12] * rhs_23;
    tmp[9] =  lhs[1] * rhs_20 + lhs[5] * rhs_21 + lhs[9] * rhs_22 + lhs[13] * rhs_23;
    tmp[10] = lhs[2] * rhs_20 + lhs[6] * rhs_21 + lhs[10] * rhs_22 + lhs[14] * rhs_23;
    tmp[11] = lhs[3] * rhs_20 + lhs[7] * rhs_21 + lhs[11] * rhs_22 + lhs[15] * rhs_23;

    // 第3列
    float rhs_30 = rhs[12]; float rhs_31 = rhs[13]; float rhs_32 = rhs[14]; float rhs_33 = rhs[15];
    tmp[12] = lhs[0] * rhs_30 + lhs[4] * rhs_31 + lhs[8] * rhs_32 + lhs[12] * rhs_33;
    tmp[13] = lhs[1] * rhs_30 + lhs[5] * rhs_31 + lhs[9] * rhs_32 + lhs[13] * rhs_33;
    tmp[14] = lhs[2] * rhs_30 + lhs[6] * rhs_31 + lhs[10] * rhs_32 + lhs[14] * rhs_33;
    tmp[15] = lhs[3] * rhs_30 + lhs[7] * rhs_31 + lhs[11] * rhs_32 + lhs[15] * rhs_33;

    memcpy(r, tmp, 16 * sizeof(float));
}

void multiplyMV(float* resultVec, int resultVecOffset, const float* lhsMat, int lhsMatOffset,
                const float* rhsVec, int rhsVecOffset) {
    float x = rhsVec[rhsVecOffset + 0];
    float y = rhsVec[rhsVecOffset + 1];
    float z = rhsVec[rhsVecOffset + 2];
    float w = rhsVec[rhsVecOffset + 3];

    resultVec[resultVecOffset + 0] = lhsMat[lhsMatOffset + 0]*x + lhsMat[lhsMatOffset + 4]*y + lhsMat[lhsMatOffset + 8]*z + lhsMat[lhsMatOffset + 12]*w;
    resultVec[resultVecOffset + 1] = lhsMat[lhsMatOffset + 1]*x + lhsMat[lhsMatOffset + 5]*y + lhsMat[lhsMatOffset + 9]*z + lhsMat[lhsMatOffset + 13]*w;
    resultVec[resultVecOffset + 2] = lhsMat[lhsMatOffset + 2]*x + lhsMat[lhsMatOffset + 6]*y + lhsMat[lhsMatOffset + 10]*z + lhsMat[lhsMatOffset + 14]*w;
    resultVec[resultVecOffset + 3] = lhsMat[lhsMatOffset + 3]*x + lhsMat[lhsMatOffset + 7]*y + lhsMat[lhsMatOffset + 11]*z + lhsMat[lhsMatOffset + 15]*w;
}

void transposeM(float* mTrans, int mTransOffset, const float* m, int mOffset) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            mTrans[mTransOffset + i * 4 + j] = m[mOffset + j * 4 + i];
        }
    }
}

void translateM(float* m, int mOffset, float x, float y, float z) {
    for (int i=0 ; i<4 ; i++) {
        int mi = mOffset + i;
        m[12 + mi] += m[mi] * x + m[4 + mi] * y + m[8 + mi] * z;
    }
}

/**
 * 计算3D向量的长度
 */
float length(float x, float y, float z) {
    return (float) sqrt(x * x + y * y + z * z);
}

/**
 * 创建绕任意轴的旋转矩阵
 * 使用Rodrigues旋转公式
 * 优化策略: 对X/Y/Z轴旋转进行特殊处理，避免不必要的计算
 */
void setRotateM(float rm[], int rmOffset,
                float a, float x, float y, float z) {
    // 设置齐次坐标的固定值
    rm[rmOffset + 3] = 0;
    rm[rmOffset + 7] = 0;
    rm[rmOffset + 11]= 0;
    rm[rmOffset + 12]= 0;
    rm[rmOffset + 13]= 0;
    rm[rmOffset + 14]= 0;
    rm[rmOffset + 15]= 1;

    a *= (float) (PI / 180.0f);  // 角度转弧度
    float s = (float) sin(a);
    float c = (float) cos(a);

    // 特殊情况优化：绕X轴旋转
    if (1.0f == x && 0.0f == y && 0.0f == z) {
        rm[rmOffset + 5] = c;   rm[rmOffset + 10]= c;
        rm[rmOffset + 6] = s;   rm[rmOffset + 9] = -s;
        rm[rmOffset + 1] = 0;   rm[rmOffset + 2] = 0;
        rm[rmOffset + 4] = 0;   rm[rmOffset + 8] = 0;
        rm[rmOffset + 0] = 1;
    } 
    // 特殊情况优化：绕Y轴旋转
    else if (0.0f == x && 1.0f == y && 0.0f == z) {
        rm[rmOffset + 0] = c;   rm[rmOffset + 10]= c;
        rm[rmOffset + 8] = s;   rm[rmOffset + 2] = -s;
        rm[rmOffset + 1] = 0;   rm[rmOffset + 4] = 0;
        rm[rmOffset + 6] = 0;   rm[rmOffset + 9] = 0;
        rm[rmOffset + 5] = 1;
    } 
    // 特殊情况优化：绕Z轴旋转
    else if (0.0f == x && 0.0f == y && 1.0f == z) {
        rm[rmOffset + 0] = c;   rm[rmOffset + 5] = c;
        rm[rmOffset + 1] = s;   rm[rmOffset + 4] = -s;
        rm[rmOffset + 2] = 0;   rm[rmOffset + 6] = 0;
        rm[rmOffset + 8] = 0;   rm[rmOffset + 9] = 0;
        rm[rmOffset + 10]= 1;
    } 
    // 一般情况：绕任意轴旋转（使用Rodrigues公式）
    else {
        float len = length(x, y, z);
        if (1.0f != len) {  // 归一化旋转轴
            float recipLen = 1.0f / len;
            x *= recipLen;
            y *= recipLen;
            z *= recipLen;
        }
        float nc = 1.0f - c;  // 1 - cos(a)
        float xy = x * y;
        float yz = y * z;
        float zx = z * x;
        float xs = x * s;
        float ys = y * s;
        float zs = z * s;
        // 填充旋转矩阵（Rodrigues公式）
        rm[rmOffset +  0] = x*x*nc +  c;
        rm[rmOffset +  4] =  xy*nc - zs;
        rm[rmOffset +  8] =  zx*nc + ys;
        rm[rmOffset +  1] =  xy*nc + zs;
        rm[rmOffset +  5] = y*y*nc +  c;
        rm[rmOffset +  9] =  yz*nc - xs;
        rm[rmOffset +  2] =  zx*nc - ys;
        rm[rmOffset +  6] =  yz*nc + xs;
        rm[rmOffset + 10] = z*z*nc +  c;
    }
}

/**
 * 对矩阵应用旋转变换
 * 先创建旋转矩阵，然后与原矩阵相乘
 */
void rotateM(float rm[], float m[],
                           float a, float x, float y, float z) {
    // 使用局部变量确保线程安全（避免多线程竞争）
    float sTemp[16];
    setRotateM(sTemp, 0, a, x, y, z);

    // 检查原地操作
    if (rm == m) {
        float tmpResult[16];
        multiplyMM(tmpResult, m, sTemp);
        memcpy(rm, tmpResult, 16 * sizeof(float));
    } else {
        multiplyMM(rm, m, sTemp);
    }
}

/**
 * 创建OpenGL风格的透视投影矩阵
 * 定义一个视锥体（frustum），用于3D透视投影
 */
void frustumM(float m[], int offset,
              float left, float right, float bottom, float top,
              float nearZ, float farZ) {

    // 预计算倒数，将除法转换为乘法
    float r_width  = 1.0f / (right - left);
    float r_height = 1.0f / (top - bottom);
    float r_depth  = 1.0f / (nearZ - farZ);
    float x = 2.0f * nearZ * r_width;
    float y = 2.0f * nearZ * r_height;
    float A = (right + left) * r_width;
    float B = (top + bottom) * r_height;
    float C = (farZ + nearZ) * r_depth;
    float D = 2.0f * farZ * nearZ * r_depth;
    m[offset + 0] = x;
    m[offset + 5] = y;
    m[offset + 8] = A;
    m[offset +  9] = B;
    m[offset + 10] = C;
    m[offset + 14] = D;
    m[offset + 11] = -1.0f;
    m[offset +  1] = 0.0f;
    m[offset +  2] = 0.0f;
    m[offset +  3] = 0.0f;
    m[offset +  4] = 0.0f;
    m[offset +  6] = 0.0f;
    m[offset +  7] = 0.0f;
    m[offset + 12] = 0.0f;
    m[offset + 13] = 0.0f;
    m[offset + 15] = 0.0f;
}

/**
 * 从相机内参创建RUB坐标系的透视投影矩阵
 * 参考: http://www.songho.ca/opengl/gl_projectionmatrix.html
 */
void frustumM_RUB(int w, int h, double fu, double fv, double u0, double v0, double zNear, double zFar ,float projectionMatrix[]) {
    // 根据相机内参计算视锥体边界
    const double L = -(u0) * zNear / fu;        // 左边界
    const double R = +(w - u0) * zNear / fu;    // 右边界
    const double T = +(v0) * zNear / fv;        // 上边界
    const double B = -(h - v0) * zNear / fv;    // 下边界
    frustumM(projectionMatrix,0,L,R,B,T,zNear,zFar);
}

/**
 * 设置为4x4单位矩阵
 */
void setIdentityM(float m[])
{
    m[0] = 1.0f;  m[1] = 0.0f;  m[2] = 0.0f;  m[3] = 0.0f;
    m[4] = 0.0f;  m[5] = 1.0f;  m[6] = 0.0f;  m[7] = 0.0f;
    m[8] = 0.0f;  m[9] = 0.0f; m[10] = 1.0f; m[11] = 0.0f;
    m[12] = 0.0f; m[13] = 0.0f; m[14] = 0.0f; m[15] = 1.0f;
}

void getRUBViewMatrixFromRDF(float inM[],float outM[]){
    // OpenGL 视图矩阵 = Rx(180) * OpenCV 视图矩阵 * Rx(180)
    // Rx(180) 翻转 Y 和 Z 轴。
    // 手机AR渲染需要双重变换：不仅相机坐标系变换(左乘)，世界坐标系也变换(右乘)。
    // 这样可以确保AR物体(RUB模型)在RUB世界中被RUB相机正确观察。

    if(inM != outM) {
        memcpy(outM, inM, 16 * sizeof(float));
    }

    // 我们需要取反以下索引：
    // Row 1 (indices 1, 5, 9, 13) 被左乘取反
    // Row 2 (indices 2, 6, 10, 14) 被左乘取反
    // Col 1 (indices 4, 5, 6, 7) 被右乘取反
    // Col 2 (indices 8, 9, 10, 11) 被右乘取反

    // 重叠部分(Row 1/2 AND Col 1/2)被取反两次 -> 保持不变
    // 重叠索引：5, 9, 6, 10

    // 最终需要取反的索引列表：
    // 仅行：1, 13, 2, 14
    // 仅列：4, 7, 8, 11

    outM[1] = -outM[1];
    outM[2] = -outM[2];
    outM[4] = -outM[4];
    outM[7] = -outM[7];
    outM[8] = -outM[8];
    outM[11] = -outM[11];
    outM[13] = -outM[13];
    outM[14] = -outM[14];
}

void getRUBModelMatrixFromRDF(float inM[],float outM[]){
    if(inM != outM) {
        memcpy(outM, inM, 16 * sizeof(float));
    }

    // R_x(180) * inM: inM的第1行和第2行取反，第0行和第3行保持不变
    // 第1行索引: 1, 5, 9, 13
    // 第2行索引: 2, 6, 10, 14
    outM[1] = -outM[1];
    outM[2] = -outM[2];
    outM[5] = -outM[5];
    outM[6] = -outM[6];
    outM[9] = -outM[9];
    outM[10] = -outM[10];
    outM[13] = -outM[13];
    outM[14] = -outM[14];
}

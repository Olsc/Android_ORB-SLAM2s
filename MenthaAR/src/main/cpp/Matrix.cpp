/*
 * Portions Copyright (C) 2007 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Copyright (C) 2025-2026 Olsc <OlscStudio@outlook.com>
 * This file is part of the Android ORB-SLAM2s project (a fork of ORB-SLAM2)
 * and is distributed under the GNU General Public License v3.0.
 * See MenthaAR/LICENSE for details.
 */

/**
 * Created by Ads on 2017/3/5.
 * 由Olsc于2025/8/25开始进行修改
 */

// 矩阵运算模块实现

#include "Matrix.h"
#include <cstring>

// 4x4矩阵乘法（列主序），r = lhs * rhs
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

// 创建OpenGL风格透视投影矩阵，定义视锥体用于3D透视投影
void frustumM(float m[], int offset,
              float left, float right, float bottom, float top,
              float nearZ, float farZ) {

    // 预计算倒数
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

// 从相机内参创建RUB坐标系透视投影矩阵
void frustumM_RUB(int w, int h, double fu, double fv, double u0, double v0, double zNear, double zFar ,float projectionMatrix[]) {
    // 根据相机内参计算视锥体边界
    const double L = -(u0) * zNear / fu; // 左边界
    const double R = +(w - u0) * zNear / fu; // 右边界
    const double T = +(v0) * zNear / fv; // 上边界
    const double B = -(h - v0) * zNear / fv; // 下边界
    frustumM(projectionMatrix,0,L,R,B,T,zNear,zFar);
}

// 设置为4x4单位矩阵
void setIdentityM(float m[])
{
    m[0] = 1.0f;  m[1] = 0.0f;  m[2] = 0.0f;  m[3] = 0.0f;
    m[4] = 0.0f;  m[5] = 1.0f;  m[6] = 0.0f;  m[7] = 0.0f;
    m[8] = 0.0f;  m[9] = 0.0f; m[10] = 1.0f; m[11] = 0.0f;
    m[12] = 0.0f; m[13] = 0.0f; m[14] = 0.0f; m[15] = 1.0f;
}

void getRUBViewMatrixFromRDF(float inM[],float outM[]){
    if(inM != outM) {
        memcpy(outM, inM, 16 * sizeof(float));
    }

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

    outM[1] = -outM[1];
    outM[2] = -outM[2];
    outM[5] = -outM[5];
    outM[6] = -outM[6];
    outM[9] = -outM[9];
    outM[10] = -outM[10];
    outM[13] = -outM[13];
    outM[14] = -outM[14];
}
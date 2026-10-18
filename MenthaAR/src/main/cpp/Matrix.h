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

// 矩阵运算模块：提供 3D 图形学常用的矩阵与四元数运算，用于 AR 渲染和坐标转换

#ifndef ORB_SLAM2_AR_MATRIX_H
#define ORB_SLAM2_AR_MATRIX_H

#include <cmath>

// 4x4矩阵乘法，r = lhs * rhs
void multiplyMM(float* r, const float* lhs, const float* rhs);

// 矩阵向量乘法，result = lhs * rhs
void multiplyMV(float* resultVec, int resultVecOffset, const float* lhsMat, int lhsMatOffset,
                const float* rhsVec, int rhsVecOffset);

// 矩阵转置
void transposeM(float* mTrans, int mTransOffset, const float* m, int mOffset);

// 矩阵平移，m = m * T
void translateM(float* m, int mOffset, float x, float y, float z);

// 创建绕任意轴的旋转矩阵
void setRotateM(float rm[], int rmOffset,
                float a, float x, float y, float z);

// 对已有矩阵应用旋转，rm = m * R
void rotateM(float rm[], float m[],
             float a, float x, float y, float z);

// 创建透视投影矩阵（OpenGL风格）
void frustumM(float m[], int offset,
              float left, float right, float bottom, float top,
              float nearZ, float farZ);

// 创建RUB坐标系的透视投影矩阵，RUB为OpenGL标准的右-上-后坐标系
void frustumM_RUB(int w, int h, double fu, double fv, double u0, double v0, double zNear, double zFar ,float projectionMatrix[]);

// 设置为单位矩阵
void setIdentityM(float m[]);

// 将RDF坐标系矩阵转换为RUB坐标系视图矩阵，RDF为SLAM常用的右-下-前坐标系
void getRUBViewMatrixFromRDF(float inM[],float outM[]);

// 将RDF坐标系矩阵转换为RUB坐标系模型矩阵
void getRUBModelMatrixFromRDF(float inM[],float outM[]);

// 对4x4列主序矩阵的3x3旋转部分做Gram-Schmidt正交规范化
void orthonormalize(float* m);

// 从旋转矢量/四元数计算4x4列主序旋转矩阵 (兼容3元素与4元素矢量)
void getRotationMatrixFromVector(float* outR, const float* rotationVector, int length);

// 从加速度计(重力)和磁力计(地磁)计算4x4列主序旋转矩阵
bool getRotationMatrixFromAccelMag(float* outR, const float* gravity, const float* geomagnetic);

// 重映射坐标系 (支持标准与反转横屏及通用轴重映射，对应Android SensorManager坐标轴常量)
bool remapCoordinateSystem(const float* inR, int X, int Y, float* outR);

#endif // ORB_SLAM2_AR_MATRIX_H
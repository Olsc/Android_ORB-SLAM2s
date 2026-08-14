/**
 * Created by Ads on 2017/3/5.
 * 由Olsc于2025/8/25开始进行修改
 */

// 矩阵运算模块：提供3D图形学常用的矩阵与四元数运算，用于AR渲染和坐标转换

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

#endif //ORB_SLAM2_AR_MATRIX_H
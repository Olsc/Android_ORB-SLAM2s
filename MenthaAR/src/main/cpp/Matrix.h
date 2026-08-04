/**
 * Created by Ads on 2017/3/5.
 * 由Olsc于2025/8/25开始进行修改
 */

/**
 * 矩阵运算模块
 * 功能描述: 提供3D图形学中常用的矩阵和四元数运算，主要用于AR渲染和坐标转换
 */

#ifndef ORB_SLAM2_AR_MATRIX_H
#define ORB_SLAM2_AR_MATRIX_H

#include <cmath>

/**
 * 4x4矩阵乘法 (r = lhs * rhs)
 * @param r 结果矩阵（16个float）
 * @param lhs 左操作数矩阵（16个float）
 * @param rhs 右操作数矩阵（16个float）
 */
void multiplyMM(float* r, const float* lhs, const float* rhs);

/**
 * 矩阵向量乘法 (result = lhs * rhs)
 * @param resultVec 结果向量 (4个float)
 * @param resultVecOffset 结果向量偏移
 * @param lhsMat 左操作数矩阵 (16个float)
 * @param lhsMatOffset 矩阵偏移
 * @param rhsVec 右操作数向量 (4个float)
 * @param rhsVecOffset 向量偏移
 */
void multiplyMV(float* resultVec, int resultVecOffset, const float* lhsMat, int lhsMatOffset,
                const float* rhsVec, int rhsVecOffset);

/**
 * 矩阵转置
 * @param mTrans 输出转置矩阵 (16个float)
 * @param mTransOffset 输出偏移
 * @param m 输入矩阵 (16个float)
 * @param mOffset 输入偏移
 */
void transposeM(float* mTrans, int mTransOffset, const float* m, int mOffset);

/**
 * 矩阵平移 (m = m * T)
 * @param m 输入/输出矩阵 (16个float)
 * @param mOffset 矩阵偏移
 * @param x 平移X
 * @param y 平移Y
 * @param z 平移Z
 */
void translateM(float* m, int mOffset, float x, float y, float z);

/**
 * 创建绕任意轴的旋转矩阵
 * @param rm 输出矩阵（16个float）
 * @param rmOffset 输出矩阵的偏移量
 * @param a 旋转角度（度）
 * @param x 旋转轴X分量
 * @param y 旋转轴Y分量
 * @param z 旋转轴Z分量
 */
void setRotateM(float rm[], int rmOffset,
                float a, float x, float y, float z);

/**
 * 对已有矩阵应用旋转 (rm = m * R)
 * @param rm 结果矩阵（16个float）
 * @param m 输入矩阵（16个float）
 * @param a 旋转角度（度）
 * @param x 旋转轴X分量
 * @param y 旋转轴Y分量
 * @param z 旋转轴Z分量
 */
void rotateM(float rm[], float m[],
             float a, float x, float y, float z);

/**
 * 创建透视投影矩阵（OpenGL风格）
 * @param m 输出矩阵（16个float）
 * @param offset 输出矩阵的偏移量
 * @param left 视锥左边界
 * @param right 视锥右边界
 * @param bottom 视锥下边界
 * @param top 视锥上边界
 * @param nearZ 近裁剪面
 * @param farZ 远裁剪面
 */
void frustumM(float m[], int offset,
              float left, float right, float bottom, float top,
              float nearZ, float farZ);

/**
 * 创建RUB坐标系的透视投影矩阵
 * RUB = Right-Up-Back坐标系（OpenGL标准）
 * @param w 图像宽度
 * @param h 图像高度
 * @param fu 相机内参：X轴焦距
 * @param fv 相机内参：Y轴焦距
 * @param u0 相机内参：X轴主点
 * @param v0 相机内参：Y轴主点
 * @param zNear 近裁剪面
 * @param zFar 远裁剪面
 * @param projectionMatrix 输出的投影矩阵（16个float）
 */
void frustumM_RUB(int w, int h, double fu, double fv, double u0, double v0, double zNear, double zFar ,float projectionMatrix[]);

/**
 * 设置为单位矩阵
 * @param m 输出矩阵（16个float）
 */
void setIdentityM(float m[]);

/**
 * 将RDF坐标系的矩阵转换为RUB坐标系的视图矩阵
 * RDF = Right-Down-Forward（SLAM常用）
 * RUB = Right-Up-Back（OpenGL标准）
 * @param inM 输入RDF矩阵（16个float）
 * @param outM 输出RUB视图矩阵（16个float）
 */
void getRUBViewMatrixFromRDF(float inM[],float outM[]);

/**
 * 将RDF坐标系的矩阵转换为RUB坐标系的模型矩阵
 * @param inM 输入RDF矩阵（16个float）
 * @param outM 输出RUB模型矩阵（16个float）
 */
void getRUBModelMatrixFromRDF(float inM[],float outM[]);

#endif //ORB_SLAM2_AR_MATRIX_H

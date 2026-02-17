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

#include <vector>
#include <stdexcept>
#include <cmath>

/**
 * 轻量矩阵类（用于EKF等数值运算）
 * 提供基本的矩阵创建、转置、求逆、加减与数乘/矩阵乘法运算
 */
class Mat
{
public:
    int rows, cols;
    std::vector<float> data;
    
    Mat() : rows(0), cols(0) {}
    
    Mat(int r, int c) : rows(r), cols(c), data(r * c, 0.0f) {}
    
    static Mat identity(int n)
    {
        Mat m(n, n);
        for (int i = 0; i < n; ++i)
        {
            m(i, i) = 1.0f;
        }
        return m;
    }
    
    float& operator()(int r, int c)
    {
        return data[r * cols + c];
    }
    
    float operator()(int r, int c) const
    {
        return data[r * cols + c];
    }
    
    Mat transpose() const
    {
        Mat t(cols, rows);
        
        for (int r = 0; r < rows; ++r)
        {
            for (int c = 0; c < cols; ++c)
            {
                t(c, r) = (*this)(r, c);
            }
        }
        
        return t;
    }
    
    Mat inverse() const
    {
        if (rows != cols) throw std::runtime_error("not square");
        
        int n = rows;
        Mat a(*this);
        Mat inv = Mat::identity(n);
        
        for (int i = 0; i < n; ++i)
        {
            float pivot = a(i, i);
            
            if (std::abs(pivot) < 1e-8f)
            {
                pivot = 1e-8f;
            }
            
            float invPivot = 1.0f / pivot;
            
            for (int j = 0; j < n; ++j)
            {
                a(i, j) *= invPivot;
                inv(i, j) *= invPivot;
            }
            
            for (int r = 0; r < n; ++r)
            {
                if (r == i) continue;
                
                float factor = a(r, i);
                
                for (int c = 0; c < n; ++c)
                {
                    a(r, c) -= factor * a(i, c);
                    inv(r, c) -= factor * inv(i, c);
                }
            }
        }
        
        return inv;
    }
};

inline Mat operator*(const Mat& A, const Mat& B)
{
    if (A.cols != B.rows) throw std::runtime_error("dim mismatch");
    
    Mat C(A.rows, B.cols);
    
    for (int i = 0; i < A.rows; ++i)
    {
        for (int j = 0; j < B.cols; ++j)
        {
            float s = 0.0f;
            
            for (int k = 0; k < A.cols; ++k)
            {
                s += A(i, k) * B(k, j);
            }
            
            C(i, j) = s;
        }
    }
    
    return C;
}

inline Mat operator+(const Mat& A, const Mat& B)
{
    if (A.rows != B.rows || A.cols != B.cols) throw std::runtime_error("dim mismatch");
    
    Mat C(A.rows, A.cols);
    
    for (int r = 0; r < A.rows; ++r)
    {
        for (int c = 0; c < A.cols; ++c)
        {
            C(r, c) = A(r, c) + B(r, c);
        }
    }
    
    return C;
}

inline Mat operator-(const Mat& A, const Mat& B)
{
    if (A.rows != B.rows || A.cols != B.cols) throw std::runtime_error("dim mismatch");
    
    Mat C(A.rows, A.cols);
    
    for (int r = 0; r < A.rows; ++r)
    {
        for (int c = 0; c < A.cols; ++c)
        {
            C(r, c) = A(r, c) - B(r, c);
        }
    }
    
    return C;
}

inline Mat operator*(const Mat& A, float s)
{
    Mat C(A.rows, A.cols);
    
    for (int r = 0; r < A.rows; ++r)
    {
        for (int c = 0; c < A.cols; ++c)
        {
            C(r, c) = A(r, c) * s;
        }
    }
    
    return C;
}

inline Mat operator*(float s, const Mat& A)
{
    return A * s;
}

/**
 * 四元数结构体
 * 用于表示旋转，避免万向节锁问题
 */
struct Quaternion
{
    float x, y, z, w;
    
    /**
     * 将四元数数据拷贝到数组
     * @param outM 输出数组
     * @param offset 起始偏移量
     */
    void copyTo(float outM[], int offset)
    {
        outM[offset + 0] = x;
        outM[offset + 1] = y;
        outM[offset + 2] = z;
        outM[offset + 3] = w;
    }
};

void quatRotateVector(const Quaternion& q, const float in[3], float out[3]);

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
 * 计算4x4矩阵的逆矩阵（使用Cramer法则）
 * @param mInv 输出逆矩阵（16个float）
 * @param mInvOffset 逆矩阵的偏移量
 * @param m 输入矩阵（16个float）
 * @param mOffset 输入矩阵的偏移量
 * @return true-成功求逆, false-矩阵奇异无法求逆
 */
bool invertM(float mInv[], int mInvOffset, float m[],
             int mOffset);

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

/**
 * 从旋转矩阵提取欧拉角
 * @param M 输入旋转矩阵（16个float）
 * @param outM 输出欧拉角数组（roll, pitch, yaw，单位：度）
 * @param offset 输出数组的偏移量
 */
void getEulerAnglesFromMatrix(float M[],float outM[],int offset);

/**
 * 四元数转旋转矩阵
 * @param M 输出旋转矩阵（16个float）
 * @param q 输入四元数
 */
void quaternionToMatrix(float M[],Quaternion &q);

/**
 * 旋转矩阵转四元数
 * @param M 输入旋转矩阵（16个float）
 * @param q 输出四元数
 */
void matrixToQuaternion(float M[],Quaternion &q);

/**
 * 旋转矩阵转四元数（Unity3D风格）
 * 参考: http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
 * @param M 输入旋转矩阵（16个float）
 * @param q 输出四元数
 */
void quaternionU3DFromMatrix(float M[],Quaternion &q);

/**
 * 准备AR对象的模型矩阵
 * 包含坐标系转换和平移缩放
 * @param inM 输入基础矩阵（16个float）
 * @param outM 输出模型矩阵（16个float）
 * @param size 对象缩放大小
 * @param x X轴偏移（默认0）
 * @param y Y轴偏移（默认0）
 * @param z Z轴偏移（默认0）
 */
void prepareModelM(float inM[],float outM[],const float size, const float x=0, const float y=0, const float z=0);

/**
 * 打印矩阵到日志（调试用）
 * @param f 要打印的矩阵（16个float）
 */
void logMatrix(float f[]);

#endif //ORB_SLAM2_AR_MATRIX_H

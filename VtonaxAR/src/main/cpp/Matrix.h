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

        if (rows == 3)
        {
            const float
                a00 = data[0], a01 = data[1], a02 = data[2],
                a10 = data[3], a11 = data[4], a12 = data[5],
                a20 = data[6], a21 = data[7], a22 = data[8];

            const float det = a00*(a11*a22 - a12*a21)
                            - a01*(a10*a22 - a12*a20)
                            + a02*(a10*a21 - a11*a20);

            if (std::fabs(det) > 1e-8f)
            {
                const float invDet = 1.0f / det;
                Mat r(3, 3);
                r.data[0] =  (a11*a22 - a12*a21) * invDet;
                r.data[1] = -(a01*a22 - a02*a21) * invDet;
                r.data[2] =  (a01*a12 - a02*a11) * invDet;
                r.data[3] = -(a10*a22 - a12*a20) * invDet;
                r.data[4] =  (a00*a22 - a02*a20) * invDet;
                r.data[5] = -(a00*a12 - a02*a10) * invDet;
                r.data[6] =  (a10*a21 - a11*a20) * invDet;
                r.data[7] = -(a00*a21 - a01*a20) * invDet;
                r.data[8] =  (a00*a11 - a01*a10) * invDet;
                return r;
            }
        }

        if (rows == 4)
        {
            const float* m = data.data();
            const float
                src00 = m[0],  src10 = m[1],  src20 = m[2],  src30 = m[3],
                src01 = m[4],  src11 = m[5],  src21 = m[6],  src31 = m[7],
                src02 = m[8],  src12 = m[9],  src22 = m[10], src32 = m[11],
                src03 = m[12], src13 = m[13], src23 = m[14], src33 = m[15];

            const float
                atmp0  = src22*src33,  atmp1  = src23*src32,
                atmp2  = src21*src33,  atmp3  = src23*src31,
                atmp4  = src21*src32,  atmp5  = src22*src31,
                atmp6  = src20*src33,  atmp7  = src23*src30,
                atmp8  = src20*src32,  atmp9  = src22*src30,
                atmp10 = src20*src31,  atmp11 = src21*src30;

            const float
                dst0 = (atmp0*src11 + atmp3*src12 + atmp4*src13) - (atmp1*src11 + atmp2*src12 + atmp5*src13),
                dst1 = (atmp1*src10 + atmp6*src12 + atmp9*src13) - (atmp0*src10 + atmp7*src12 + atmp8*src13),
                dst2 = (atmp2*src10 + atmp7*src11 + atmp10*src13) - (atmp3*src10 + atmp6*src11 + atmp11*src13),
                dst3 = (atmp5*src10 + atmp8*src11 + atmp11*src12) - (atmp4*src10 + atmp9*src11 + atmp10*src12),
                dst4 = (atmp1*src01 + atmp2*src02 + atmp5*src03) - (atmp0*src01 + atmp3*src02 + atmp4*src03),
                dst5 = (atmp0*src00 + atmp7*src02 + atmp8*src03) - (atmp1*src00 + atmp6*src02 + atmp9*src03),
                dst6 = (atmp3*src00 + atmp6*src01 + atmp11*src03) - (atmp2*src00 + atmp7*src01 + atmp10*src03),
                dst7 = (atmp4*src00 + atmp9*src01 + atmp10*src02) - (atmp5*src00 + atmp8*src01 + atmp11*src02);

            const float
                btmp0 = src02*src13,  btmp1 = src03*src12,
                btmp2 = src01*src13,  btmp3 = src03*src11,
                btmp4 = src01*src12,  btmp5 = src02*src11,
                btmp6 = src00*src13,  btmp7 = src03*src10,
                btmp8 = src00*src12,  btmp9 = src02*src10,
                btmp10 = src00*src11, btmp11 = src01*src10;

            const float
                dst8  = (btmp0*src31 + btmp3*src32 + btmp4*src33) - (btmp1*src31 + btmp2*src32 + btmp5*src33),
                dst9  = (btmp1*src30 + btmp6*src32 + btmp9*src33) - (btmp0*src30 + btmp7*src32 + btmp8*src33),
                dst10 = (btmp2*src30 + btmp7*src31 + btmp10*src33) - (btmp3*src30 + btmp6*src31 + btmp11*src33),
                dst11 = (btmp5*src30 + btmp8*src31 + btmp11*src32) - (btmp4*src30 + btmp9*src31 + btmp10*src32),
                dst12 = (btmp1*src21 + btmp2*src22 + btmp5*src23) - (btmp0*src21 + btmp3*src22 + btmp4*src23),
                dst13 = (btmp0*src20 + btmp7*src22 + btmp8*src23) - (btmp1*src20 + btmp6*src22 + btmp9*src23),
                dst14 = (btmp3*src20 + btmp6*src21 + btmp11*src23) - (btmp2*src20 + btmp7*src21 + btmp10*src23),
                dst15 = (btmp4*src20 + btmp9*src21 + btmp10*src22) - (btmp5*src20 + btmp8*src21 + btmp11*src22);

            const float det = src00*dst0 + src01*dst1 + src02*dst2 + src03*dst3;

            if (std::fabs(det) > 1e-8f)
            {
                const float invDet = 1.0f / det;
                Mat r(4, 4);
                r.data[0]  = dst0*invDet;  r.data[1]  = dst4*invDet;
                r.data[2]  = dst8*invDet;  r.data[3]  = dst12*invDet;
                r.data[4]  = dst1*invDet;  r.data[5]  = dst5*invDet;
                r.data[6]  = dst9*invDet;  r.data[7]  = dst13*invDet;
                r.data[8]  = dst2*invDet;  r.data[9]  = dst6*invDet;
                r.data[10] = dst10*invDet; r.data[11] = dst14*invDet;
                r.data[12] = dst3*invDet;  r.data[13] = dst7*invDet;
                r.data[14] = dst11*invDet; r.data[15] = dst15*invDet;
                return r;
            }
        }

        int n = rows;
        Mat a(*this);
        Mat inv = Mat::identity(n);

        for (int i = 0; i < n; ++i)
        {
            float pivot = a(i, i);
            if (std::abs(pivot) < 1e-8f)
                pivot = 1e-8f;
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

#endif //ORB_SLAM2_AR_MATRIX_H

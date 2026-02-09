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
    for (int i=0 ; i<4 ; i++) {
        const float rhs_i0 = rhs[ I(i,0) ];
        float ri0 = lhs[ I(0,0) ] * rhs_i0;
        float ri1 = lhs[ I(0,1) ] * rhs_i0;
        float ri2 = lhs[ I(0,2) ] * rhs_i0;
        float ri3 = lhs[ I(0,3) ] * rhs_i0;
        for (int j=1 ; j<4 ; j++) {
            const float rhs_ij = rhs[ I(i,j) ];
            ri0 += lhs[ I(j,0) ] * rhs_ij;
            ri1 += lhs[ I(j,1) ] * rhs_ij;
            ri2 += lhs[ I(j,2) ] * rhs_ij;
            ri3 += lhs[ I(j,3) ] * rhs_ij;
        }
        r[ I(i,0) ] = ri0;
        r[ I(i,1) ] = ri1;
        r[ I(i,2) ] = ri2;
        r[ I(i,3) ] = ri3;
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
    multiplyMM(rm, m, sTemp);
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

/**
 * 4x4矩阵求逆（使用Cramer法则）
 * 
 * 注意事项:
 *   - mInv和m不能重叠
 *   - 如果矩阵奇异（行列式为0），无法求逆
 * 
 * @param mInv 输出逆矩阵数组
 * @param mInvOffset 逆矩阵在数组中的偏移
 * @param m 输入矩阵数组
 * @param mOffset 输入矩阵在数组中的偏移
 * @return true-成功求逆, false-矩阵奇异无法求逆
 */
bool invertM(float mInv[], int mInvOffset, float m[],
                              int mOffset) {
    // 使用Cramer法则求逆（计算伴随矩阵/行列式）

    // 转置矩阵（列主序转行主序）
    float src0  = m[mOffset +  0];
    float src4  = m[mOffset +  1];
    float src8  = m[mOffset +  2];
    float src12 = m[mOffset +  3];

    float src1  = m[mOffset +  4];
    float src5  = m[mOffset +  5];
    float src9  = m[mOffset +  6];
    float src13 = m[mOffset +  7];

    float src2  = m[mOffset +  8];
    float src6  = m[mOffset +  9];
    float src10 = m[mOffset + 10];
    float src14 = m[mOffset + 11];

    float src3  = m[mOffset + 12];
    float src7  = m[mOffset + 13];
    float src11 = m[mOffset + 14];
    float src15 = m[mOffset + 15];

    // 计算前8个元素的中间值（余子式对）
    float atmp0  = src10 * src15;
    float atmp1  = src11 * src14;
    float atmp2  = src9  * src15;
    float atmp3  = src11 * src13;
    float atmp4  = src9  * src14;
    float atmp5  = src10 * src13;
    float atmp6  = src8  * src15;
    float atmp7  = src11 * src12;
    float atmp8  = src8  * src14;
    float atmp9  = src10 * src12;
    float atmp10 = src8  * src13;
    float atmp11 = src9  * src12;

    // 计算前8个余子式
    float dst0  = (atmp0 * src5 + atmp3 * src6 + atmp4  * src7)
                  - (atmp1 * src5 + atmp2 * src6 + atmp5  * src7);
    float dst1  = (atmp1 * src4 + atmp6 * src6 + atmp9  * src7)
                  - (atmp0 * src4 + atmp7 * src6 + atmp8  * src7);
    float dst2  = (atmp2 * src4 + atmp7 * src5 + atmp10 * src7)
                  - (atmp3 * src4 + atmp6 * src5 + atmp11 * src7);
    float dst3  = (atmp5 * src4 + atmp8 * src5 + atmp11 * src6)
                  - (atmp4 * src4 + atmp9 * src5 + atmp10 * src6);
    float dst4  = (atmp1 * src1 + atmp2 * src2 + atmp5  * src3)
                  - (atmp0 * src1 + atmp3 * src2 + atmp4  * src3);
    float dst5  = (atmp0 * src0 + atmp7 * src2 + atmp8  * src3)
                  - (atmp1 * src0 + atmp6 * src2 + atmp9  * src3);
    float dst6  = (atmp3 * src0 + atmp6 * src1 + atmp11 * src3)
                  - (atmp2 * src0 + atmp7 * src1 + atmp10 * src3);
    float dst7  = (atmp4 * src0 + atmp9 * src1 + atmp10 * src2)
                        - (atmp5 * src0 + atmp8 * src1 + atmp11 * src2);

    // 计算后8个元素的中间值（余子式对）
    float btmp0  = src2 * src7;
    float btmp1  = src3 * src6;
    float btmp2  = src1 * src7;
    float btmp3  = src3 * src5;
    float btmp4  = src1 * src6;
    float btmp5  = src2 * src5;
    float btmp6  = src0 * src7;
    float btmp7  = src3 * src4;
    float btmp8  = src0 * src6;
    float btmp9  = src2 * src4;
    float btmp10 = src0 * src5;
    float btmp11 = src1 * src4;

    // 计算后8个元素（余因子）
    float dst8  = (btmp0  * src13 + btmp3  * src14 + btmp4  * src15)
                  - (btmp1  * src13 + btmp2  * src14 + btmp5  * src15);
    float dst9  = (btmp1  * src12 + btmp6  * src14 + btmp9  * src15)
                  - (btmp0  * src12 + btmp7  * src14 + btmp8  * src15);
    float dst10 = (btmp2  * src12 + btmp7  * src13 + btmp10 * src15)
                  - (btmp3  * src12 + btmp6  * src13 + btmp11 * src15);
    float dst11 = (btmp5  * src12 + btmp8  * src13 + btmp11 * src14)
                  - (btmp4  * src12 + btmp9  * src13 + btmp10 * src14);
    float dst12 = (btmp2  * src10 + btmp5  * src11 + btmp1  * src9 )
                  - (btmp4  * src11 + btmp0  * src9  + btmp3  * src10);
    float dst13 = (btmp8  * src11 + btmp0  * src8  + btmp7  * src10)
                  - (btmp6  * src10 + btmp9  * src11 + btmp1  * src8 );
    float dst14 = (btmp6  * src9  + btmp11 * src11 + btmp3  * src8 )
                  - (btmp10 * src11 + btmp2  * src8  + btmp7  * src9 );
    float dst15 = (btmp10 * src10 + btmp4  * src8  + btmp9  * src9 )
                        - (btmp8  * src9  + btmp11 * src10 + btmp5  * src8 );

    // 计算行列式
    float det =
            src0 * dst0 + src1 * dst1 + src2 * dst2 + src3 * dst3;

    if (det == 0.0f) {
        return false;
    }

    // 预计算倒数，将除法转换为乘法
    // 除法比乘法慢3-10倍
    float invdet = 1.0f / det;
    
    // 使用乘法替代除法
    mInv[     mInvOffset] = dst0  * invdet;
    mInv[ 1 + mInvOffset] = dst1  * invdet;
    mInv[ 2 + mInvOffset] = dst2  * invdet;
    mInv[ 3 + mInvOffset] = dst3  * invdet;

    mInv[ 4 + mInvOffset] = dst4  * invdet;
    mInv[ 5 + mInvOffset] = dst5  * invdet;
    mInv[ 6 + mInvOffset] = dst6  * invdet;
    mInv[ 7 + mInvOffset] = dst7  * invdet;

    mInv[ 8 + mInvOffset] = dst8  * invdet;
    mInv[ 9 + mInvOffset] = dst9  * invdet;
    mInv[10 + mInvOffset] = dst10 * invdet;
    mInv[11 + mInvOffset] = dst11 * invdet;

    mInv[12 + mInvOffset] = dst12 * invdet;
    mInv[13 + mInvOffset] = dst13 * invdet;
    mInv[14 + mInvOffset] = dst14 * invdet;
    mInv[15 + mInvOffset] = dst15 * invdet;

    return true;
}

/**
 * 刚体变换矩阵快速求逆 (列主序)
 * 
 * 针对变换矩阵 [R | t; 0 | 1] 的特殊化求逆：
 *   [R | t]^-1 = [R^T | -R^T * t]
 *   [0 | 1]      [0   |    1    ]
 * 
 * 相比通用 invertM: 从 ~80 次乘法 + 16 次加法 + 1 次除法
 *                   减少到 18 次乘法 + 6 次加法，无除法
 * 
 * @param mInv 输出逆矩阵 (列主序)
 * @param m 输入变换矩阵 (列主序)
 * @return 始终返回 true（刚体变换矩阵始终可逆）
 */
bool invertTransformM(float mInv[], const float m[])
{
    // 列主序索引: m[col*4 + row]
    // m[0-3]:   R第0列 + m[3]=0
    // m[4-7]:   R第1列 + m[7]=0
    // m[8-11]:  R第2列 + m[11]=0
    // m[12-15]: t向量  + m[15]=1
    
    // R^T: 转置旋转矩阵
    mInv[0]  = m[0];   mInv[1]  = m[4];   mInv[2]  = m[8];   mInv[3]  = 0.0f;
    mInv[4]  = m[1];   mInv[5]  = m[5];   mInv[6]  = m[9];   mInv[7]  = 0.0f;
    mInv[8]  = m[2];   mInv[9]  = m[6];   mInv[10] = m[10];  mInv[11] = 0.0f;
    
    // -R^T * t
    const float tx = m[12], ty = m[13], tz = m[14];
    mInv[12] = -(mInv[0] * tx + mInv[4] * ty + mInv[8]  * tz);
    mInv[13] = -(mInv[1] * tx + mInv[5] * ty + mInv[9]  * tz);
    mInv[14] = -(mInv[2] * tx + mInv[6] * ty + mInv[10] * tz);
    mInv[15] = 1.0f;
    
    return true;
}


void getRUBViewMatrixFromRDF(float inM[],float outM[]){
    // View_GL = Rx(180) * View_CV * Rx(180)
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
    // Overlap indices: 5, 9, 6, 10.
    
    // 最终需要取反的索引列表：
    // Row only: 1, 13, 2, 14
    // Col only: 4, 7, 8, 11
    
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
    float tmpM[16],tmpVM[16];
    setIdentityM(tmpM);
    rotateM(tmpVM,tmpM, +180.0f, 1.0f, 0.0f, 0.0f);
    multiplyMM(outM,tmpVM,inM);
}


void getEulerAnglesFromMatrix(float M[],float outM[],int offset){
    outM[offset+0] =180.0f/PI*atan2(M[1], M[5]);
    outM[offset+1] =180.0f/PI*asin(-M[9]);
    outM[offset+2] =180.0f/PI*atan2(-M[8], M[10]);
}

void quaternionToMatrix(float M[],Quaternion &q){
    float m00;
    float m10;
    float m20;

    float m01;
    float m11;
    float m21;

    float m02;
    float m12;
    float m22;

    float sqw = q.w*q.w;
    float sqx = q.x*q.x;
    float sqy = q.y*q.y;
    float sqz = q.z*q.z;

    // 预计算倒数，将除法转换为乘法
    // invs（逆平方长度）仅在四元数未归一化时需要
    float invs = 1.0f / (sqx + sqy + sqz + sqw);
    m00 = ( sqx - sqy - sqz + sqw)*invs ; // 因为 sqw + sqx + sqy + sqz =1/invs*invs
    m11 = (-sqx + sqy - sqz + sqw)*invs ;
    m22 = (-sqx - sqy + sqz + sqw)*invs ;

    float tmp1 = q.x*q.y;
    float tmp2 = q.z*q.w;
    m10 = 2.0f * (tmp1 + tmp2)*invs ;
    m01 = 2.0f * (tmp1 - tmp2)*invs ;

    tmp1 = q.x*q.z;
    tmp2 = q.y*q.w;
    m20 = 2.0f * (tmp1 - tmp2)*invs ;
    m02 = 2.0f * (tmp1 + tmp2)*invs ;
    tmp1 = q.y*q.z;
    tmp2 = q.x*q.w;
    m21 = 2.0f * (tmp1 + tmp2)*invs ;
    m12 = 2.0f * (tmp1 - tmp2)*invs ;

    M[0]=m00;
    M[1]=m10;
    M[2]=m20;
    M[4]=m01;
    M[5]=m11;
    M[6]=m21;
    M[8]=m02;
    M[9]=m12;
    M[10]=m22;
}

void matrixToQuaternion(float M[],Quaternion &q){
    float qx,qy,qz,qw;
    float m00=M[0];
    float m10=M[1];
    float m20=M[2];
    float m01=M[4];
    float m11=M[5];
    float m21=M[6];
    float m02=M[8];
    float m12=M[9];
    float m22=M[10];

    float tr = m00 + m11 + m22;

    if (tr > 0) {
        float S = (float) (sqrt(tr+1) * 2); // S=4*qw
        qw = 0.25f * S;
        qx = (m21 - m12) / S;
        qy = (m02 - m20) / S;
        qz = (m10 - m01) / S;
    } else if ((m00 > m11)&(m00 > m22)) {
        float S = (float) (sqrt(1.0 + m00 - m11 - m22) * 2); // S=4*qx
        qw = (m21 - m12) / S;
        qx = 0.25f * S;
        qy = (m01 + m10) / S;
        qz = (m02 + m20) / S;
    } else if (m11 > m22) {
        float S = (float) (sqrt(1.0 + m11 - m00 - m22) * 2); // S=4*qy
        qw = (m02 - m20) / S;
        qx = (m01 + m10) / S;
        qy = 0.25f * S;
        qz = (m12 + m21) / S;
    } else {
        float S = (float) (sqrt(1.0 + m22 - m00 - m11) * 2); // S=4*qz
        qw = (m10 - m01) / S;
        qx = (m02 + m20) / S;
        qy = (m12 + m21) / S;
        qz = 0.25f * S;
    }
    q.x=qx;
    q.y=qy;
    q.z=qz;
    q.w=qw;
}

inline float sign(float f){
    if(f<0) return -1.0f;
    return 1.0f;
}

void quaternionU3DFromMatrix(float M[],Quaternion &q)
{
    // 改编自: http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
    float m00=M[0],m10=M[1],m20=M[2];
    float m01=M[4],m11=M[5],m21=M[6];
    float m02=M[8],m12=M[9],m22=M[10];
    q.w = sqrt(std::max(0.0f, 1 + m00 + m11 + m22)) / 2;
    q.x = sqrt(std::max(0.0f, 1 + m00 - m11 - m22)) / 2;
    q.y = sqrt(std::max(0.0f, 1 - m00 + m11 - m22)) / 2;
    q.z = sqrt(std::max(0.0f, 1 - m00 - m11 + m22)) / 2;
    q.x *= sign(q.x * (m21 - m12));
    q.y *= sign(q.y * (m02 - m20));
    q.z *= sign(q.z * (m10 - m01));
}

void prepareModelM(float inM[],float outM[],const float size, const float x, const float y, const float z){
    float M[16],tmpM[16];
    setIdentityM(M);
    rotateM(tmpM,M,180,1,0,0);
    tmpM[12] = -x;
    tmpM[13] = -size-y;
    tmpM[14] = -z;
    multiplyMM(outM,inM,tmpM);
}

void logMatrix(float f[]){
    LOGD("矩阵为:");
    std::string str;
    char buf[255];
    for(int i=0;i<4;i++){
        str="";
        for(int j=i;j<16;j+=4){
            sprintf(buf," %.3lf ",f[j]);
            str+=buf;
        }
        LOGD("%s",str.c_str());
    }
}

void quatRotateVector(const Quaternion &q, const float in[3], float out[3]){
    float ux=q.x, uy=q.y, uz=q.z, s=q.w;
    float dot_u_u = ux*ux + uy*uy + uz*uz;
    float dot_u_v = ux*in[0] + uy*in[1] + uz*in[2];
    float cx = uy*in[2] - uz*in[1];
    float cy = uz*in[0] - ux*in[2];
    float cz = ux*in[1] - uy*in[0];
    out[0] = 2.0f*dot_u_v*ux + (s*s - dot_u_u)*in[0] + 2.0f*s*cx;
    out[1] = 2.0f*dot_u_v*uy + (s*s - dot_u_u)*in[1] + 2.0f*s*cy;
    out[2] = 2.0f*dot_u_v*uz + (s*s - dot_u_u)*in[2] + 2.0f*s*cz;
}

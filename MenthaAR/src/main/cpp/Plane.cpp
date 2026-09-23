/**
 * Created by Ads on 2017/1/15.
 * 由Olsc于2025/8/25开始进行修改
 */

// 平面检测与表示模块实现

#include "Plane.h"
#include "Matrix.h"
#include "UIUtils.h"

// 3x3 矩阵直接乘法 C = A * B，避免分配中间临时对象及通用 GEMM 开销
static cv::Mat Mat33Mul(const cv::Mat& A, const cv::Mat& B)
{
    cv::Mat C(3, 3, CV_32F);
    const float* a = A.ptr<float>();
    const float* b = B.ptr<float>();
    float* c = C.ptr<float>();

    for (int i = 0; i < 3; ++i) {
        const float a0 = a[i * 3 + 0];
        const float a1 = a[i * 3 + 1];
        const float a2 = a[i * 3 + 2];
        c[i * 3 + 0] = a0 * b[0] + a1 * b[3] + a2 * b[6];
        c[i * 3 + 1] = a0 * b[1] + a1 * b[4] + a2 * b[7];
        c[i * 3 + 2] = a0 * b[2] + a1 * b[5] + a2 * b[8];
    }
    return C;
}

// SO(3)李代数指数映射：用闭式Rodrigues公式直接填充3x3矩阵，彻底杜绝堆内存临时开辟与动态析构
// exp([w]×) = I·cos(θ) + (1-cos(θ))·k·k^T + [k]×·sin(θ)，θ=||w||, k=w/θ
cv::Mat Plane::ExpSO3(const float& x, const float& y, const float& z)
{
    cv::Mat R(3, 3, CV_32F);
    float* r = R.ptr<float>();
    const float d2 = x * x + y * y + z * z;
    const float d = std::sqrt(d2);

    // 当旋转角度很小时，使用泰勒级数前两项解析展开：I + W + 0.5*W*W
    if (d < 1e-4f)
    {
        r[0] = 1.0f - 0.5f * (y * y + z * z);
        r[1] = -z + 0.5f * x * y;
        r[2] =  y + 0.5f * x * z;

        r[3] =  z + 0.5f * x * y;
        r[4] = 1.0f - 0.5f * (x * x + z * z);
        r[5] = -x + 0.5f * y * z;

        r[6] = -y + 0.5f * x * z;
        r[7] =  x + 0.5f * y * z;
        r[8] = 1.0f - 0.5f * (x * x + y * y);
    }
    else
    {
        const float inv_d = 1.0f / d;
        const float kx = x * inv_d, ky = y * inv_d, kz = z * inv_d;
        const float c = std::cos(d), s = std::sin(d), C = 1.0f - c;

        r[0] = c + kx * kx * C;
        r[1] = kx * ky * C - kz * s;
        r[2] = kx * kz * C + ky * s;

        r[3] = ky * kx * C + kz * s;
        r[4] = c + ky * ky * C;
        r[5] = ky * kz * C - kx * s;

        r[6] = kz * kx * C - ky * s;
        r[7] = kz * ky * C + kx * s;
        r[8] = c + kz * kz * C;
    }
    return R;
}

// SO(3)李代数指数映射的向量版本
cv::Mat Plane::ExpSO3(const cv::Mat& v)
{
    return ExpSO3(v.at<float>(0), v.at<float>(1), v.at<float>(2));
}

// 从地图点集合构造平面，初始化平面参数并计算变换矩阵
Plane::Plane(const std::vector<ORB_SLAM2::MapPoint*>& vMPs, const cv::Mat& Tcw) :
    mvMPs(vMPs), mTcw(Tcw.clone())
{
    // 随机初始化平面绕法向量的旋转角度（-π/2 到 π/2）
    rang = -3.14159265358979323846f / 2 +
           static_cast<float>(static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) *
           3.14159265358979323846f;

    Recompute();  // 计算平面参数
}

// 从法向量和原点直接构造平面，无需重新计算
Plane::Plane(const float& nx, const float& ny, const float& nz,
             const float& ox, const float& oy, const float& oz,
             const float& rang_)
{
    // 设置平面法向量和原点
    n = (cv::Mat_<float>(3, 1) << nx, ny, nz);
    o = (cv::Mat_<float>(3, 1) << ox, oy, oz);
    rang = rang_;

    // 计算从世界坐标系到平面坐标系的变换矩阵
    // 使平面的Y轴与世界的Y轴对齐 (up = [0, 1, 0]^T)
    // v = up.cross(n) = [nz, 0, -nx]^T
    const float vx = nz, vy = 0.0f, vz = -nx;
    const float sa = std::sqrt(vx * vx + vz * vz); // sin(angle)
    const float ca = ny;                           // cos(angle) = up.dot(n)
    const float ang = std::atan2(sa, ca);          // 旋转角度

    Tpw = cv::Mat::eye(4, 4, CV_32F);

    if (sa > 1e-6f)
    {  // 法向量不平行于up向量
        // 组合两个旋转：先旋转到up方向，再绕法向量旋转rang角度
        const float factor = ang / sa;
        cv::Mat R1 = ExpSO3(vx * factor, vy * factor, vz * factor);
        cv::Mat R2 = ExpSO3(0.0f, rang, 0.0f);
        Tpw.rowRange(0, 3).colRange(0, 3) = Mat33Mul(R1, R2);
    }
    else
    {  // 法向量平行于up向量，绕up旋转rang角度
        Tpw.rowRange(0, 3).colRange(0, 3) = ExpSO3(0.0f, rang, 0.0f);
    }

    o.copyTo(Tpw.col(3).rowRange(0, 3));  // 设置平移部分

    // 转换为OpenGL格式的列主序矩阵
    setIdentityM(glTpw);
    getColMajorMatrixFromMat(glTpw, Tpw);
}

// 根据地图点用SVD分解重新计算平面参数：拟合法向量、计算质心原点并确保法向量朝向相机
void Plane::Recompute()
{
    const int N = static_cast<int>(mvMPs.size());

    // 收集有效点并单次遍历计算质心
    std::vector<cv::Point3f> validPoints;
    validPoints.reserve(N);
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;

    for (int i = 0; i < N; i++)
    {
        ORB_SLAM2::MapPoint* pMP = mvMPs[i];
        if (pMP && !pMP->isBad())
        {
            cv::Point3f Xw;
            pMP->GetWorldPos(Xw);
            sumX += Xw.x;
            sumY += Xw.y;
            sumZ += Xw.z;
            validPoints.push_back(Xw);
        }
    }

    const int nPoints = static_cast<int>(validPoints.size());
    if (nPoints < 3) return;

    const float invN = 1.0f / nPoints;
    const float meanX = sumX * invN;
    const float meanY = sumY * invN;
    const float meanZ = sumZ * invN;

    o = (cv::Mat_<float>(3, 1) << meanX, meanY, meanZ);

    // 闭式 PCA：累加 3x3 协方差矩阵
    float cxx = 0.0f, cxy = 0.0f, cxz = 0.0f;
    float cyy = 0.0f, cyz = 0.0f, czz = 0.0f;
    for (int i = 0; i < nPoints; i++)
    {
        const float dx = validPoints[i].x - meanX;
        const float dy = validPoints[i].y - meanY;
        const float dz = validPoints[i].z - meanZ;
        cxx += dx * dx; cxy += dx * dy; cxz += dx * dz;
        cyy += dy * dy; cyz += dy * dz; czz += dz * dz;
    }

    cv::Mat Cov = (cv::Mat_<float>(3, 3) << cxx, cxy, cxz,
                                            cxy, cyy, cyz,
                                            cxz, cyz, czz);
    cv::Mat eval, evec;
    // cv::eigen 按特征值降序返回，平面法向量严格等于最小特征值对应特征向量 (第2行)
    cv::eigen(Cov, eval, evec);

    float a = evec.at<float>(2, 0);
    float b = evec.at<float>(2, 1);
    float c = evec.at<float>(2, 2);

    const float f = 1.0f / std::sqrt(a * a + b * b + c * c);

    // 首次计算时，计算从相机中心指向平面原点的向量
    // 用于确定法向量方向：Oc = -Rcw^T * tcw
    if (XC.empty())
    {
        const float* tcw_ptr = mTcw.ptr<float>();
        const float r00 = tcw_ptr[0], r01 = tcw_ptr[1], r02 = tcw_ptr[2], t0 = tcw_ptr[3];
        const float r10 = tcw_ptr[4], r11 = tcw_ptr[5], r12 = tcw_ptr[6], t1 = tcw_ptr[7];
        const float r20 = tcw_ptr[8], r21 = tcw_ptr[9], r22 = tcw_ptr[10], t2 = tcw_ptr[11];

        const float ocX = -(r00 * t0 + r10 * t1 + r20 * t2);
        const float ocY = -(r01 * t0 + r11 * t1 + r21 * t2);
        const float ocZ = -(r02 * t0 + r12 * t1 + r22 * t2);

        XC = (cv::Mat_<float>(3, 1) << ocX - meanX, ocY - meanY, ocZ - meanZ);
    }

    // 确保法向量指向相机侧（点积>0则反向）
    if ((XC.at<float>(0) * a + XC.at<float>(1) * b + XC.at<float>(2) * c) > 0)
    {
        a = -a;
        b = -b;
        c = -c;
    }

    // 归一化法向量
    const float nx = a * f;
    const float ny = b * f;
    const float nz = c * f;

    n = (cv::Mat_<float>(3, 1) << nx, ny, nz);

    // 计算从世界坐标系到平面坐标系的变换矩阵
    // 使平面的Y轴与世界的Y轴对齐 (up = [0, 1, 0]^T)
    // v = up.cross(n) = [nz, 0, -nx]^T
    const float vx = nz, vy = 0.0f, vz = -nx;
    const float sa = std::sqrt(vx * vx + vz * vz); // sin(angle)
    const float ca = ny;                           // cos(angle) = up.dot(n)
    const float ang = std::atan2(sa, ca);          // 旋转角度

    Tpw = cv::Mat::eye(4, 4, CV_32F);

    // 组合旋转：先将法向量旋转到up方向，再绕法向量旋转
    if (sa > 1e-6f)
    {  // 法向量不平行于up向量
        const float factor = ang / sa;
        cv::Mat R1 = ExpSO3(vx * factor, vy * factor, vz * factor);
        cv::Mat R2 = ExpSO3(0.0f, rang, 0.0f);
        Tpw.rowRange(0, 3).colRange(0, 3) = Mat33Mul(R1, R2);
    }
    else
    {  // 法向量平行于up向量（如水平地面），绕up旋转rang角度
        Tpw.rowRange(0, 3).colRange(0, 3) = ExpSO3(0.0f, rang, 0.0f);
    }
    o.copyTo(Tpw.col(3).rowRange(0, 3));  // 设置平移部分

    // 转换为OpenGL格式
    setIdentityM(glTpw);
    getColMajorMatrixFromMat(glTpw, Tpw);
}
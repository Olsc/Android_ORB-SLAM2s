/**
 * Created by Ads on 2017/1/15.
 * 由Olsc于2025/8/25开始进行修改
 */

// 平面检测与表示模块实现

#include "Plane.h"
#include "Matrix.h"
#include "UIUtils.h"

// SO(3)李代数指数映射：用Rodrigues公式将旋转向量转为旋转矩阵
// exp([w]×) = I + sin(θ)/θ·[w]× + (1-cos(θ))/θ²·[w]×²，θ=||w||
cv::Mat Plane::ExpSO3(const float& x, const float& y, const float& z)
{
    cv::Mat I = cv::Mat::eye(3, 3, CV_32F);  // 单位矩阵
    const float d2 = x * x + y * y + z * z;  // θ²
    const float d = sqrt(d2);                // θ = ||w||

    // 反对称矩阵 [w]×
    cv::Mat W = (cv::Mat_<float>(3, 3) << 0, -z, y,
                                          z,  0, -x,
                                         -y,  x,  0);

    // 当旋转角度很小时，使用泰勒展开的前两项
    if (d < 1e-4)
    {
        return (I + W + 0.5f * W * W);
    }
    else
    {
        return (I + W * sin(d) / d + W * W * (1.0f - cos(d)) / d2);
    }
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
             const float& ox, const float& oy, const float& oz)
{
    // 设置平面法向量和原点
    n = (cv::Mat_<float>(3, 1) << nx, ny, nz);
    o = (cv::Mat_<float>(3, 1) << ox, oy, oz);
    rang = 0.0f;

    // 计算从世界坐标系到平面坐标系的变换矩阵
    // 使平面的Y轴与世界的Y轴对齐
    cv::Mat up = (cv::Mat_<float>(3, 1) << 0.0f, 1.0f, 0.0f);
    cv::Mat v = up.cross(n);  // 旋转轴
    const float sa = cv::norm(v);   // sin(angle)
    const float ca = up.dot(n);     // cos(angle)
    const float ang = atan2(sa, ca); // 旋转角度

    Tpw = cv::Mat::eye(4, 4, CV_32F);

    if (sa > 1e-6)
    {  // 法向量不平行于up向量
        // 组合两个旋转：先旋转到up方向，再绕法向量旋转rang角度
        Tpw.rowRange(0, 3).colRange(0, 3) = ExpSO3(v * ang / sa) * ExpSO3(up * rang);
    }
    else
    {  // 法向量平行于up向量，不需要旋转
        Tpw.rowRange(0, 3).colRange(0, 3) = cv::Mat::eye(3, 3, CV_32F);
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

    // 准备SVD分解所需的矩阵（N个点，每行[x,y,z,1]）
    cv::Mat A = cv::Mat(N, 4, CV_32F);
    A.col(3) = cv::Mat::ones(N, 1, CV_32F);

    o = cv::Mat::zeros(3, 1, CV_32F);

    int nPoints = 0;
    for (int i = 0; i < N; i++)
    {
        ORB_SLAM2::MapPoint* pMP = mvMPs[i];

        if (!pMP->isBad())
        {
            // 栈版读取
            cv::Point3f Xw;
            pMP->GetWorldPos(Xw);
            o.at<float>(0) += Xw.x;  // 累加坐标（后续计算质心）
            o.at<float>(1) += Xw.y;
            o.at<float>(2) += Xw.z;
            A.at<float>(nPoints,0) = Xw.x;
            A.at<float>(nPoints,1) = Xw.y;
            A.at<float>(nPoints,2) = Xw.z;
            nPoints++;
        }
    }

    A.resize(nPoints);  // 调整矩阵大小为实际点数

    // SVD分解：平面法向量是最小奇异值对应的右奇异向量
    cv::Mat u, w, vt;
    cv::SVDecomp(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    float a = vt.at<float>(3, 0);  // 平面方程系数
    float b = vt.at<float>(3, 1);
    float c = vt.at<float>(3, 2);

    o = o * (1.0f / nPoints);  // 计算质心作为平面原点
    const float f = 1.0f / sqrt(a * a + b * b + c * c);  // 归一化系数

    // 首次计算时，计算从相机中心指向平面原点的向量
    // 用于确定法向量方向
    if (XC.empty())
    {
        cv::Mat Oc = -mTcw.colRange(0, 3).rowRange(0, 3).t() * mTcw.rowRange(0, 3).col(3);
        XC = Oc - o;
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
    cv::Mat up = (cv::Mat_<float>(3, 1) << 0.0f, 1.0f, 0.0f);

    cv::Mat v = up.cross(n);  // 旋转轴
    const float sa = cv::norm(v);
    const float ca = up.dot(n);
    const float ang = atan2(sa, ca);

    Tpw = cv::Mat::eye(4, 4, CV_32F);

    // 组合旋转：先将法向量旋转到up方向，再绕法向量旋转
    Tpw.rowRange(0, 3).colRange(0, 3) = ExpSO3(v * ang / sa) * ExpSO3(up * rang);
    o.copyTo(Tpw.col(3).rowRange(0, 3));  // 设置平移部分

    // 转换为OpenGL格式
    setIdentityM(glTpw);
    getColMajorMatrixFromMat(glTpw, Tpw);
    // transposeR(glTpw);
}
/**
* 本文件是 ORB-SLAM2 的一部分。
*
* 版权所有 (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es>（萨拉戈萨大学）
* 更多信息请参见 [https://github.com/raulmur/ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2)
*
* ORB-SLAM2 是自由软件：您可以依据自由软件基金会发布的 GNU 通用公共许可证（第3版，或您可选择的更高版本）来重新分发和修改本软件。
*
* ORB-SLAM2 的发布目的是希望它能发挥作用，
* 但**不附带任何保证**；甚至没有适销性或适用于特定用途的隐含保证。
* 更多细节请参阅 GNU 通用公共许可证。
*
* 您应该已经随 ORB-SLAM2 一同收到了 GNU 通用公共许可证的副本。
* 如果没有，请参见 [http://www.gnu.org/licenses/](http://www.gnu.org/licenses/)。
*
* 本项目由 Olsc 于 2025/8/25 开始进行修改，在原项目基础上增加了地图存储、读取和重定位等功能。随 ORB-SLAM2 一同采取 GNU 通用公共许可证（第3版）。
*/

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv/cv.h>


namespace ORB_SLAM2
{

class ExtractorNode
{
public:
    ExtractorNode():bNoMore(false){}

    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;
    cv::Point2i UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;
    bool bNoMore;
};

class ORBextractor
{
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);

    ~ORBextractor(){}

    // 计算图像上的ORB特征和描述子
    // ORB特征使用八叉树在图像上分散分布
    // 当前实现中忽略掩码
    void operator()( cv::InputArray image, cv::InputArray mask,
      std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors);

    int inline GetLevels(){
        return nlevels;}

    float inline GetScaleFactor(){
        return scaleFactor;}

    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }

    std::vector<cv::Mat> mvImagePyramid;

protected:

    void ComputePyramid(cv::Mat image);
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);    
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

    void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
    std::vector<cv::Point> pattern;

    int nfeatures;
    double scaleFactor;
    int nlevels;
    int iniThFAST;
    int minThFAST;

    std::vector<int> mnFeaturesPerLevel;

    std::vector<int> umax;

    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;    
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;
};

} //namespace ORB_SLAM2

#endif


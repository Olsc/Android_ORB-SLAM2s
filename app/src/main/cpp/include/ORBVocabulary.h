/**
* 本文件是 ORB-SLAM2 的一部分。
*
* 版权所有 (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es>（萨拉戈萨大学）
* 更多信息请参见 [https://github.com/raulmur/ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2)
*
* ORB-SLAM2 是自由软件：您可以依据自由软件基金会发布的 GNU 通用公共许可证（第3版，
* 或您可选择的更高版本）来重新分发和修改本软件。
*
* ORB-SLAM2 的发布目的是希望它能发挥作用，
* 但**不附带任何保证**；甚至没有适销性或适用于特定用途的隐含保证。
* 更多细节请参阅 GNU 通用公共许可证。
*
* 您应该已经随 ORB-SLAM2 一同收到了 GNU 通用公共许可证的副本。
* 如果没有，请参见 [http://www.gnu.org/licenses/](http://www.gnu.org/licenses/)。
*
* 本项目由 Olsc 于 2025/8/25 开始进行修改，在原项目基础上增加了地图存储、读取和重定位等功能。
*/

#ifndef ORBVOCABULARY_H
#define ORBVOCABULARY_H

#include"../Thirdparty/DBoW2/DBoW2/FORB.h"
#include"Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h"

namespace ORB_SLAM2
{

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
  ORBVocabulary;

} //namespace ORB_SLAM2

#endif // ORBVOCABULARY_H

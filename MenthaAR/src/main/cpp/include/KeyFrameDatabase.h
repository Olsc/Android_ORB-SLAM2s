/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

/*
 * This project is based on ORB-SLAM2.
 *
 * The ORB-SLAM2 project was ported to the Android platform by Ads
 * under the GitHub account Martin20150405 in 2017.
 *
 * Starting from August 25, 2025, Olsc began modifying this project.
 * On the basis of the original project, functions such as map saving,
 * map loading, and relocalization were added.
 *
 * This project is distributed under the GNU General Public License
 * version 3, together with ORB-SLAM2.
 */

#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>
#include <atomic>

#include "KeyFrame.h"
#include "Frame.h"
#include "HBSTTypes.h"

#include<mutex>

namespace ORB_SLAM2
{

class KeyFrame;
class Frame;

class KeyFrameDatabase
{
public:

    KeyFrameDatabase();

   void add(KeyFrame* pKF);

   void erase(KeyFrame* pKF);

   void clear();

   // 回环检测
   std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame* pKF, float minScore);

   // 重定位
   std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F);

   // 若存在待重建标记则重建
   void RebuildIfPending();

protected:

  // HBST 树
  HBSTTree* mpTree;

  // KeyFrame ID 到 KeyFrame* 的映射
  std::map<long unsigned int, KeyFrame*> mhmKeyFrames;

  // 延迟重建 HBST 树以消除被删除关键帧的物理残留
  void rebuild();

  // 自上次重建以来删除的关键帧数量
  int mnErasedCount;

  // 待重建标记：erase 达到阈值时置位，由 RebuildIfPending() 消费
  std::atomic<bool> mbRebuildPending{false};

  // 互斥锁
  std::mutex mMutex;
};

} //namespace ORB_SLAM2

#endif
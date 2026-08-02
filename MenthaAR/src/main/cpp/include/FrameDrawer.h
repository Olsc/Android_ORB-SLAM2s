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

#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "Tracking.h"
#include "MapPoint.h"
#include "Map.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include<mutex>


namespace ORB_SLAM2
{

class Tracking;

class FrameDrawer
{
public:
    FrameDrawer(Map* pMap);

    void SetMap(Map* pMap);

    // 更新最后处理帧的信息
    void Update(Tracking *pTracker);

protected:
    // 要绘制的帧信息
    int N;
    std::vector<cv::KeyPoint> mvCurrentKeys;
    std::vector<bool> mvbMap, mvbVO;
    bool mbOnlyTracking;
    std::vector<cv::KeyPoint> mvIniKeys;
    std::vector<int> mvIniMatches;
    int mState;

    Map* mpMap;

    std::mutex mMutex;
};

} //namespace ORB_SLAM2

#endif // FRAMEDRAWER_H

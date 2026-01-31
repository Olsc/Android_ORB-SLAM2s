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

#include "Map.h"
#include "Common.h"
#include "MapPoint.h"
#include "KeyFrame.h"

#include<mutex>
#include<vector>
#include<map>

namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
    // Debug: 跟踪加载地图标志的传播（避免在此处迭代现有集合）
    // if(pMP && pMP->mbFromLoadedMap){
    //     LOGD("Map::AddMapPoint loaded=1, p=%p, totalAll=%d", (void*)pMP, (int)mspMapPoints.size());
    // }
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    // if(pMP){
    //     LOGD("Map::EraseMapPoint p=%p loaded=%d bad=%d", (void*)pMP, (int)pMP->mbFromLoadedMap, (int)pMP->isBad());
    // }
    mspMapPoints.erase(pMP);
    
    // 将废弃的 MapPoint 放入回收站，等待 Map::clear() 统一释放
    mspMapPointsTrash.insert(pMP);
    
    // 不要立即删除，因为后台线程或其他关键帧可能持有引用
    // delete pMP;

}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // 将废弃的 KeyFrame 放入回收站，等待 Map::clear() 统一释放
    mspKeyFramesTrash.insert(pKF);

    // 不要立即删除
    // delete pKF;
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    // 必须加锁，防止多线程访问冲突（特别是Tracking线程可能正在访问地图）
    unique_lock<mutex> lock(mMutexMap);

    // 恢复原始逻辑：强制清空所有地图点和关键帧
    // 这确保Reset是真正的重置，而不是保留可能损坏的状态
    
    // 删除所有地图点
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
    {
        if(*sit)
            delete *sit;
    }
    mspMapPoints.clear();

    // 删除所有关键帧
    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
    {
        if(*sit)
            delete *sit;
    }
    mspKeyFrames.clear();

    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();

    // 清空回收站 (垃圾回收)
    for(set<MapPoint*>::iterator sit = mspMapPointsTrash.begin(), send = mspMapPointsTrash.end(); sit != send; sit++)
    {
        if(*sit) delete *sit;
    }
    mspMapPointsTrash.clear();

    for(set<KeyFrame*>::iterator sit = mspKeyFramesTrash.begin(), send = mspKeyFramesTrash.end(); sit != send; sit++)
    {
        if(*sit) delete *sit;
    }
    mspKeyFramesTrash.clear();
    
    // Increment change index
    mnBigChangeIdx++;
}

} //namespace ORB_SLAM2


long unsigned int ORB_SLAM2::Map::GetLoadedMapMPCount()
{
    unique_lock<mutex> lock(mMutexMap);
    long unsigned int count = 0;
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
    {
        MapPoint* pMP = *sit;
        // pMP 可能为空或者 bad
        if(pMP && !pMP->isBad() && pMP->mbFromLoadedMap)
            count++;
    }
    return count;
}

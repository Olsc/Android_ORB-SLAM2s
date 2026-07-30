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

#include "KeyFrameDatabase.h"

#include "KeyFrame.h"
#include "Config.h"

#include<mutex>

using namespace std;

namespace ORB_SLAM2
{

KeyFrameDatabase::KeyFrameDatabase ():
    mpTree(new HBSTTree()), mnErasedCount(0)
{
}


void KeyFrameDatabase::add(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutex);

    if (pKF->mDescriptors.empty()) return;

    std::vector<size_t> objects(pKF->N);
    for(int i=0; i<pKF->N; i++) objects[i] = i;
    
    HBSTTree::MatchableVector matchables = HBSTTree::getMatchables(pKF->mDescriptors, objects, pKF->mnId);
    mpTree->add(matchables);

    mhmKeyFrames[pKF->mnId] = pKF;
}

void KeyFrameDatabase::erase(KeyFrame* pKF)
{
    unique_lock<mutex> lock(mMutex);
    if (mhmKeyFrames.erase(pKF->mnId) > 0) {
        mnErasedCount++;
        // 当删除数量达到设定的阈值时，触发树重建，彻底清理残留特征点
        if (mnErasedCount >= 20) {
            rebuild();
            mnErasedCount = 0;
        }
    }
}

void KeyFrameDatabase::clear()
{
    unique_lock<mutex> lock(mMutex);
    mpTree->clear();
    mhmKeyFrames.clear();
    mnErasedCount = 0;
}


vector<KeyFrame*> KeyFrameDatabase::DetectLoopCandidates(KeyFrame* pKF, float minScore)
{
    set<KeyFrame*> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
    list<pair<float,KeyFrame*> > lScoreAndMatch;

    {
        unique_lock<mutex> lock(mMutex);

        if (pKF->mDescriptors.empty()) return vector<KeyFrame*>();

        std::vector<size_t> objects(pKF->N);
        for(int i=0; i<pKF->N; i++) objects[i] = i;
        
        HBSTTree::MatchableVector query_matchables = HBSTTree::getMatchables(pKF->mDescriptors, objects, pKF->mnId);
        
        HBSTTree::MatchVectorMap matches;
        mpTree->match(query_matchables, matches, 50);
        
        for (auto m : query_matchables) delete m;

        for (const auto& match_pair : matches) {
            long unsigned int id = match_pair.first;
            if (id == pKF->mnId) continue;
            
            if (mhmKeyFrames.count(id) == 0) continue; 
            
            KeyFrame* pKFi = mhmKeyFrames[id];
            if (spConnectedKeyFrames.count(pKFi)) continue;

            int num_matches = match_pair.second.size();
            float score = (float)num_matches / (float)pKF->N;
            
            pKFi->mLoopScore = score;
            pKFi->mnLoopQuery = pKF->mnId;
            
            if (num_matches >= 15) {
                lScoreAndMatch.push_back(make_pair(score, pKFi));
            }
        }
    }

    if(lScoreAndMatch.empty())
        return vector<KeyFrame*>();

    list<pair<float,KeyFrame*> > lAccScoreAndMatch;
    float bestAccScore = 0;

    for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        KeyFrame* pKFi = it->second;
        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = it->first;
        KeyFrame* pBestKF = pKFi;
        for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF2 = *vit;
            if (pKF2->mnLoopQuery == pKF->mnId && pKF2->mLoopScore > 0)
            {
                accScore+=pKF2->mLoopScore;
                if(pKF2->mLoopScore>bestScore)
                {
                    pBestKF=pKF2;
                    bestScore = pKF2->mLoopScore;
                }
            }
        }

        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    float minScoreToRetain = 0.75f*bestAccScore;

    set<KeyFrame*> spAlreadyAddedKF;
    vector<KeyFrame*> vpLoopCandidates;
    vpLoopCandidates.reserve(lAccScoreAndMatch.size());

    for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        if(it->first>minScoreToRetain)
        {
            KeyFrame* pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpLoopCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }

    return vpLoopCandidates;
}

vector<KeyFrame*> KeyFrameDatabase::DetectRelocalizationCandidates(Frame *F)
{
    list<pair<float,KeyFrame*> > lScoreAndMatch;

    {
        unique_lock<mutex> lock(mMutex);

        if (F->mDescriptors.empty()) return vector<KeyFrame*>();

        std::vector<size_t> objects(F->N);
        for(int i=0; i<F->N; i++) objects[i] = i;
        
        HBSTTree::MatchableVector query_matchables = HBSTTree::getMatchables(F->mDescriptors, objects, F->mnId);
        
        HBSTTree::MatchVectorMap matches;
        mpTree->match(query_matchables, matches, 50);
        
        for (auto m : query_matchables) delete m;

        for (const auto& match_pair : matches) {
            long unsigned int id = match_pair.first;
            if (mhmKeyFrames.count(id) == 0) continue;
            
            KeyFrame* pKFi = mhmKeyFrames[id];

            int num_matches = match_pair.second.size();
            float score = (float)num_matches / (float)F->N;
            
            pKFi->mRelocScore = score;
            pKFi->mnRelocQuery = F->mnId;
            
            if (num_matches > 15) {
                lScoreAndMatch.push_back(make_pair(score, pKFi));
            }
        }
    }

    if(lScoreAndMatch.empty())
        return vector<KeyFrame*>();

    list<pair<float,KeyFrame*> > lAccScoreAndMatch;
    float bestAccScore = 0;

    for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        KeyFrame* pKFi = it->second;
        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = bestScore;
        KeyFrame* pBestKF = pKFi;
        for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF2 = *vit;
            if (pKF2->mnRelocQuery == F->mnId && pKF2->mRelocScore > 0)
            {
                accScore+=pKF2->mRelocScore;
                if(pKF2->mRelocScore>bestScore)
                {
                    pBestKF=pKF2;
                    bestScore = pKF2->mRelocScore;
                }
            }
        }
        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    float minScoreToRetain = 0.75f*bestAccScore;
    set<KeyFrame*> spAlreadyAddedKF;
    vector<KeyFrame*> vpRelocCandidates;
    vpRelocCandidates.reserve(lAccScoreAndMatch.size());
    for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        if(it->first>minScoreToRetain)
        {
            KeyFrame* pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpRelocCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }

    if(vpRelocCandidates.size() > RELOC_MAX_CANDIDATES)
    {
        vpRelocCandidates.resize(RELOC_MAX_CANDIDATES);
    }

    return vpRelocCandidates;
}

void KeyFrameDatabase::rebuild()
{
    // 清除树以安全释放所有 Matchable 对象的内存
    mpTree->clear();

    // 重新把 mhmKeyFrames 中所有的活动关键帧特征插入树中
    for (auto& pair : mhmKeyFrames) {
        KeyFrame* pKF = pair.second;
        if (!pKF || pKF->isBad() || pKF->mDescriptors.empty())
            continue;

        std::vector<size_t> objects(pKF->N);
        for(int i = 0; i < pKF->N; i++) objects[i] = i;

        HBSTTree::MatchableVector matchables = HBSTTree::getMatchables(pKF->mDescriptors, objects, pKF->mnId);
        mpTree->add(matchables);
    }
}

} //namespace ORB_SLAM2

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

#include "Tracking.h"

#include <chrono>
#include <unordered_set>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include "Map.h"
#include "Initializer.h"

#include "Optimizer.h"
#include "PnPsolver.h"
#include "Config.h"
#include "Common.h"

#include <iostream>
#include <mutex>
#include <unistd.h>
#include <future>
#include <unordered_map>
#include <numeric>
#include <thread>
#include <chrono>
#include "VtonaxProfiler.h" // 性能分析器

// 轻量级投影工具：避免在热点循环中频繁创建小矩阵，减少分配和复制开销
static inline void ProjectPwWithRTK(const float R[9], const float t[3],
                                    const cv::Point3f &Pw,
                                    float fx, float fy, float cx, float cy,
                                    float &u, float &v, float &Z)
{
    // Pc = R * Pw + t
    const float X = R[0] * Pw.x + R[1] * Pw.y + R[2] * Pw.z + t[0];
    const float Y = R[3] * Pw.x + R[4] * Pw.y + R[5] * Pw.z + t[1];
    Z = R[6] * Pw.x + R[7] * Pw.y + R[8] * Pw.z + t[2];

    if (Z > 0.0f)
    {
        const float invZ = 1.0f / Z;
        u = fx * X * invZ + cx;
        v = fy * Y * invZ + cy;
    }
    else
    {
        u = v = -1e10f;
    }
}

// 同步过滤：同时过滤索引映射，保持与out3d/out2d一致
static void FilterPnPInputsSync(const std::vector<cv::Point3f> &in3d,
                                const std::vector<cv::Point2f> &in2d,
                                const std::vector<int> &inQIdx,
                                const std::vector<int> &inRefIdx,
                                std::vector<cv::Point3f> &out3d,
                                std::vector<cv::Point2f> &out2d,
                                std::vector<int> &outQIdx,
                                std::vector<int> &outRefIdx)
{
    out3d.clear(); out2d.clear(); outQIdx.clear(); outRefIdx.clear();
    if(in3d.size()!=in2d.size() || in3d.size()!=inQIdx.size() || in3d.size()!=inRefIdx.size()) return;
    out3d.reserve(in3d.size()); out2d.reserve(in2d.size()); outQIdx.reserve(inQIdx.size()); outRefIdx.reserve(inRefIdx.size());
    // const float LIM2D = 1e5f; const float LIM3D = 1e6f;
    auto finite = [](float v){ return std::isfinite(v); };
    for(size_t i=0;i<in3d.size();++i){
        const cv::Point3f &P = in3d[i]; const cv::Point2f &q = in2d[i];
        if(!finite(P.x)||!finite(P.y)||!finite(P.z)||!finite(q.x)||!finite(q.y)) continue;
        if(std::fabs(P.x)>ORB_SLAM2::PNP_LIMIT_3D||std::fabs(P.y)>ORB_SLAM2::PNP_LIMIT_3D||std::fabs(P.z)>ORB_SLAM2::PNP_LIMIT_3D) continue;
        if(std::fabs(q.x)>ORB_SLAM2::PNP_LIMIT_2D||std::fabs(q.y)>ORB_SLAM2::PNP_LIMIT_2D) continue;
        out3d.push_back(P); out2d.push_back(q); outQIdx.push_back(inQIdx[i]); outRefIdx.push_back(inRefIdx[i]);
    }
}

// OpenCV错误回调：将断言转为返回错误，避免进程中止
static int SeeCvErrorCallback(int status, const char* func_name, const char* err_msg,
                           const char* file_name, int line, void* userdata)
{
    LOGE("OpenCV错误: status=%d func=%s msg=> %s file=%s:%d", status, func_name?func_name:"?", err_msg?err_msg:"?", file_name?file_name:"?", line);
    return 0; // 不中断
}

static void EnsureCvErrorRedirect()
{
    static std::atomic<bool> inited{false};
    bool expected=false;
    if(inited.compare_exchange_strong(expected, true)){
        cv::redirectError(SeeCvErrorCallback);
        cv::setBreakOnError(false);
    }
}

// 统一的安全PnP：样本<6绝不调用RANSAC；5点降为4点P3P；双精度内参
static bool SolvePnPSafe(const std::vector<cv::Point3f>& pts3d,
                         const std::vector<cv::Point2f>& pts2d,
                         const cv::Mat& K32, const cv::Mat& dist32,
                         cv::Mat& rvec, cv::Mat& tvec,
                         std::vector<int>& inliers)
{
    EnsureCvErrorRedirect();
    
    // 严格检查点数：为避免OpenCV内部DLT断言，统一要求至少6点
    if (pts3d.size() < 6 || pts2d.size() < 6 || pts3d.size() != pts2d.size())
    {
        LOGE("安全PnP求解: 点数不足或数量不匹配 3D=%d 2D=%d", (int)pts3d.size(), (int)pts2d.size());
        return false;
    }
    
    // 检查输入数据的有效性
    for(size_t i=0; i<pts3d.size(); ++i) {
        if(!std::isfinite(pts3d[i].x) || !std::isfinite(pts3d[i].y) || !std::isfinite(pts3d[i].z) ||
           !std::isfinite(pts2d[i].x) || !std::isfinite(pts2d[i].y)) {
            LOGE("安全PnP求解: 输入数据包含无效值");
            return false;
        }
    }
    
    cv::Mat Kd; K32.convertTo(Kd, CV_64F);
    cv::Mat Dd; dist32.convertTo(Dd, CV_64F);
    
    try{
        // 6个点或更多时，使用RANSAC（EPNP最小样本为4，避免内部走DLT 6点断言）
        bool ok = cv::solvePnPRansac(pts3d, pts2d, Kd, Dd, rvec, tvec, false, ORB_SLAM2::PNP_RANSAC_ITERATIONS, ORB_SLAM2::PNP_RANSAC_ERROR, ORB_SLAM2::PNP_RANSAC_CONFIDENCE, inliers, cv::SOLVEPNP_EPNP);
        if(ok) {
            // 验证结果的有效性
            if(!std::isfinite(rvec.at<double>(0)) || !std::isfinite(rvec.at<double>(1)) || !std::isfinite(rvec.at<double>(2)) ||
               !std::isfinite(tvec.at<double>(0)) || !std::isfinite(tvec.at<double>(1)) || !std::isfinite(tvec.at<double>(2))) {
                LOGE("安全PnP求解: RANSAC结果包含无效值");
                inliers.clear();
                return false;
            }
        }
        return ok;
    } catch(const cv::Exception& e){ 
        LOGE("安全PnP求解: OpenCV异常: %s", e.what());
        inliers.clear();
        return false; 
    } catch(const std::exception& e){ 
        LOGE("安全PnP求解: 标准异常: %s", e.what());
        inliers.clear();
        return false; 
    } catch(...){ 
        LOGE("安全PnP求解: 未知异常");
        inliers.clear();
        return false; 
    }
}

// 使用参考快照在主线程快速绑定已加载地图点，避免只出现一次的闪烁现象
void ORB_SLAM2::Tracking::BindLoadedMapPointsUsingSnapshots()
{
    // 只在真正对齐成功后才绑定，防止在错误位置强制匹配
    // Reset后未对齐时应允许正常扫描新点，因此需检查对齐状态
    if (mRefDesc.empty() || mRefSnapshots.empty() || mRefIdxToMP.empty())
        return;

    // 若当前帧无位姿/关键点，跳过
    if (mCurrentFrame.N <= 0 || mCurrentFrame.mTcw.empty())
        return;
    
    // 如果当前跟踪已很稳定（内点多），跳过绑定以节省主线程开销
    // 阈值取自Config.h，避免每帧做繁重的投影匹配
    if(mnMatchesInliers > MAIN_THREAD_BIND_INLIER_THRESHOLD) return;
    
    // 只有在成功对齐后才绑定，防止Reset后在没有对齐的情况下错误匹配
    // 但如果地图已加载但没有对齐，应该允许系统继续扫描新点，而不是强制绑定旧地图点
    {
        std::unique_lock<std::mutex> lk(mMutexReloc);
        if(!mbHaveMapAlign) {
            // 没有对齐时，不进行绑定，允许系统正常扫描新点
            return;
        }
    }

    // 取只读拷贝，避免长时间占用互斥量
    cv::Mat refDesc; std::vector<MapPoint*> refMPs; std::vector<RefMPSnapshot> refSnaps;
    {
        std::unique_lock<std::mutex> lk(mMutexReloc);
        refDesc = mRefDesc; refMPs = mRefIdxToMP; refSnaps = mRefSnapshots;
    }
    if(refSnaps.size()!=refMPs.size() || refDesc.rows!=(int)refMPs.size()) return;

    // 通过当前帧位姿投影快照点，进行半径内最近邻像素匹配（不读 MP 成员）
    const float &fx = mCurrentFrame.fx;
    const float &fy = mCurrentFrame.fy;
    const float &cx = mCurrentFrame.cx;
    const float &cy = mCurrentFrame.cy;

    // 提取R,t为标量数组以便快速投影
    cv::Mat RcwM = mCurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcwM = mCurrentFrame.mTcw.rowRange(0,3).col(3);
    float Rcw[9] = {
        RcwM.at<float>(0,0), RcwM.at<float>(0,1), RcwM.at<float>(0,2),
        RcwM.at<float>(1,0), RcwM.at<float>(1,1), RcwM.at<float>(1,2),
        RcwM.at<float>(2,0), RcwM.at<float>(2,1), RcwM.at<float>(2,2)
    };
    float tcw[3] = { tcwM.at<float>(0), tcwM.at<float>(1), tcwM.at<float>(2) };

    const int curMapId = mnCurrentMapId.load();
    // 使用Config.h中的参数限制每帧绑定数量，避免主线程卡顿
    const int nMaxBind = std::min(MAIN_THREAD_MAX_BIND_PER_FRAME, (int)refSnaps.size());
    int projBinds = 0;

    // 投影所有可见点，根据质量排序后绑定，提高稳定性和一致性
    struct BindCandidate { int refIdx; int frameIdx; float projErr; float depth; };
    std::vector<BindCandidate> candidates; candidates.reserve(nMaxBind * 2);
    
    const bool haveAlign = mbHaveMapAlign;
    // const float radius = haveAlign ? 12.0f : 8.0f;
    const float radius = haveAlign ? TRACKING_SEARCH_RADIUS_ALIGNED : TRACKING_SEARCH_RADIUS_UNALIGNED;
    
    // 使用网格搜索减少遍历数量
    std::vector<int> gridCandidates;
    bool useGrid = false;
    {
        std::unique_lock<std::mutex> lk(mMutexReloc);
        if(mRefGrid.nCols > 0) {
             cv::Mat OwM = mCurrentFrame.GetCameraCenter();
             cv::Point3f Ow(OwM.at<float>(0), OwM.at<float>(1), OwM.at<float>(2));
             mRefGrid.GetCandidatesInBBox(Ow, TRACKING_GRID_SEARCH_RADIUS, gridCandidates); // 40m radius
             useGrid = true;
        }
    }

    const size_t totalPoints = useGrid ? gridCandidates.size() : refSnaps.size();
    // 进一步限制处理数量，防止卡顿
    int stride = 1;
    if(totalPoints > TRACKING_CANDIDATE_STRIDE_THRESHOLD) stride = (int)(totalPoints / TRACKING_CANDIDATE_STRIDE_THRESHOLD) + 1;

    // 遍历所有参考点，找到可投影的候选
    for(size_t k=0; k < totalPoints; k += stride){
        size_t i = useGrid ? gridCandidates[k] : k;
        if(i >= refSnaps.size()) continue;

        const RefMPSnapshot &s = refSnaps[i];
        
        // 只投影当前激活地图的点，减少多地图时的冗余计算
        if(s.mapId != curMapId && curMapId != -1) continue;

        float u=0.f,v=0.f,Z=0.f;
        ProjectPwWithRTK(Rcw, tcw, s.Pw, fx, fy, cx, cy, u, v, Z);
        if(Z<=0 || Z>BIND_MAX_DEPTH) continue;  // 过滤深度异常的点
        if(u<0 || u>=mCurrentFrame.mnMaxX || v<0 || v>=mCurrentFrame.mnMaxY) continue;


        // 使用空间网格搜索代替暴力遍历
        vector<size_t> vIndices = mCurrentFrame.GetFeaturesInArea(u, v, radius);
        
        if (vIndices.empty())
            continue;
        int bestIdx = -1;
        float bestDist = 1e10f;
        for (size_t j : vIndices)
        {
            if (mCurrentFrame.mvpMapPoints[j])
                continue; // 已经有绑定
            const cv::Point2f &pt = mCurrentFrame.mvKeysUn[j].pt;
            float du = pt.x - u;
            float dv = pt.y - v;
            float r2 = du * du + dv * dv;
            if (r2 > radius * radius)
                continue;
            if (r2 < bestDist)
            {
                bestDist = r2;
                bestIdx = (int)j;
            }
        }

        if(bestIdx>=0 && bestDist<1e9f){
            // 计算投影质量得分：距离越小、深度越合理，得分越高
            candidates.push_back({(int)i, bestIdx, bestDist, Z});
        }
    }
    
    // 按投影误差排序，优先绑定质量好的匹配
    std::sort(candidates.begin(), candidates.end(), 
              [](const BindCandidate &a, const BindCandidate &b){ 
                  return a.projErr < b.projErr; 
              });
    
    // 绑定前nMaxBind个最佳匹配，去重确保一个特征点只绑定一次
    std::vector<bool> frameUsed(mCurrentFrame.N, false);
    for(const auto &c : candidates){
        if(projBinds >= nMaxBind) break;
        if(frameUsed[c.frameIdx]) continue; // 该特征点已被绑定
        
        MapPoint* pMP = refMPs[c.refIdx];
        if(pMP && !pMP->isBad()){
            mCurrentFrame.mvpMapPoints[c.frameIdx] = pMP;
            pMP->mbMatchedInCurrentFrame = true;
            frameUsed[c.frameIdx] = true;
            projBinds++;
        }
    }
}
void ORB_SLAM2::Tracking::BuildLoadedRefCache()
{
    // 并发保护：避免多次重入导致refDesc与索引不同步
    bool expected=false; if(!mRefBuilding.compare_exchange_strong(expected, true)) return;
    
    // 使用双缓冲在锁外构建，锁内一次性交换，降低锁竞争
    cv::Mat newRefDesc; 
    std::vector<MapPoint*> newRefIdxToMP; 
    std::vector<RefMPSnapshot> newRefSnapshots;
    std::vector<size_t> objects;
    
    const vector<MapPoint*> allMPs = mpMap->GetAllMapPoints();
    
    // 预分配内存,避免动态增长
    // 先统计有效点数量
    int validCount = 0;
    for(MapPoint* p : allMPs) {
        if(!p || p->isBad()) continue;
        if(!p->mbFromLoadedMap) continue;
        cv::Mat d = p->GetDescriptor();
        if(d.empty()) continue;
        validCount++;
        if(validCount >= TRACKING_REF_CACHE_LIMIT) break;  // 限制最大数量
    }
    
    // 一次性预分配所有需要的空间
    if(validCount > 0) {
        newRefIdxToMP.reserve(validCount);
        newRefSnapshots.reserve(validCount);
        objects.reserve(validCount);
        
        // 预分配描述子矩阵,避免逐行push_back的拷贝开销
        // 先假设 descriptor（特征描述子）的列数是 32，之后再通过实际数据去确认
        int descCols = ORB_DESC_COLS;

        // 先获取第一个有效描述子来确定维度
        for(MapPoint* p : allMPs) {
            if(!p || p->isBad()) continue;
            if(!p->mbFromLoadedMap) continue;
            cv::Mat d = p->GetDescriptor();
            if(!d.empty()) {
                descCols = d.cols;
                break;
            }
        }
        
        newRefDesc.create(validCount, descCols, CV_8U);
    }
    
    int totalLoaded = 0, withDesc = 0, highQuality = 0;
    int rowIdx = 0;  // 当前写入行
    
    int loopCnt = 0;
    for(MapPoint* p : allMPs){
        // 定期检查退出标记，避免Reset时因大规模循环导致 join() 阻塞
        if(++loopCnt % 100 == 0 && mbRelocThreadStop) break;

        if(!p || p->isBad()) continue;
        if(!p->mbFromLoadedMap) continue;
        totalLoaded++;
        
        // 加载的地图点在导入后通常没有关键帧观测，这里不再以 Observations 过滤
        
        // 复制描述子（只读）
        cv::Mat d = p->GetDescriptor();
        if(d.empty()) continue;
        withDesc++;
        
        // 直接写入预分配的矩阵,避免push_back的拷贝
        if(rowIdx < newRefDesc.rows) {
            d.copyTo(newRefDesc.row(rowIdx));
        }
        
        newRefIdxToMP.push_back(p);

        // 复制必要的几何信息到快照，避免后续对 MapPoint 的加锁读取
        RefMPSnapshot snap;
        {
            cv::Mat Pw = p->GetWorldPos();
            snap.Pw = cv::Point3f(Pw.at<float>(0), Pw.at<float>(1), Pw.at<float>(2));
        }
        snap.minD = p->GetMinDistanceInvariance();
        snap.maxD = p->GetMaxDistanceInvariance();
        snap.mapId = p->GetMapId();
        newRefSnapshots.push_back(snap);

        // HBST的叶节点映射（行索引）
        objects.push_back(rowIdx);
        
        highQuality++;
        rowIdx++;
        if(rowIdx >= TRACKING_REF_CACHE_LIMIT) break;
    }
    
    // 如果实际数量少于预分配,调整矩阵大小
    if(rowIdx < newRefDesc.rows) {
        newRefDesc = newRefDesc.rowRange(0, rowIdx).clone();
    }
    
    // 构建 HBST 树 (移到锁外以避免持有锁时间过长)
    std::shared_ptr<HBSTTree> pNewRefTree = std::make_shared<HBSTTree>();
    if (!newRefDesc.empty() && !objects.empty()) {
        HBSTTree::MatchableVector matchables = HBSTTree::getMatchables(newRefDesc, objects, 0);
        pNewRefTree->add(matchables);
    }

    // 交换到正式对象（短锁保护）
    {
        std::unique_lock<std::mutex> lk(mMutexReloc);
        mRefDesc = newRefDesc; // 浅拷贝
        mRefIdxToMP.swap(newRefIdxToMP);
        mRefSnapshots.swap(newRefSnapshots);

        mpRefTree = pNewRefTree;
        mRefCachedMPCount = (int)mRefIdxToMP.size();
        mRefLastBuildTs = mLastTimestamp;
        
        mRefLastBuildTs = mLastTimestamp;
    }
    
    // 构建空间网格索引 (移到锁外构建以避免阻塞主线程)
    // 使用本地对象构建，然后交换
    LoadedMapGrid newGrid;
    newGrid.Build(newRefSnapshots, 10.0f);
    
    {
        std::unique_lock<std::mutex> lk(mMutexReloc);
        mRefGrid = newGrid;
    } 

    // 通知后台线程参考缓存已就绪（无延时）
    mSnapSeqProduced++;
    mCvReloc.notify_all();
    mRefBuilding.store(false);
}

void ORB_SLAM2::Tracking::PublishRelocAlignment(const cv::Mat &TmapFromSlam, int inliers, float conf, double ts, int mapId)
{
    std::unique_lock<std::mutex> lk(mMutexRelocBuf);
    mRelocSeqProduced++;
    mRelocBuf.seq = mRelocSeqProduced;
    mRelocBuf.T_map_from_slam = TmapFromSlam.clone();
    mRelocBuf.inliers = inliers;
    mRelocBuf.confidence = conf;
    mRelocBuf.ts = ts;
    mRelocBuf.mapId = mapId;
}

bool ORB_SLAM2::Tracking::TryConsumeRelocAlignment(RelocAlignResult &out)
{
    std::unique_lock<std::mutex> lk(mMutexRelocBuf);
    if(mRelocSeqProduced == mRelocSeqConsumed) return false;
    mRelocSeqConsumed = mRelocSeqProduced;
    out = mRelocBuf;
    return true;
}

// 平滑对齐更新：使用EMA（指数移动平均）减少单帧抖动
void ORB_SLAM2::Tracking::UpdateAlignmentSmooth(const cv::Mat &T_new, int inliers, float confidence, double ts)
{
    // 质量检查：只接受高质量的对齐结果
    const int minInliersForUpdate = TRACKING_ALIGN_MIN_INLIERS_UPDATE;
    const float minConfidenceForUpdate = TRACKING_ALIGN_MIN_CONFIDENCE_UPDATE;
    
    if(inliers < minInliersForUpdate || confidence < minConfidenceForUpdate) {
        return;  // 质量不足，不更新
    }
    
    // 降低更新频率：每3帧更新一次
    if(++mAlignSkipCounter % TRACKING_ALIGN_SMOOTH_SKIP_FRAMES != 0) {
        return;
    }
    
    std::unique_lock<std::mutex> lk(mMutexReloc);
    
    // 首次初始化
    if(mSmoothedT_map_from_slam.empty()) {
        mSmoothedT_map_from_slam = T_new.clone();
        mT_map_from_slam = T_new.clone();
        mAlignUpdateCount++;
        return;
    }
    
    // 指数移动平均（EMA）
    // alpha 根据质量动态调整：质量越高，新值权重越大
    float qualityScore = std::min(100.0f, static_cast<float>(inliers));
    float alpha = std::min(0.5f, confidence * qualityScore / 100.0f);
    alpha = std::max(0.1f, alpha);  // 限制在 0.1-0.5 之间
    
    // 直接在原矩阵上更新，避免克隆（关键优化点）
    cv::Mat& smoothed = mSmoothedT_map_from_slam;
    
    // smoothed = (1-alpha)*smoothed + alpha*T_new
    // 只更新旋转和平移部分（前3行4列），避免处理底部的[0 0 0 1]
    float oneMinusAlpha = 1.0f - alpha;
    for(int i = 0; i < 3; i++) {
        float* smoothRow = smoothed.ptr<float>(i);
        const float* newRow = T_new.ptr<float>(i);
        for(int j = 0; j < 4; j++) {
            smoothRow[j] = oneMinusAlpha * smoothRow[j] + alpha * newRow[j];
        }
    }
    
    // 旋转矩阵正交化（保证旋转矩阵的有效性）
    // 使用SVD分解获得最近的正交矩阵
    cv::Mat R = smoothed.rowRange(0,3).colRange(0,3);
    cv::Mat U, W, Vt;
    cv::SVD::compute(R, W, U, Vt, cv::SVD::MODIFY_A);
    
    // R = U * Vt 是最近的正交矩阵
    cv::Mat R_ortho = U * Vt;
    R_ortho.copyTo(smoothed.rowRange(0,3).colRange(0,3));
    
    // 更新主对齐变换（浅拷贝，避免克隆）
    mT_map_from_slam = mSmoothedT_map_from_slam;
    
    mAlignUpdateCount++;
}



using namespace std;

namespace ORB_SLAM2
{

Tracking::Tracking(System *pSys, FrameDrawer *pFrameDrawer,  Map *pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mLastInitAttemptTime(0.0), mpSystem(pSys),
    mpFrameDrawer(pFrameDrawer),  mpMap(pMap), mnLastRelocFrameId(0), mnCurrentMapId(0),
    mCfgTopKWords(20), mCfgMaxCandidates(5000), mCfgMatchChunk(1000),
    mCfgMaxBindInliers(100), mCfgMaxProjBinds(50), mConsecutiveLostFrames(0)
{
    // 设置OpenCV的错误处理为非致命，防止硬崩
    cv::redirectError(SeeCvErrorCallback);
    // 从 Config.h 加载相机参数
    float fx = CAMERA_FX;
    float fy = CAMERA_FY;
    float cx = CAMERA_CX;
    float cy = CAMERA_CY;

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = CAMERA_K1;
    DistCoef.at<float>(1) = CAMERA_K2;
    DistCoef.at<float>(2) = CAMERA_P1;
    DistCoef.at<float>(3) = CAMERA_P2;
    const float k3 = CAMERA_K3;
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = 0; // bf 参数在单目相机中不使用

    float fps = CAMERA_FPS;
    if(fps==0)
        fps=30;

    // 插入关键帧和检查重定位的最大/最小帧数
    mMinFrames = 0;
    mMaxFrames = fps;

    int nRGB = CAMERA_RGB;
    mbRGB = nRGB;

    int nFeatures = ORB_EXTRACTOR_N_FEATURES;
    float fScaleFactor = ORB_EXTRACTOR_SCALE_FACTOR;
    int nLevels = ORB_EXTRACTOR_N_LEVELS;
    int fIniThFAST = ORB_EXTRACTOR_INI_TH_FAST;
    int fMinThFAST = ORB_EXTRACTOR_MIN_TH_FAST;

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    
    // 仅支持单目模式
    mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    // 单目模式不需要深度阈值和深度图因子

    // 启动基于生命周期事件的后台全局重定位线程（无延时唤醒）
    StartGlobalRelocThread();
    
    // 设置重定位配置（topKWords/maxCandidates/matchChunk/bgSleepUs/maxBindInliers/maxProjBinds）
    // 确保后台线程不会干扰SLAM主流程
    SetRelocConfig(SYSTEM_RELOC_CONFIG_TOP_K, SYSTEM_RELOC_CONFIG_MAX_CANDIDATES, SYSTEM_RELOC_CONFIG_MATCH_CHUNK, SYSTEM_RELOC_CONFIG_BG_SLEEP_US, SYSTEM_RELOC_CONFIG_MAX_BIND_INLIERS, SYSTEM_RELOC_CONFIG_MAX_PROJ_BINDS);
}

void Tracking::GlobalRelocLoop(int sessionId)
{
    // 定期尝试使用2D-3D匹配+PnP将SLAM与加载的地图对齐
    mLastBgRunTime = std::chrono::steady_clock::now();
    
    while(true){
        // 等待事件：新快照或停止信号
        {
            std::unique_lock<std::mutex> lk(mMutexReloc);
            mCvReloc.wait(lk, [this, sessionId]{ return mbRelocThreadStop || mRelocThreadSessionId.load() != sessionId || mSnapSeqProduced!=mSnapSeqConsumed; });
            if(mbRelocThreadStop || mRelocThreadSessionId.load() != sessionId) break;
        }
        

        // 使用 try_lock 复制快照，避免阻塞主跟踪线程
        cv::Mat desc; std::vector<cv::KeyPoint> keys; int N=0; double ts=0.0; cv::Mat TcwSlam;
        {
            std::unique_lock<std::mutex> lk(mMutexReloc, std::try_to_lock);
            if (!lk.owns_lock()) {
                // 跟踪线程正在更新快照，跳过一次
                mSnapSeqConsumed.store(mSnapSeqProduced.load());
                continue;
            }
            if(!mLastDesc.empty()) desc = mLastDesc.clone();
            keys = mLastKeysUn; N = mLastN; ts = mLastTimestamp;
            if(!mLastTcwSlam.empty()) TcwSlam = mLastTcwSlam.clone();
            // 消费本次快照（使用原子load/store避免赋值原子对象）
            mSnapSeqConsumed.store(mSnapSeqProduced.load());
        }

        if(desc.empty() || N<=0 || TcwSlam.empty()){
            continue;
        }


        // KNN + ratio，然后回退
        // 步骤1：计算当前帧的BoW（移到锁外以避免在计算期间持有锁）
        Frame tmp; // 用于bow的轻量级容器
        // tmp.mDescriptors = desc; tmp.N = N; tmp.mvKeysUn = keys; tmp.ComputeBoW();

        cv::Mat refDesc;
        std::vector<RefMPSnapshot> refSnaps;
        std::vector<int> valToRef;
        std::vector<int> candidateIdx;

        // 重试循环以确保所有参考数据的一致快照（refDesc, refSnaps, mRefInverted）
        // 使用 try_lock 确保后台线程不会阻塞主跟踪线程
        while(true) {
            std::unique_lock<std::mutex> lk(mMutexReloc, std::try_to_lock);
            if (!lk.owns_lock()) {
                // 跟踪线程正在持有锁（正在更新快照），等待释放后立即重新尝试
                std::unique_lock<std::mutex> waitLock(mMutexReloc);
                mCvReloc.wait_for(waitLock, std::chrono::milliseconds(1));
                continue;
            }
            
            // 在内部重试循环中检查退出标志
            // 否则 Reset() 调用 StopGlobalRelocThread() 时，如果此处在死循环重试，主线程会卡死在 join()
            if(mbRelocThreadStop || mRelocThreadSessionId.load() != sessionId) break;
            
            // 检查重retry次数，防止死循环
            if(mRefCacheRetryCount >= TRACKING_MAX_REF_CACHE_RETRIES) {
                LOGD("GlobalRelocLoop: 缓存重建连续失败%d次，跳过本次处理", mRefCacheRetryCount);
                mRefCacheRetryCount = 0;
                // 消费快照，继续等待下一次机会
                mSnapSeqConsumed.store(mSnapSeqProduced.load());
                break;  // 退出重试循环，回到外层等待
            }
            
            // 检查是否需要构建/重建缓存，确保描述符和快照间的一致性
            // 移除了mRefCachedMPCount<200检查，防止点少时死循环
            if(mRefDesc.empty() || mRefDesc.rows!=(int)mRefSnapshots.size()) {
                // 1. 检查状态：系统必须已初始化
                if(mState==NO_IMAGES_YET || mState==NOT_INITIALIZED) {
                    lk.unlock();
                    mRefCacheRetryCount++;
                    {
                        std::unique_lock<std::mutex> waitLock(mMutexReloc);
                        mCvReloc.wait_for(waitLock, std::chrono::milliseconds(500));
                    }
                    continue;
                }

                // 2. 检查是否有加载的地图点，如果没有，则不需要构建缓存，避免死循环空转
                // 防止新环境建图时后台线程频繁占用锁
                int loadedCount = 0;
                {
                    // 快速检查，无需获取全量地图锁
                    // 注意：这只是一个估计，为了性能我们不严格加锁统计
                     loadedCount = mpMap->GetLoadedMapMPCount();
                }

                if(loadedCount == 0) {
                     lk.unlock();
                     mRefCacheRetryCount++;
                     {
                         std::unique_lock<std::mutex> waitLock(mMutexReloc);
                         mCvReloc.wait_for(waitLock, std::chrono::seconds(2));
                     }
                     continue;
                }

                lk.unlock();
                BuildLoadedRefCache();

                // 如果重建后仍然为空，暂停一会避免死循环空转
                {
                    std::unique_lock<std::mutex> lk2(mMutexReloc);
                    if(mRefDesc.empty()) {
                        mRefCacheRetryCount++;
                        mCvReloc.wait_for(lk2, std::chrono::milliseconds(500));
                    } else {
                        mRefCacheRetryCount = 0;
                    }
                }
                continue; // 重试
            }

            // 成功获取有效缓存，重置计数
            mRefCacheRetryCount = 0;

            // 复制一致的数据
            refDesc = mRefDesc; // 浅拷贝可以（cv::Mat引用计数）
            refSnaps = mRefSnapshots; // 快照的深拷贝
            std::shared_ptr<HBSTTree> pTree = mpRefTree;

            // 初始化 valToRef 恒等映射（修复 HBST 重构引入的遗漏问题，防止全局重定位丢弃所有匹配）
            valToRef.resize(refSnaps.size());
            for(size_t i = 0; i < refSnaps.size(); ++i) {
                valToRef[i] = (int)i;
            }
            
            // 立即释放锁！以避免在重型匹配计算期间持有锁而阻塞主跟踪线程！
            lk.unlock();
            
            if (pTree && !desc.empty()) {
                std::vector<size_t> query_objects;
                for(int i=0; i<desc.rows; i++) query_objects.push_back(i);
                HBSTTree::MatchableVector query_matchables = HBSTTree::getMatchables(desc, query_objects, 0);

                HBSTTree::MatchVectorMap matches;
                pTree->match(query_matchables, matches, 50);

                for(auto m : query_matchables) delete m;

                std::unordered_set<int> candidateSet;
                for(const auto& match_pair : matches) {
                    for(const auto& m : match_pair.second) {
                        for(size_t ref_idx : m.object_references) {
                            candidateSet.insert(ref_idx);
                        }
                    }
                }
                candidateIdx.assign(candidateSet.begin(), candidateSet.end());
            }

            break; // 完成 (此时锁已释放，安全退出循环)
        }
        
        // 验证检查
        if(refDesc.empty() || refDesc.cols!=desc.cols){ continue; }
        if(candidateIdx.empty()){
            // 退化策略：先基于最近位姿粗投影筛选可见点
            // 否则对参考快照均匀采样，确保任何起始位置都能尝试匹配
            if(!TcwSlam.empty()){
                // 使用网格索引加速查找
                cv::Mat Rcw = TcwSlam.rowRange(0,3).colRange(0,3);
                cv::Mat tcw = TcwSlam.rowRange(0,3).col(3);
                cv::Mat Ow = -Rcw.t()*tcw;
                cv::Point3f center(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
                
            // 从配置文件读取搜索半径
                float searchRadius = TRACKING_RELOC_SEARCH_RADIUS;
                
                std::vector<int> gridCandidates;
                gridCandidates.reserve(mCfgMaxCandidates);
                
                // 使用 try_lock 保护 mRefGrid 访问，避免阻塞主线程
                {
                    std::unique_lock<std::mutex> lk(mMutexReloc, std::try_to_lock);
                    if (lk.owns_lock()) {
                        // 确保Grid状态与当前快照一致（防止极端情况下的重建）
                        if(mRefDesc.rows == (int)refSnaps.size()) {
                             mRefGrid.GetCandidatesInBBox(center, searchRadius, gridCandidates);
                        }
                    }
                }
                
                if(!gridCandidates.empty()) {
                    for(int idx : gridCandidates){
                        if(idx<0 || idx>=(int)refSnaps.size()) continue;
                        const RefMPSnapshot &s = refSnaps[idx];
                        
                        // 投影检查
                        cv::Mat Pw = (cv::Mat_<float>(3,1) << s.Pw.x, s.Pw.y, s.Pw.z);
                        cv::Mat Pc = Rcw*Pw + tcw; 
                        float z = Pc.at<float>(2);
                        
                        if(z<=0) continue; // 在相机后面
                        
                        candidateIdx.push_back(idx);
                        if((int)candidateIdx.size()>=mCfgMaxCandidates) break;
                    }
                }
                
                // 如果网格搜索结果太少（可能是因为定位漂移太大，导致相机坐标跑出了网格范围），
                // 尝试全量采样作为最后的保底
                if(candidateIdx.empty()) {
                    int step = std::max(1, (int)refSnaps.size()/std::max(1, mCfgMaxCandidates));
                    for(int i=0;i<(int)refSnaps.size() && (int)candidateIdx.size()<mCfgMaxCandidates;i+=step){ candidateIdx.push_back(i); }
                }
                
            } else {
                 // 没有位姿先验，只能均匀采样
                int step = std::max(1, (int)refSnaps.size()/std::max(1, mCfgMaxCandidates));
                for(int i=0;i<(int)refSnaps.size() && (int)candidateIdx.size()<mCfgMaxCandidates;i+=step){ candidateIdx.push_back(i); }
            }
        }

        // 步骤2.5：基于姿态的门控（利用最近姿态做粗筛）
        // 自适应策略：若粗筛结果过少，说明位姿不准，回退到使用原始候选集以提高稳定性
        int candidatesBeforeGating = (int)candidateIdx.size();
        if(!TcwSlam.empty() && candidatesBeforeGating >= TRACKING_GATING_MIN_CANDIDATES){  // 只在候选数足够时才应用姿态粗筛
            cv::Mat Rcw = TcwSlam.rowRange(0,3).colRange(0,3);
            cv::Mat tcw = TcwSlam.rowRange(0,3).col(3);
            std::vector<int> gated; gated.reserve(candidateIdx.size());
            for(int idx : candidateIdx){
                if(idx<0 || idx>=(int)refSnaps.size()) continue;
                const RefMPSnapshot &s = refSnaps[idx];
                cv::Mat Pw = (cv::Mat_<float>(3,1) << s.Pw.x, s.Pw.y, s.Pw.z);
                cv::Mat Pc = Rcw*Pw + tcw;
                float z = Pc.at<float>(2);
                if(z<=0) continue;
                // 使用平方距离避免sqrt开销
                float dist2 = Pc.at<float>(0)*Pc.at<float>(0)+Pc.at<float>(1)*Pc.at<float>(1)+z*z;
                float minD = s.minD;
                float maxD = s.maxD;
                // 放宽姿态粗筛范围；若参考深度无效则不过滤
                if(minD>0 && maxD>0){
                    float minD2 = (minD*0.4f)*(minD*0.4f);
                    float maxD2 = (maxD*2.5f)*(maxD*2.5f);
                    if(dist2<minD2 || dist2>maxD2) continue;
                }
                gated.push_back(idx);
                if((int)gated.size()>=mCfgMaxCandidates) break;
            }
            // 自适应回退：若粗筛后候选数<原始数量的20%且<100，说明位姿不准，保留原始候选以提高鲁棒性
            if(!gated.empty() && ((int)gated.size() >= candidatesBeforeGating/5 || (int)gated.size() >= 100)){
                candidateIdx.swap(gated);
            } else {
            }
        }

        // 步骤3：仅对候选行进行KNN匹配（分割成块；可选并行）
        // 复用线程局部的BF匹配器，降低构造与缓存开销
        static thread_local cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<std::vector<cv::DMatch>> knn; knn.reserve(N);
        const int chunk = mCfgMatchChunk;
        for(size_t base=0;base<candidateIdx.size();base+=chunk){
            size_t end = std::min(candidateIdx.size(), base+chunk);
            // 收集子矩阵视图：预分配矩阵空间，避免逐行 push_back 导致的重复内存分配与拷贝开销
            cv::Mat sub;
            sub.create(end-base, refDesc.cols, CV_8U);
            std::vector<int> subMap; subMap.reserve(end-base);
            int rowIdx = 0;
            for(size_t i=base;i<end;i++){
                int ridx = (int)candidateIdx[i];
                if(ridx<0 || ridx>=refDesc.rows) continue; // 越界保护
                refDesc.row(ridx).copyTo(sub.row(rowIdx));
                subMap.push_back(ridx);
                rowIdx++;
            }
            if(rowIdx==0) continue; // 无有效子集则跳过本批
            if(rowIdx < sub.rows){
                sub = sub.rowRange(0, rowIdx); // 裁切到实际大小（轻量级浅拷贝，共享 buffer 头信息）
            }
            std::vector<std::vector<cv::DMatch>> knnPart;
            matcher.knnMatch(desc, sub, knnPart, 2);
            // 将trainIdx重新映射到全局
            for(auto &vec : knnPart){
                if(vec.size()>=1){
                    if(vec[0].trainIdx>=0 && vec[0].trainIdx<(int)subMap.size()) vec[0].trainIdx = subMap[vec[0].trainIdx]; else vec.resize(0);
                    if(vec.size()>=2){ if(vec[1].trainIdx>=0 && vec[1].trainIdx<(int)subMap.size()) vec[1].trainIdx = subMap[vec[1].trainIdx]; }
                }
            }
            knn.insert(knn.end(), knnPart.begin(), knnPart.end());
        }
        std::vector<cv::Point2f> pts2d; pts2d.reserve(knn.size());
        std::vector<cv::Point3f> pts3d; pts3d.reserve(knn.size());
        std::vector<int> qIdx; qIdx.reserve(knn.size());
        std::vector<int> refIdx; refIdx.reserve(knn.size());
        std::vector<int> descDist; descDist.reserve(knn.size());
        const float ratio=TRACKING_KNN_RATIO; const int distMax=TRACKING_KNN_DIST_MAX;
        int keptPairs=0;
        for(size_t i=0;i<knn.size();++i){
            if(knn[i].size()<1) continue;  // 至少要有1个匹配
            const cv::DMatch &m1=knn[i][0];
            if(m1.distance>distMax) continue;
            // 有2个候选时做ratio test；只有1个候选时仅用distMax筛选
            if(knn[i].size()>=2 && m1.distance>=ratio*knn[i][1].distance) continue;
            {
                int idx2D=m1.queryIdx, idxRef=m1.trainIdx;
                if(idx2D<0 || idx2D>=N) continue;
                if(idxRef<0 || idxRef>=(int)valToRef.size()) continue;
                int rRef = valToRef[idxRef];
                pts2d.push_back(keys[idx2D].pt);
                const RefMPSnapshot &s = refSnaps[rRef];
                pts3d.emplace_back(s.Pw.x, s.Pw.y, s.Pw.z);
                qIdx.push_back(idx2D); refIdx.push_back(rRef);
                descDist.push_back((int)m1.distance);
                keptPairs++;
            }
        }
        // 实时匹配分数：归一化的匹配覆盖率（用于UI展示进入目标区域的可能性）
        float matchScore = 0.0f;
        if(N>0){
            matchScore = std::min(1.0f, (float)keptPairs / std::max(RELO_MATCH_SCORE_DIVISOR, (float)N*0.5f));
        }
        mRelocMatchScore.store(matchScore);
        if(pts3d.size()<12){
            std::vector<cv::DMatch> ms; matcher.match(desc, refDesc, ms);
            const int hardMax=50;
            for(const auto&m: ms){
                if(m.distance>hardMax) continue;
                int idx2D=m.queryIdx, idxRef=m.trainIdx;
                if(idx2D<0 || idx2D>=N) continue;
                if(idxRef<0 || idxRef>=(int)valToRef.size()) continue;
                int rRef=valToRef[idxRef];
                pts2d.push_back(keys[idx2D].pt);
                const RefMPSnapshot &s = refSnaps[rRef];
                pts3d.emplace_back(s.Pw.x, s.Pw.y, s.Pw.z);
                qIdx.push_back(idx2D); refIdx.push_back(rRef);
                descDist.push_back((int)m.distance);
            }
        }

        // 去重与裁剪：对相同参考索引仅保留距离最小者，并限制样本上限以降低RANSAC代价
        if(!refIdx.empty()){
            std::unordered_map<int, size_t> bestPosByRef; bestPosByRef.reserve(refIdx.size());
            for(size_t i=0;i<refIdx.size();++i){
                int r = refIdx[i];
                auto it = bestPosByRef.find(r);
                if(it==bestPosByRef.end()) bestPosByRef[r]=i;
                else{
                    size_t prev = it->second;
                    if(descDist[i] < descDist[prev]) bestPosByRef[r]=i;
                }
            }
            // 组装去重后的索引集合
            std::vector<size_t> keep; keep.reserve(bestPosByRef.size());
            for(auto &kv: bestPosByRef) keep.push_back(kv.second);
            // 按距离升序截断到上限
            const size_t MAX_PNP_SAMPLES = RELO_BG_PNP_MAX_SAMPLES;
            std::nth_element(keep.begin(), keep.begin()+std::min(keep.size(), MAX_PNP_SAMPLES), keep.end(),
                             [&](size_t a, size_t b){ return descDist[a] < descDist[b]; });
            if(keep.size() > MAX_PNP_SAMPLES) keep.resize(MAX_PNP_SAMPLES);
            // 重排到新的容器
            std::vector<cv::Point2f> n2d; n2d.reserve(keep.size());
            std::vector<cv::Point3f> n3d; n3d.reserve(keep.size());
            std::vector<int> nq; nq.reserve(keep.size());
            std::vector<int> nr; nr.reserve(keep.size());
            std::sort(keep.begin(), keep.end(), [&](size_t a, size_t b){ return descDist[a] < descDist[b]; });
            for(size_t id : keep){ n2d.push_back(pts2d[id]); n3d.push_back(pts3d[id]); nq.push_back(qIdx[id]); nr.push_back(refIdx[id]); }
            pts2d.swap(n2d); pts3d.swap(n3d); qIdx.swap(nq); refIdx.swap(nr);
        }

        bool alignedNow=false; int inliersCnt=0;
        if(mRelocCooldownFrames>0){ mRelocCooldownFrames--; continue; }
        
        // PnP求解策略：6+ 使用RANSAC；4-5 使用P3P/EPNP并做重投影验证，严禁在<6时进入RANSAC路径
        if(pts3d.size()>=4){
            std::vector<cv::Point3f> f3d; std::vector<cv::Point2f> f2d; std::vector<int> fq, fr;
            FilterPnPInputsSync(pts3d, pts2d, qIdx, refIdx, f3d, f2d, fq, fr);
            if(f3d.size()<4){
                continue;
            }
            cv::Mat K = mK; cv::Mat distCoeffs = cv::Mat::zeros(4,1,CV_32F);
            cv::Mat rvec, tvec; std::vector<int> inliers; bool ok=false;
            if(f3d.size()>=6){
                ok = SolvePnPSafe(f3d, f2d, K, distCoeffs, rvec, tvec, inliers);
                inliersCnt = (int)inliers.size();
            } else {
                // 4-5点：使用P3P/EPNP直接求解，随后做重投影误差验证，避免RANSAC的DLT断言
                try{
                    cv::Mat Kd; K.convertTo(Kd, CV_64F);
                    cv::Mat Dd; distCoeffs.convertTo(Dd, CV_64F);
                    int method = (f3d.size()==4? cv::SOLVEPNP_P3P : cv::SOLVEPNP_EPNP);
                    ok = cv::solvePnP(f3d, f2d, Kd, Dd, rvec, tvec, false, method);
                    if(ok){
                        // 计算重投影误差，阈值按像素（例如 5px）
                        cv::Mat R; cv::Rodrigues(rvec, R);
                        double sumErr=0.0; int cnt=(int)f3d.size();
                        for(size_t i=0;i<f3d.size();++i){
                            cv::Mat Pw = (cv::Mat_<double>(3,1) << f3d[i].x, f3d[i].y, f3d[i].z);
                            cv::Mat Pc = R*Pw + tvec;
                            double Z = Pc.at<double>(2);
                            if(Z<=1e-6){ sumErr += 1e6; continue; }
                            double u = Kd.at<double>(0,0)*Pc.at<double>(0)/Z + Kd.at<double>(0,2);
                            double v = Kd.at<double>(1,1)*Pc.at<double>(1)/Z + Kd.at<double>(1,2);
                            double du = u - f2d[i].x; double dv = v - f2d[i].y;
                            sumErr += std::sqrt(du*du+dv*dv);
                        }
                        double meanErr = sumErr / std::max(1, cnt);
                        if(meanErr>5.0){ ok=false; }
                        inliers.clear(); inliersCnt = ok? cnt : 0;
                    }
                }catch(const cv::Exception& e){ LOGE("RelocBG: PnP异常: %s", e.what()); ok=false; inliers.clear(); }
            }
            // 提高对齐阈值，要求至少30个inliers且置信度>=0.5，避免错误匹配
            // Reset后如果立即匹配到少量点（比如10-20个），很可能是错误匹配，不应该接受
            if(ok && inliersCnt>=TRACKING_RELOC_PNP_MIN_INLIERS){
                cv::Mat R; 
                try { cv::Rodrigues(rvec, R); } catch(const cv::Exception& e){ LOGE("RelocBG: Rodrigues异常: %s", e.what()); mRelocCooldownFrames=5; continue; }
                cv::Mat Tcw_map = cv::Mat::eye(4,4,CV_32F);
                R.copyTo(Tcw_map.rowRange(0,3).colRange(0,3));
                tvec.copyTo(Tcw_map.rowRange(0,3).col(3));
                
                if(!TcwSlam.empty()){
                    cv::Mat Twc_slam = TcwSlam.inv();
                    cv::Mat T_map_from_slam = Tcw_map * Twc_slam;
                    float conf = std::min(1.0f, inliersCnt/50.0f);
                    
                    // 根据内点确定地图ID
                    std::map<int, int> mapVotes;
                    for(int idx : inliers) {
                        if(idx >= 0 && idx < (int)fr.size()) {
                            int refSnapshotIdx = fr[idx];
                            if(refSnapshotIdx >= 0 && refSnapshotIdx < (int)refSnaps.size()) {
                                int mId = refSnaps[refSnapshotIdx].mapId;
                                mapVotes[mId]++;
                            }
                        }
                    }
                    
                    int bestMapId = -1;
                    int maxVote = 0;
                    int secondMaxVote = 0;
                    
                    for(auto& kv : mapVotes) {
                        if(kv.second > maxVote) {
                            secondMaxVote = maxVote;
                            maxVote = kv.second;
                            bestMapId = kv.first;
                        } else if(kv.second > secondMaxVote) {
                            secondMaxVote = kv.second;
                        }
                    }

                    // 仅在最佳地图优胜（内点>=15且多于第二名20%）且内点足够时发布
                    if (bestMapId != -1 && maxVote >= RELOC_MIN_INLIERS_FOR_ALIGN && (maxVote > secondMaxVote * 1.2)) {
                        // 使用平滑更新机制，减少单帧抖动
                        UpdateAlignmentSmooth(T_map_from_slam, inliersCnt, conf, ts);
                        
                        // 仍然发布对齐结果供主线程使用
                        PublishRelocAlignment(T_map_from_slam, inliersCnt, conf, ts, bestMapId);
                        //      inliersCnt, conf, bestMapId, maxVote, inliersCnt);
                        alignedNow = true;
                    } else {
                        //      inliersCnt, bestMapId, maxVote, secondMaxVote);
                    }
                }
            } else {
                 mRelocCooldownFrames = TRACKING_RELOC_COOLDOWN_FRAMES;
            }
        } else {
        }
    }
}

void Tracking::StartGlobalRelocThread()
{
    std::unique_lock<std::mutex> lk(mMutexReloc);
    if(mptGlobalReloc) return;
    mbRelocThreadStop = false;
    mRelocThreadSessionId++;  // 递增会话ID，使旧线程安全退出
    mptGlobalReloc = new std::thread(&Tracking::GlobalRelocLoop, this, mRelocThreadSessionId.load());
}

void Tracking::StopGlobalRelocThread()
{
    {
        std::unique_lock<std::mutex> lk(mMutexReloc);
        if(!mptGlobalReloc) return;
        mbRelocThreadStop = true;
        mRelocThreadSessionId++;  // 递增会话ID，强制任何被分离的旧线程立即退出
    }
    if(mptGlobalReloc){
        mCvReloc.notify_all();
        // 替换 join() 为 detach()，避免主线程在此阻塞
        mptGlobalReloc->detach();
        delete mptGlobalReloc;
        mptGlobalReloc = nullptr;
    }
}

void Tracking::SetRelocConfig(int topKWords, int maxCandidates, int matchChunk, int bgSleepUs,
                        int maxBindInliers, int maxProjBinds)
{
    // 设置重定位参数：topKWords/maxCandidates/matchChunk/bgSleepUs/maxBindInliers/maxProjBinds
    
    if(topKWords>0) mCfgTopKWords = topKWords;
    if(maxCandidates>0) mCfgMaxCandidates = maxCandidates;
    if(matchChunk>0) mCfgMatchChunk = matchChunk;
    {}//
    if(maxBindInliers>0) mCfgMaxBindInliers = maxBindInliers;
    if(maxProjBinds>0) mCfgMaxProjBinds = maxProjBinds;
    
    // 设置合理的默认值，确保后台线程不会过于频繁地运行
    {}// // 至少50ms间隔
    if(mCfgMaxCandidates > 10000) mCfgMaxCandidates = 10000; // 限制候选数量
}

void Tracking::ClearMapAlignment()
{
    std::unique_lock<std::mutex> lk(mMutexReloc);
    mbHaveMapAlign = false;
    mAlignConfidence = 0.0f;
    mLastAlignTs = 0.0;
    mT_map_from_slam.release();
    mSmoothedT_map_from_slam.release();
    mAlignUpdateCount = 0;
    mAlignSkipCounter = 0;
    mPendingMapId = -1;
    mPendingMapCount = 0;
    mNoCurMapLoadedInliersFrames = 0;
    mLastAcceptedAlignInliers = 0;
}

cv::Mat Tracking::GetMapAlignedPose(const cv::Mat &TcwSlam)
{
    std::unique_lock<std::mutex> lk(mMutexReloc);
    if(!mbHaveMapAlign || mT_map_from_slam.empty()) return TcwSlam.clone();
    return mT_map_from_slam * TcwSlam;
}

bool Tracking::HasLoadedMapData() const
{
    // 通过检查参考快照是否存在来判断是否已加载地图
    std::unique_lock<std::mutex> lk(mMutexReloc);
    return !mRefSnapshots.empty();
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetMap(Map *pMap)
{
    mpMap = pMap;
}

cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
    VT_PROFILE_FUNCTION();

    if(im.channels()==3)
    {
        if(mbRGB)
            cvtColor(im, mImGray, CV_RGB2GRAY);
        else
            cvtColor(im, mImGray, CV_BGR2GRAY);
    }
    else if(im.channels()==4)
    {
        if(mbRGB)
            cvtColor(im, mImGray, CV_RGBA2GRAY);
        else
            cvtColor(im, mImGray, CV_BGRA2GRAY);
    }
    else
    {
        im.copyTo(mImGray);
    }

    {
        VT_PROFILE_SCOPE("Tracking::FrameConstruction");
        if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
            mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mK,mDistCoef,mbf);
        else
            mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mK,mDistCoef,mbf);
    }

    Track();

    return mCurrentFrame.mTcw.clone();
}

void Tracking::Track()
{
    VT_PROFILE_FUNCTION();
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;
    if (mState == NOT_INITIALIZED)
    {
        VT_PROFILE_SCOPE("Tracking::MonocularInitialization");
        // 仅支持单目初始化
        // 初始化过程会创建KF和MapPoint并写入地图，需短暂持锁
        {
            std::unique_lock<std::mutex> lock(mpMap->mMutexMapUpdate);
            MonocularInitialization();
        }

        mpFrameDrawer->Update(this);

        if (mState != OK)
            return;
    }
    else
    {
        // 系统已初始化。跟踪帧。
        bool bOK;

        // 使用运动模型或重定位进行初始相机位姿估计（如果跟踪丢失）
        if(!mbOnlyTracking)
        {
            if(mState==OK)
            {
                // CheckReplacedInLastFrame 访问 MapPoint::GetReplaced()，需短暂持锁防止并发替换
                {
                    std::unique_lock<std::mutex> lock(mpMap->mMutexMapUpdate, std::try_to_lock);
                    if (lock.owns_lock()) {
                        CheckReplacedInLastFrame();
                    }
                }

                VT_PROFILE_SCOPE("Tracking::TrackWithMotionModel");
                bOK = TrackWithMotionModel();
                if(!bOK)
                {
                    VT_PROFILE_SCOPE("Tracking::TrackRefKF_Fallback");
                    bOK = TrackReferenceKeyFrame();
                }
            }
            else
            {
                VT_PROFILE_SCOPE("Tracking::Relocalization");
                bOK = Relocalization();
            }
        }
        else
        {
            // 定位模式：局部建图已停用
            if(mState==LOST)
            {
                bOK = Relocalization();
            }
            else
            {
                if(!mbVO)
                {
                    // 在上一帧中，我们在地图中跟踪了足够的地图点
                    bOK = TrackWithMotionModel();
                    if(!bOK)
                    {
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint*> vpMPsMM;
                    vector<bool> vbOutMM;
                    cv::Mat TcwMM;
                    bOKMM = TrackWithMotionModel();
                    if(bOKMM)
                    {
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }
                    // 跟踪丢失且不是VO模式时，尝试重定位
                    if(mCurrentFrame.mnId % 2 == 0)
                        bOKReloc = Relocalization();
                    else 
                        bOKReloc = false;

                    if(bOKMM && !bOKReloc)
                    {
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        if(mbVO)
                        {
                            for(int i =0; i<mCurrentFrame.N; i++)
                            {
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    else if(bOKReloc)
                    {
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // 如果我们有相机位姿和匹配的初始估计值，则跟踪局部地图。
        if(!mbOnlyTracking)
        {
            if(bOK)
            {
                VT_PROFILE_SCOPE("Tracking::TrackLocalMap");
                bOK = TrackLocalMap();
            }
        }
        else
        {
            // mbVO 为 true 表示地图中很少有与地图点匹配的点。我们无法检索
            // 局部地图，因此不执行 TrackLocalMap()。一旦系统重新定位相机，我们将再次使用局部地图。
            if(bOK && !mbVO)
            {
                VT_PROFILE_SCOPE("Tracking::TrackLocalMap_VO");
                bOK = TrackLocalMap();
            }
        }

        if(bOK) {
            mState = OK;
        }
        else {
            mState=LOST;
        }

        // 更新绘制器
        mpFrameDrawer->Update(this);

        // 如果跟踪良好，检查是否插入关键帧
        if(bOK)
        {
            // 更新运动模型
            if(!mLastFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();

            //mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            // 清理VO匹配：保留来自已加载地图的匹配点（它们通常没有KF观测）
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP) continue;
                if(pMP->mbFromLoadedMap) continue; // 保留，便于UI高亮与后续融合
                if(pMP->Observations()<1)
                {
                    mCurrentFrame.mvbOutlier[i] = false;
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                }
            }

            // 删除临时地图点
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

            // 检查是否需要插入新关键帧
            if(NeedNewKeyFrame())
            {
                VT_PROFILE_SCOPE("Tracking::CreateNewKeyFrame");
                CreateNewKeyFrame();
            }

            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        // 更新连续丢失帧计数
        if(mState == OK)
            mConsecutiveLostFrames = 0;
        else if(mState == LOST)
            mConsecutiveLostFrames++;

        // 更新智能调度状态，供后台重定位线程读取
        mTrackingOK.store(mState == OK);

        // 如果相机在初始化后不久丢失，则重置
        if(mState==LOST)
        {
            // 初始化后的前20帧内，即使丢失也不立即重置，给予重定位及加载点绑定的机会
            if(mpMap->KeyFramesInMap()<=5 && mCurrentFrame.mnId > mnLastKeyFrameId + 20)
            {
                mpSystem->Reset();
                return;
            }
            
            // 多地图模式：如果丢失时间过长，创建新地图（子地图）
            // 必须保证地图有足够关键帧，防止初始扫描阶段点云被意外清空
            if (mConsecutiveLostFrames > TRACKING_LOST_FRAMES_FOR_NEW_MAP && mpMap->KeyFramesInMap() > 10)
            {
                // 冷却保护：限频防止高性能机器反复触发CreateNewMap导致主线程卡死
                // 低性能机器因每帧间隔大反而不易触发
                if (mLastNewMapFrameId > 0 &&
                    mCurrentFrame.mnId < mLastNewMapFrameId + TRACKING_NEW_MAP_COOLDOWN_FRAMES)
                {
                    // 仍在冷却期内，不触发 CreateNewMap；让丢失计数继续累积，
                    // 等本次相机视野稳定后或冷却结束后再做处理。
                    LOGD("跟踪: 处于子地图冷却期 (距上次创建 %u 帧 < %d)，跳过 CreateNewMap",
                         mCurrentFrame.mnId - mLastNewMapFrameId, TRACKING_NEW_MAP_COOLDOWN_FRAMES);
                }
                else
                {
                    //      mConsecutiveLostFrames, TRACKING_LOST_FRAMES_FOR_NEW_MAP);
                    mpSystem->CreateNewMap();
                    mLastNewMapFrameId = mCurrentFrame.mnId;
                    mConsecutiveLostFrames = 0;
                    return;
                }
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
        
        // 更新内点数供后台线程智能调度使用（TrackLocalMap 已统计 mnMatchesInliers）
        mLastTrackingInliers.store(mnMatchesInliers);

        cv::Mat tempTcw = mCurrentFrame.mTcw.clone();
        cv::Mat tempDesc = mCurrentFrame.mDescriptors.clone();
        std::vector<cv::KeyPoint> tempKeysUn = mCurrentFrame.mvKeysUn;
        
        {
            std::unique_lock<std::mutex> lk(mMutexReloc);
            cv::swap(mLastTcwSlam, tempTcw);
            cv::swap(mLastDesc, tempDesc);
            mLastKeysUn.swap(tempKeysUn);
            mLastN = mCurrentFrame.N;
            mLastTimestamp = mCurrentFrame.mTimeStamp;
            // 生产新快照并唤醒后台线程（无延时）
            mSnapSeqProduced++;
            mCvReloc.notify_all();
        }
    }

    // 存储帧位姿信息以便之后检索完整的相机轨迹。
    if(!mCurrentFrame.mTcw.empty())
    {
        if(mCurrentFrame.mpReferenceKF){
            // 在访问 KeyFrame 方法之前检查其有效性
            KeyFrame* pRefKF = mCurrentFrame.mpReferenceKF;
            if(!pRefKF || pRefKF->isBad())
            {
                // 如果参考关键帧无效，使用单位矩阵作为默认值
                mlRelativeFramePoses.push_back(cv::Mat::eye(4,4,CV_32F));
                mlpReferences.push_back(nullptr);
            }
            else
            {
                try {
                    cv::Mat Tcr = mCurrentFrame.mTcw*pRefKF->GetPoseInverse();
                    mlRelativeFramePoses.push_back(Tcr);
                    mlpReferences.push_back(mpReferenceKF);
                } catch (...) {
                    // 如果访问失败（例如 KeyFrame 已被销毁），使用默认值
                    mlRelativeFramePoses.push_back(cv::Mat::eye(4,4,CV_32F));
                    mlpReferences.push_back(nullptr);
                }
            }
        } else {
            mlRelativeFramePoses.push_back(mlRelativeFramePoses.empty()?cv::Mat::eye(4,4,CV_32F):mlRelativeFramePoses.back());
            mlpReferences.push_back(nullptr);
        }
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    else
    {
        // 如果跟踪丢失，这可能发生
        // 确保列表非空再访问 back()，防止 ClearTrackingState 后崩溃
        if(!mlRelativeFramePoses.empty()) {
            mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        } else {
            mlRelativeFramePoses.push_back(cv::Mat::eye(4,4,CV_32F));
        }
        if(!mlpReferences.empty()) {
            mlpReferences.push_back(mlpReferences.back());
        } else {
            mlpReferences.push_back(nullptr);
        }
        if(!mlFrameTimes.empty()) {
            mlFrameTimes.push_back(mlFrameTimes.back());
        } else {
            mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        }
        mlbLost.push_back(mState==LOST);
    }

}

void Tracking::MonocularInitialization()
{

    if(!mpInitializer)
    {
        // 设置参考帧
        if(mCurrentFrame.mvKeys.size()>INITIALIZER_MIN_TRACKED_POINTS)
        {
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete mpInitializer;

            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }
    }
    else
    {
        // 尝试初始化
        if((int)mCurrentFrame.mvKeys.size()<=INITIALIZER_MIN_TRACKED_POINTS)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        // 查找对应点
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // 检查是否有足够的对应点
        if(nmatches<TRACKING_INIT_MIN_MATCHES)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        {
            float distSum = 0.0f;
            int validCount = 0;
            for(size_t i=0, iend=mvIniMatches.size(); i<iend; i++) {
                if(mvIniMatches[i]>=0) {
                    const cv::KeyPoint &kp1 = mInitialFrame.mvKeysUn[i];
                    const cv::KeyPoint &kp2 = mCurrentFrame.mvKeysUn[mvIniMatches[i]];
                    distSum += std::abs(kp1.pt.x - kp2.pt.x) + std::abs(kp1.pt.y - kp2.pt.y);
                    validCount++;
                }
            }
            if(validCount > 0 && (distSum / validCount) < INITIALIZER_MIN_PARALLAX_PX) {
                // 超时保护：超过设定时间仍未成功初始化时，强制尝试初始化（即使视差不足）
                if(mCurrentFrame.mTimeStamp - mLastInitAttemptTime > INITIALIZER_TIMEOUT_SEC) {
                    // 超时强制初始化：跳过视差检查
                } else {
                    return; // 视差不足，等待相机移动充足基线再初始化
                }
            }
        }

        mLastInitAttemptTime = mCurrentFrame.mTimeStamp;

        cv::Mat Rcw; // 当前相机旋转
        cv::Mat tcw; // 当前相机平移
        vector<bool> vbTriangulated; // 三角化的对应点 (mvIniMatches)

        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // 设置帧位姿
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
}

void Tracking::CreateInitialMapMonocular()
{
    // 确保初始帧和当前帧有有效的位姿才能创建KeyFrame
    if(mInitialFrame.mTcw.empty() || mInitialFrame.mTcw.rows < 4 || mInitialFrame.mTcw.cols < 4){
        LOGE("创建初始单目地图: 初始帧位姿无效，无法创建关键帧");
        return;
    }
    if(mCurrentFrame.mTcw.empty() || mCurrentFrame.mTcw.rows < 4 || mCurrentFrame.mTcw.cols < 4){
        LOGE("创建初始单目地图: 当前帧位姿无效，无法创建关键帧");
        return;
    }
    
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);


    // pKFini->ComputeBoW();
    // pKFcur->ComputeBoW();

    // 插入关键帧到地图中
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // 创建地图点并与关键帧关联
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //创建地图点。
        cv::Mat worldPos(mvIniP3D[i]);

        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //填充当前帧结构
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //添加到地图
        mpMap->AddMapPoint(pMP);
    }

    // 更新连接关系
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // 初始化BA优化：恢复20次迭代，确保初始地图精度，提高后续帧跟踪稳定性
    Optimizer::GlobalBundleAdjustemnt(mpMap, 20);

    // 设置中位深度为1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<INITIALIZER_MIN_TRIANGULATED)
    {
        Reset();
        return;
    }

    // 缩放初始基线
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // 缩放点
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
            pMP->UpdateNormalAndDepth();
        }
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    //mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;
}

void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}


bool Tracking::TrackReferenceKeyFrame()
{
    // 关键帧为空或已坏则不进行参考跟踪，避免互斥锁空指针崩溃
    if(!mpReferenceKF || mpReferenceKF->isBad()) return false;
    if(mLastFrame.mTcw.empty()) return false; // 防御性判空
    
    // 计算词袋向量
    // mCurrentFrame.ComputeBoW();

    // 首先对参考关键帧进行ORB匹配
    // 如果找到足够多的匹配点，我们就设置PnP求解器
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

    int nmatches = matcher.SearchByHBST(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

    if(nmatches<TRACKING_REFKF_MIN_MATCHES)
        return false;

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw);

    Optimizer::PoseOptimization(&mCurrentFrame);

    // 丢弃离群值
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            //  加载的地图点没有Observations，但仍应计入匹配数
            else if(mCurrentFrame.mvpMapPoints[i]->mbFromLoadedMap ||
                    mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    return nmatchesMap>=TRACKING_SUCCESS_LOADED;
}

void Tracking::UpdateLastFrame()
{
    // 根据参考关键帧更新位姿
    KeyFrame* pRef = mLastFrame.mpReferenceKF;

    // 确保列表非空再访问 back()，防止 ClearTrackingState 后崩溃
    cv::Mat Tlr;
    if(!mlRelativeFramePoses.empty()) {
        Tlr = mlRelativeFramePoses.back();
    } else {
        Tlr = cv::Mat::eye(4,4,CV_32F);
    }

    // 确保参考关键帧有效
    if(!pRef) {
        return;
    }

    cv::Mat T_ref_pose;
    {
        std::unique_lock<std::mutex> lock(mpMap->mMutexMapUpdate);
        if(pRef->isBad()) {
            return;
        }
        T_ref_pose = pRef->GetPose();
    }

    mLastFrame.SetPose(Tlr * T_ref_pose);

    // 单目模式不需要创建视觉里程计地图点
    if(mnLastKeyFrameId==mLastFrame.mnId || !mbOnlyTracking)
        return;
}

bool Tracking::TrackWithMotionModel()
{
    // 防御性检查：确保跟踪状态正常，防止 Reset 后
    // mLastFrame.mpReferenceKF 悬空指针导致的崩溃
    if(mState != OK)
        return false;
    if(mLastFrame.mTcw.empty())
        return false;

    ORBmatcher matcher(0.9,true);

    // 根据参考关键帧更新上一帧位姿
    // 如果处于定位模式则创建"视觉里程计"点
    UpdateLastFrame();

    if(mVelocity.empty())
        mCurrentFrame.SetPose(mLastFrame.mTcw.clone());
    else
        mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // 投影上一帧中看到的点
    int th = TRACKING_MOTION_SEARCH_TH; // 单目模式使用15像素阈值
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,true);

    // 如果匹配点较少，使用更宽的窗口搜索
    if(nmatches<TRACKING_MOTION_MIN_MATCHES)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR);
    }

    if(nmatches<TRACKING_MOTION_MIN_MATCHES)
        return false;

    // 使用所有匹配点优化帧位姿
    Optimizer::PoseOptimization(&mCurrentFrame);

    // 丢弃离群点
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            //  加载的地图点没有Observations，但仍应计入匹配数
            else if(mCurrentFrame.mvpMapPoints[i]->mbFromLoadedMap || 
                    mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }    

    if(mbOnlyTracking)
    {
        mbVO = nmatchesMap<10;
        return nmatches>20;
    }

    return nmatchesMap>=TRACKING_SUCCESS_LOADED;
}

bool Tracking::TrackLocalMap()
{
    UpdateLocalMap();
    // 在新匹配前清除匹配标志
    for(MapPoint* p : mvpLocalMapPoints){ if(p) p->mbMatchedInCurrentFrame = false; }
    // 记录局部地图组成
    {
        int localLoaded=0, localWithDesc=0; int localTotal=(int)mvpLocalMapPoints.size();
        for(MapPoint* p : mvpLocalMapPoints){
            if(!p || p->isBad()) continue;
            if(p->mbFromLoadedMap){ localLoaded++; if(!p->GetDescriptor().empty()) localWithDesc++; }
        }
    }

    // 避免遍历全局已加载的地图点，这会导致严重卡顿且过早进行地图匹配。
    // 必须等待SLAM建立一定规模的新地图（稳定）后，再尝试混合已加载的地图点。
    if((int)mvpLocalMapPoints.size()<50)
    {
        // 此时跳过全局搜索，不仅节省性能，也符合"先建图稳了再匹配"的逻辑
        bool bSkipGlobalWithLoadedMap = (mpMap->MapPointsInMap() > 5000 && mpMap->KeyFramesInMap() < 20);
        
        if(!bSkipGlobalWithLoadedMap) 
        {
            const vector<MapPoint*> allMPs = mpMap->GetAllMapPoints();
            int added=0;
            for (size_t i = 0; i < allMPs.size() && added < LOCAL_MAP_SUPPLEMENT_COUNT; i++)
            {
                MapPoint *p = allMPs[i];
                if (!p || p->isBad())
                    continue;
                if (p->mnTrackReferenceForFrame == mCurrentFrame.mnId)
                    continue;

                mvpLocalMapPoints.push_back(p);
                p->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                added++;
            }
        }
    }

    SearchLocalPoints();

    // 优化位姿
    Optimizer::PoseOptimization(&mCurrentFrame);
    
    // 若地图中没有任何关键帧或当前没有参考关键帧，则在匹配充分时自举一个关键帧，恢复/继续建图
    {
        const int nKFs = mpMap->KeyFramesInMap();
        if((nKFs==0 || mpReferenceKF==nullptr)){
            int validMatches = 0;
            for(int i=0;i<mCurrentFrame.N;i++){ if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i]) validMatches++; }
            if(validMatches>=20){
                // 确保mCurrentFrame有有效的位姿才能创建KeyFrame
                // Reset后或LoadMap后，mTcw可能为空或无效，必须检查
                if(mCurrentFrame.mTcw.empty() || mCurrentFrame.mTcw.rows < 4 || mCurrentFrame.mTcw.cols < 4){
                    LOGW("局部地图跟踪: 无法创建关键帧，当前帧位姿无效 (empty=%d, rows=%d, cols=%d)", 
                         mCurrentFrame.mTcw.empty()?1:0,
                         mCurrentFrame.mTcw.empty()?0:mCurrentFrame.mTcw.rows,
                         mCurrentFrame.mTcw.empty()?0:mCurrentFrame.mTcw.cols);
                    // 不创建KeyFrame，等待下一帧有有效位姿
                } else {
                    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);
                    for(int i=0; i<mCurrentFrame.N; i++){
                        MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                        if(!pMP || pMP->isBad()) continue;
                        pKFcur->AddMapPoint(pMP, i);
                        pMP->AddObservation(pKFcur, i);
                    }
                    // pKFcur->ComputeBoW();
                    mpMap->AddKeyFrame(pKFcur);
                    pKFcur->UpdateConnections();
                    mpLocalMapper->InsertKeyFrame(pKFcur);
                    mpReferenceKF = pKFcur;
                    mCurrentFrame.mpReferenceKF = pKFcur;
                    mpLastKeyFrame = pKFcur;
                    mnLastKeyFrameId = mCurrentFrame.mnId;
                    LOGD("局部地图跟踪: 自举恢复建图 匹配=%d", validMatches);
                }
            }
        }
    }
    // 消费异步对齐结果（如果更好/更新），然后进行柔和融合
    {
        if(mMapSwitchCooldownFrames > 0) mMapSwitchCooldownFrames--;
        RelocAlignResult res;
        if(TryConsumeRelocAlignment(res)){
            // 使用Config.h中的参数，检查对齐结果的置信度和inliers数量
            // 只有高质量匹配才接受，避免Reset后立即匹配到错误区域
            const float minConfidence = RELOC_MIN_CONFIDENCE_FOR_ALIGN;
            const int minInliers = RELOC_MIN_INLIERS_FOR_ALIGN;
            if(res.confidence >= minConfidence && res.inliers >= minInliers){
                const int curMapId = mnCurrentMapId.load();

                auto applyAlign = [&](const RelocAlignResult &a){
                    std::unique_lock<std::mutex> lk(mMutexReloc);
                    mT_map_from_slam = a.T_map_from_slam.clone();
                    mbHaveMapAlign = true;
                    mAlignConfidence = a.confidence;
                    mLastAlignTs = a.ts;
                    mnCurrentMapId = a.mapId;
                    mLastAcceptedAlignInliers = a.inliers;
                    mPendingMapId = -1;
                    mPendingMapCount = 0;
                    mNoCurMapLoadedInliersFrames = 0;

                    if(mAlignConfidence >= 0.9f && !mCurrentFrame.mTcw.empty()){
                        cv::Mat Tcw_map = mT_map_from_slam * mCurrentFrame.mTcw;
                        cv::Mat Rcw = Tcw_map.rowRange(0,3).colRange(0,3);
                        cv::Mat tcw = Tcw_map.rowRange(0,3).col(3);
                        cv::Mat refDescStrong = mRefDesc;
                        std::vector<RefMPSnapshot> refSnapsStrong = mRefSnapshots;
                        int activeMapId = a.mapId;
                        lk.unlock();
                        int strongBinds = 0;
                        for(size_t i=0;i<refSnapsStrong.size();++i){
                            const RefMPSnapshot &s = refSnapsStrong[i];
                            if(s.mapId != activeMapId) continue;

                            cv::Mat Pw = (cv::Mat_<float>(3,1) << s.Pw.x, s.Pw.y, s.Pw.z);
                            cv::Mat Pc = Rcw*Pw + tcw;
                            float Z = Pc.at<float>(2); if(Z<=0) continue;
                            float u = mCurrentFrame.fx*Pc.at<float>(0)/Z + mCurrentFrame.cx;
                            float v = mCurrentFrame.fy*Pc.at<float>(1)/Z + mCurrentFrame.cy;
                            if(u<0 || u>=mCurrentFrame.mnMaxX || v<0 || v>=mCurrentFrame.mnMaxY) continue;

                            const float searchRadius = RELOC_PROJ_SEARCH_RADIUS;
                            vector<size_t> vIndices = mCurrentFrame.GetFeaturesInArea(u, v, searchRadius);
                            if(vIndices.empty()) continue;

                            int bestIdx=-1;
                            float bestDist2 = searchRadius * searchRadius;

                            for(size_t j : vIndices){
                                if(mCurrentFrame.mvpMapPoints[j]) continue;
                                const cv::KeyPoint &kp = mCurrentFrame.mvKeysUn[j];
                                float du = kp.pt.x - u, dv = kp.pt.y - v;
                                float d2 = du*du+dv*dv;
                                if(d2 < bestDist2){ bestDist2=d2; bestIdx=(int)j; }
                            }

                            if(bestIdx>=0){
                                MapPoint* pBind = nullptr;
                                {
                                    std::unique_lock<std::mutex> lk2(mMutexReloc);
                                    if(i < mRefIdxToMP.size()) pBind = mRefIdxToMP[i];
                                }
                                if(pBind && !pBind->isBad()){
                                    mCurrentFrame.mvpMapPoints[bestIdx] = pBind;
                                    pBind->mbMatchedInCurrentFrame = true;
                                    strongBinds++;
                                    if(strongBinds >= mCfgMaxProjBinds) break;
                                }
                            }
                        }
                    }
                };

                if(!mbHaveMapAlign || res.mapId == curMapId){
                    applyAlign(res);
                } else {
                    if(mMapSwitchCooldownFrames > 0){
                        mPendingMapId = -1;
                        mPendingMapCount = 0;
                    } else {
                        if(mPendingMapId != res.mapId){
                            mPendingMapId = res.mapId;
                            mPendingMapCount = 1;
                            mPendingAlign = res;
                        } else {
                            mPendingMapCount++;
                            if(res.inliers > mPendingAlign.inliers || res.confidence > mPendingAlign.confidence){
                                mPendingAlign = res;
                            }
                        }

                        const int needCount = 3;
                        const int needInliers = std::max(60, (int)std::ceil(std::max(1.0f, (float)mLastAcceptedAlignInliers) * 1.25f));
                        if(mPendingMapCount >= needCount && mPendingAlign.inliers >= needInliers){
                            const int newMapId = mPendingAlign.mapId;
                            const int confirmCount = mPendingMapCount;
                            const int confirmInliers = mPendingAlign.inliers;
                            applyAlign(mPendingAlign);
                            mMapSwitchCooldownFrames = 20;
                            //      curMapId, newMapId, confirmCount, confirmInliers);
                        }
                    }
                }
            } else {
                //     res.inliers, res.confidence, minInliers, minConfidence);
                // 不清除已有对齐状态，但拒绝新的低质量对齐
            }
        }
    }
    // 如果我们有稳定的地图对齐，轻轻融合它以减少漂移并提供稳定的地图位姿
    if(mbHaveMapAlign && !mCurrentFrame.mTcw.empty()){
        std::unique_lock<std::mutex> lk(mMutexReloc);
        if(!mT_map_from_slam.empty()){
            cv::Mat Tcw_map = mT_map_from_slam * mCurrentFrame.mTcw;
        }
    }
    // 利用快照进行快速绑定，避免只匹配一帧后就中断
    BindLoadedMapPointsUsingSnapshots();
    {
        int bindCnt=0; for(auto *p : mCurrentFrame.mvpMapPoints){ if(p && !p->isBad()) bindCnt++; }
    }
    // 对齐存在时，连续失败的宽限策略：如果上帧成功，这一帧边缘内点稍低允许通过一次
    if(mbHaveMapAlign){
        if(mnMatchesInliers>=CONSECUTIVE_FAIL_INLIERS_BASELINE) mConsecutiveFail=0; else mConsecutiveFail++;
    } else {
        mConsecutiveFail=0;
    }
    mnMatchesInliers = 0;

    // 统计匹配到加载点的数量，作为置信度来源
    int mnLoadedMapInliers = 0;
    int mnLoadedMapInliersCurMap = 0;
    const int curMapIdForCount = mnCurrentMapId.load();

    // 更新地图点统计信息
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                mCurrentFrame.mvpMapPoints[i]->mbMatchedInCurrentFrame = true;
                if(mCurrentFrame.mvpMapPoints[i]->mbFromLoadedMap){
                    mnLoadedMapInliers++;
                    if(mCurrentFrame.mvpMapPoints[i]->mnMapId == curMapIdForCount)
                        mnLoadedMapInliersCurMap++;
                }
                if(!mbOnlyTracking)
                {
                    // 加载的地图点没有Observations，但仍应计入匹配数
                    // 对于加载的地图点，跳过Observations检查
                    if(mCurrentFrame.mvpMapPoints[i]->mbFromLoadedMap || 
                       mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            // 单目模式不需要特殊处理

        }
    }

    // 进一步确保：当前帧中来自已加载地图且非外点的匹配，统一标记为本帧已匹配，便于UI绿色高亮
    for(int i=0; i<mCurrentFrame.N; ++i)
    {
        MapPoint* p = mCurrentFrame.mvpMapPoints[i];
        if(!p) continue;
        if(mCurrentFrame.mvbOutlier[i]) continue;
        if(p->mbFromLoadedMap) p->mbMatchedInCurrentFrame = true;
    }

    if(mbHaveMapAlign){
        if(mnLoadedMapInliersCurMap < 5) mNoCurMapLoadedInliersFrames++; else mNoCurMapLoadedInliersFrames = 0;
    } else {
        mNoCurMapLoadedInliersFrames = 0;
    }

    // 决定跟踪是否成功
    // 大幅放宽跟踪成功条件，提高稳定性，避免过早丢失
    int thStrict = TRACKING_SUCCESS_STRICT;  // 从50降至30
    int thLoose = TRACKING_SUCCESS_LOOSE;   // 从25降至15
    int thLoaded = TRACKING_SUCCESS_LOADED;  // 从15降至10，加载点阈值
    
    // 若已有地图对齐，进一步放宽阈值，确保持续跟踪
    if(mbHaveMapAlign){ thStrict = ALIGNED_STRICT_INLIERS_OVERRIDE; }
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<thStrict)
        return false;

    if(mbHaveMapAlign){
        // 对齐后极度宽松的条件，确保不轻易丢失
        if(mnMatchesInliers>=thLoose) return true;
        if(mnLoadedMapInliers >= thLoaded) return true;  // 新增：加载点也可以判定成功
        // 宽限：已对齐且连续失败次数不超过3时放行（从2增至3）
        if(mConsecutiveFail<=TRACKING_MAX_CONSECUTIVE_FAIL) return true;
        return false;
    }else{
        //  未对齐时，如果有足够的加载点匹配，也应该认为跟踪成功
        if(mnLoadedMapInliers >= thLoaded) return true;
        return mnMatchesInliers >= thStrict;
    }
}


bool Tracking::NeedNewKeyFrame()
{
    if(mbOnlyTracking)
        return false;

    // 如果局部建图被回环闭合冻结，则不插入关键帧
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // 如果从上次重定位以来没有经过足够多的帧，则不插入关键帧
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
        return false;

    // 参考关键帧中跟踪的地图点
    int nMinObs = REFKF_MIN_OBSERVATIONS;
    if(nKFs<=2)
        nMinObs=2;
    if(mpReferenceKF==nullptr) return false;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // 局部建图接受关键帧吗？
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // 单目模式不需要检查近距离点
    bool bNeedToInsertClose = false;

    // 阈值
    float thRefRatio = TRACKING_KF_REF_RATIO;
    if(nKFs<2)
        thRefRatio = TRACKING_KF_NEWMAP_RATIO;

    if(mSensor==System::MONOCULAR)
        thRefRatio = TRACKING_KF_MONO_RATIO;

    // 条件1a：从上次关键帧插入以来已过去超过"MaxFrames"
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // 条件1b：已过去超过"MinFrames"且局部建图空闲
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
    // 初期建图特例：如果地图中只有很少关键帧(<=2)，强制放宽闲置要求，允许频繁插入以迅速扩大地图
    const bool c1_init = (nKFs<=2 && mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames);
    // 条件1c：跟踪较弱（单目模式不适用）
    const bool c1c = false;
    // 条件2：与参考关键帧相比跟踪点较少。与地图匹配相比有很多视觉里程计。
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>=TRACKING_SUCCESS_LOADED);

    if ((c1a || c1b || c1c || c1_init) && c2)
    {
        // 如果建图接受关键帧，则插入关键帧。
        // 否则发送信号中断BA
        if (bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if (mSensor != System::MONOCULAR || nKFs <= 2)
            {
                if (mpLocalMapper->KeyframesInQueue() < 3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    if(!mpLocalMapper->SetNotStop(true))
        return;

    // 确保mCurrentFrame有有效的位姿才能创建KeyFrame
    // Reset后或LoadMap后，mTcw可能为空或无效
    if (mCurrentFrame.mTcw.empty() || mCurrentFrame.mTcw.rows < 4 || mCurrentFrame.mTcw.cols < 4)
    {
        LOGW("创建新关键帧: 无法创建关键帧，当前帧位姿无效 (empty=%d, rows=%d, cols=%d)",
             mCurrentFrame.mTcw.empty() ? 1 : 0,
             mCurrentFrame.mTcw.empty() ? 0 : mCurrentFrame.mTcw.rows,
             mCurrentFrame.mTcw.empty() ? 0 : mCurrentFrame.mTcw.cols);
        mpLocalMapper->SetNotStop(false);
        return;
    }

    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints()
{
    // 不搜索已经匹配的地图点
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    // 投影帧中的点并检查其可见性
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // 投影（这会填充用于匹配的MapPoint变量）
        if(mCurrentFrame.isInFrustum(pMP,FRUSTUM_VISIBILITY_TH))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th;

        // 动态搜索半径：根据跟踪状态自适应调整
        if (mState == LOST) {
            // 丢失状态：用最大搜索半径提高找回概率
            th = TRACKING_LOCAL_SEARCH_TH_LOST;
        } else if (mCurrentFrame.mnId < mnLastRelocFrameId + 10) {
            // 重定位后短期内：中等搜索半径快速建立匹配
            th = TRACKING_LOCAL_SEARCH_TH_RELOC;
        } else {
            // 正常跟踪：标准搜索半径
            th = TRACKING_LOCAL_SEARCH_TH;
        }

        // 更新动态搜索半径成员变量，供后续逻辑引用
        mDynamicSearchTh = static_cast<float>(th);

        // 先进行常规描述子匹配
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);

        // 对从已加载地图来的、无描述子的点，基于投影位置进行近邻特征匹配
        // 但避免与已有描述子匹配冲突
        const int totalKeys = mCurrentFrame.N;
        int additionalMatches = 0;
        // 大幅提升额外匹配上限，确保充分利用加载的地图点，维持稳定跟踪
        const int maxAdditionalMatches = mbHaveMapAlign ? 500 : 200;
        
        for(size_t idx=0; idx<mvpLocalMapPoints.size(); idx++)
        {
            if(additionalMatches >= maxAdditionalMatches) break;
            
            MapPoint* pMP = mvpLocalMapPoints[idx];
            if(!pMP || pMP->isBad()) continue;
            if(!pMP->mbFromLoadedMap) continue;
            if(!pMP->GetDescriptor().empty()) continue; // 已有描述子交给常规匹配
            if(!pMP->mbTrackInView) continue; // 需要在视野内

            const float u = pMP->mTrackProjX;
            const float v = pMP->mTrackProjY;
            const int predictedLevel = pMP->mnTrackScaleLevel;
            // 在已对齐时适当放宽半径，提升召回；否则使用默认
            float radiusScale = mbHaveMapAlign ? 1.8f : 1.0f;
            const float baseRadius = radiusScale * th * mCurrentFrame.mvScaleFactors[predictedLevel];

            int bestIdx = -1;
            float bestDist2 = 1e10f;

            // 使用网格搜索代替遍历所有特征点 (O(N) -> O(1))
            vector<size_t> vIndices = mCurrentFrame.GetFeaturesInArea(u, v, baseRadius, predictedLevel-1, predictedLevel+1);

            for(size_t i : vIndices)
            {
                if(mCurrentFrame.mvpMapPoints[i]) continue; // 已匹配
                const cv::KeyPoint &kp = mCurrentFrame.mvKeysUn[i];
                
                const float du = kp.pt.x - u;
                const float dv = kp.pt.y - v;
                const float dist2 = du*du + dv*dv;
                
                if(dist2 < bestDist2 && dist2 < baseRadius*baseRadius)
                {
                    bestDist2 = dist2;
                    bestIdx = i;
                }
            }
            if (bestIdx >= 0)
            {
                mCurrentFrame.mvpMapPoints[bestIdx] = pMP;
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                additionalMatches++;
            }
        }
        
    }
}

void Tracking::UpdateLocalMap()
{
    // 这是为了可视化
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // 更新
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                // 载入地图后，已加载点往往缺少关键帧观测，
                // 为了提升召回，不再按 Observations 进行过滤
                
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
    
    // 限制局部地图点数量，避免过多低质量点影响性能
    // 大幅提升上限以支持加载地图的重定位和持续跟踪
    if((int)mvpLocalMapPoints.size() > TRACKING_MAX_LOCAL_MAP_POINTS)
    {
        // 1. 快速分区：已加载的地图点 mbFromLoadedMap 优先（无需完整排序），O(N)
        auto pivot = std::partition(mvpLocalMapPoints.begin(), mvpLocalMapPoints.end(),
                                    [](const MapPoint* p) { return p->mbFromLoadedMap; });

        // 2. 判断已加载点数量是否已经超限
        int numLoaded = std::distance(mvpLocalMapPoints.begin(), pivot);
        if(numLoaded < TRACKING_MAX_LOCAL_MAP_POINTS)
        {
            // 如果已加载点少于限制，则对于普通点进行 O(N) 的 partial selection
            std::nth_element(pivot, mvpLocalMapPoints.begin() + TRACKING_MAX_LOCAL_MAP_POINTS, mvpLocalMapPoints.end(),
                             [](const MapPoint* a, const MapPoint* b) {
                                 // nObs 是原子变量，直接通过 Observations() 获取，零锁开销且极快
                                 return a->Observations() > b->Observations();
                             });
        }
        mvpLocalMapPoints.resize(TRACKING_MAX_LOCAL_MAP_POINTS);
    }
    
}


void Tracking::UpdateLocalKeyFrames()
{
    // 每个地图点都会投票给观察到它的关键帧
    map<KeyFrame*,int> keyframeCounter;
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                pMP->ShareObservations(keyframeCounter);
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // 所有观察到地图点的关键帧都包含在局部地图中。同时检查哪个关键帧共享最多的点
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // 还包括一些尚未包含的关键帧，它们是已包含关键帧的邻居 (使用安全的索引迭代以防止 vector 重分配导致迭代器失效崩溃)
    const size_t nMaxLocalKFs = MAX_LOCAL_KEYFRAMES;
    for(size_t i=0; i<mvpLocalKeyFrames.size(); i++)
    {
        // 限制关键帧数量
        if(mvpLocalKeyFrames.size() > nMaxLocalKFs)
            break;

        KeyFrame* pKF = mvpLocalKeyFrames[i];

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(COVISIBILITY_NEIGHBOR_COUNT);

        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                // 此处不能break，否则提前终止外层for循环
                // 导致其余关键帧的邻居/孩子/父节点无法加入局部地图
            }
        }

    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization()
{

    // 当跟踪丢失时执行重定位
    // 跟踪丢失：查询关键帧数据库以获取重定位的候选关键帧
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

    if(vpCandidateKFs.empty())
        return false;

    int nKFs = vpCandidateKFs.size();

    // 我们首先对每个候选帧进行ORB匹配
    // 如果找到足够多的匹配点，我们就设置PnP求解器
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    // 使用 vector<int> 替代 vector<bool> 以避免多线程并发写导致的竞争问题
    vector<int> vbDiscarded;
    vbDiscarded.resize(nKFs, 0);

    int nCandidates=0;


    // 限制最大候选数量，防止计算量过大导致卡死
    const int MAX_RELOC_CANDIDATES = 3;
    if(nKFs > MAX_RELOC_CANDIDATES) {
        // 简单截断，因为候选帧通常按BoW分数排序
        nKFs = MAX_RELOC_CANDIDATES; 
        vpCandidateKFs.resize(MAX_RELOC_CANDIDATES);
        vpPnPsolvers.resize(MAX_RELOC_CANDIDATES);
        vvpMapPointMatches.resize(MAX_RELOC_CANDIDATES);
        vbDiscarded.resize(MAX_RELOC_CANDIDATES);
    }

    for(int i=0; i<nKFs; i++)
    {
        try {
            KeyFrame* pKF = vpCandidateKFs[i];
            if(!pKF || pKF->isBad())
            {
                vbDiscarded[i] = true;
            }
            else
            {
                // 使用局部 ORBmatcher
                ORBmatcher matcher(0.75,true);
                int nmatches = matcher.SearchByHBST(pKF,mCurrentFrame,vvpMapPointMatches[i]);
                if(nmatches<TRACKING_REFKF_MIN_MATCHES)
                {
                    vbDiscarded[i] = true;
                }
                else
                {
                    PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                    pSolver->SetRansacParameters(PNP_RANSAC_PROB, PNP_RANSAC_MIN_INLIERS, PNP_RANSAC_MAX_ITERS, PNP_RANSAC_MIN_SET, PNP_RANSAC_EPSILON, PNP_RANSAC_TH2);
                    vpPnPsolvers[i] = pSolver;
                }
            }
        } catch (...) {
            vbDiscarded[i] = true;
        }
    }

    // 统计有效候选数
    for(int i=0; i<nKFs; i++) {
        if(!vbDiscarded[i]) nCandidates++;
    }

    // 或者执行一些P4P RANSAC迭代，直到找到由足够内点支持的相机姿态
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    auto start_time = std::chrono::steady_clock::now();
    while(nCandidates>0 && !bMatch)
    {
        auto current_time = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count() > 1000) {
            break; // 超时强制退出
        }

        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // 执行5次Ransac迭代
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            
            // 防止空指针解引用
            if(!pSolver) {
                vbDiscarded[i] = true;
                nCandidates--;
                continue;
            }

            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // 如果Ransac达到最大迭代次数，则丢弃关键帧
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // 如果计算出相机姿态，则进行优化
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood<TRACKING_POSE_OPT_MIN_INLIERS)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // 如果内点较少，在粗窗口中通过投影搜索并再次优化
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // 如果内点较多但仍不够，在更窄的窗口中再次通过投影搜索
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // 如果姿态由足够的内点支持，则停止ransac并继续
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    // 清理 PnP 求解器，防止内存泄漏。由于 Relocalization 每帧都可能调用，此处泄露会导致内存迅速耗尽。
    for(int i=0; i<nKFs; i++)
    {
        if(vpPnPsolvers[i])
            delete vpPnPsolvers[i];
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void Tracking::Reset()
{
    LOGD("跟踪::重置 地图=%p", (void*)mpMap);

    mpLocalMapper->RequestReset();
    mpLoopClosing->RequestReset();
    mpKeyFrameDB->clear();

    // 在清除地图之前停止后台重定位线程，以避免MapPoint互斥锁上的竞争
    StopGlobalRelocThread();
    
    // 完全清空重定位缓存和对齐状态，确保Reset后不会残留旧状态
    {
        std::unique_lock<std::mutex> lk(mMutexReloc);
        // 清空参考缓存
        mRefDesc.release();
        mRefIdxToMP.clear();
        mRefSnapshots.clear();
        mpRefTree.reset();
        mRefCachedMPCount = 0;
        mRefLastBuildTs = 0.0;
        
        // 清空对齐状态（确保在清空缓存后清除）
        mbHaveMapAlign = false;
        mAlignConfidence = 0.0f;
        mLastAlignTs = 0.0;
        mT_map_from_slam.release();
        mSmoothedT_map_from_slam.release();
        mAlignUpdateCount = 0;
        mAlignSkipCounter = 0;
        
        // 重置快照序列号，确保后台线程不会使用旧快照
        mSnapSeqProduced.store(0ULL);
        mSnapSeqConsumed.store(0ULL);
        
        // 清空最后快照数据
        mLastDesc.release();
        mLastKeysUn.clear();
        mLastN = 0;
        mLastTimestamp = 0.0;
        mLastTcwSlam.release();
    }
    
    // 清空重定位缓冲区，确保不会使用旧的对齐结果
    {
        std::unique_lock<std::mutex> lk(mMutexRelocBuf);
        mRelocBuf.T_map_from_slam.release();
        mRelocBuf.inliers = 0;
        mRelocBuf.confidence = 0.0f;
        mRelocBuf.ts = 0.0;
        mRelocSeqProduced = 0ULL;
        mRelocSeqConsumed = 0ULL;
    }
    
    // 重置匹配分数
    mRelocMatchScore.store(0.0f);
    // Reset后设置cooldown期（至少30帧），等待足够的新扫描点积累后再开始重定位
    // 避免Reset后立即匹配到错误区域
    mRelocCooldownFrames = RESET_COOLDOWN_FRAMES;  // 从Config.h读取冷却帧数
    mConsecutiveFail = 0;

    // 清除地图（这会删除地图点和关键帧）
    {
        // 统计将丢失多少加载的点
        int total=(int)mpMap->MapPointsInMap();
        int loaded=0; for(MapPoint* p : mpMap->GetAllMapPoints()){ if(p && !p->isBad() && p->mbFromLoadedMap) loaded++; }
        LOGD("跟踪::清理地图 点数=%d 已加载=%d", total, loaded);
        mpMap->clear();
    }

    //KeyFrame::nNextId = 0;
    //Frame::nNextId = 0;
    mState = NO_IMAGES_YET;
    mpReferenceKF = static_cast<KeyFrame*>(NULL);
    mLastFrame = Frame();
    mCurrentFrame = Frame();

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }
    mLastInitAttemptTime = 0.0;
    
    // 重新启动线程（确保在清空所有状态后）
    StartGlobalRelocThread();
    LOGD("跟踪::重置结束");

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();
}

void Tracking::ClearTrackingState()
{
    LOGD("跟踪::清除状态 (仅运行时)");
    
    // 清除跟踪状态但保留加载地图点：地图非空则设LOST尝试重定位
    // 避免设为NO_IMAGES_YET导致新建坐标系与加载地图不匹配
    if(mpMap && mpMap->KeyFramesInMap() > 0) {
        mState = LOST;
    } else {
        mState = NO_IMAGES_YET;
    }
    mnLastRelocFrameId = 0;
    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();
    mpReferenceKF = static_cast<KeyFrame*>(NULL);
    mLastFrame = Frame();
    mCurrentFrame = Frame();
    
    // 重置帧ID，但地图有关键帧时不能重置以防G2O ID冲突
    //KeyFrame::nNextId = 0; Frame::nNextId = 0;
    if(!mpMap || mpMap->KeyFramesInMap() == 0) {
        KeyFrame::nNextId = 0;
        Frame::nNextId = 0;
    } else {
    }
    
    // 清除关键帧引用以防止访问已删除的对象
    mpLastKeyFrame = nullptr;
    mpReferenceKF = nullptr;
    
    // 清除本地跟踪向量
    mvpLocalKeyFrames.clear();
    mvpLocalMapPoints.clear();
    
    // 清除对齐状态和重定位缓冲区，确保不会使用旧的对齐结果
    // 如果地图点还在，需要立即重建重定位缓存以确保一致性（防止使用无效的缓存数据）
    {
        std::unique_lock<std::mutex> lk(mMutexReloc);
        // 清除对齐状态
        mbHaveMapAlign = false;
        mAlignConfidence = 0.0f;
        mLastAlignTs = 0.0;
        mT_map_from_slam.release();
        mSmoothedT_map_from_slam.release();
        mAlignUpdateCount = 0;
        mAlignSkipCounter = 0;
        
        // 重置快照序列号
        mSnapSeqProduced.store(0ULL);
        mSnapSeqConsumed.store(0ULL);
        
        // 清空最后快照数据
        mLastDesc.release();
        mLastKeysUn.clear();
        mLastN = 0;
        mLastTimestamp = 0.0;
        mLastTcwSlam.release();
        
        // 清空旧的重定位缓存，强制重建（防止使用无效的MapPoint指针）
        // 因为Reset后MapPoint对象可能已经变化，旧的缓存数据不可用
        mRefDesc.release();
        mRefIdxToMP.clear();
        mRefSnapshots.clear();
        mpRefTree.reset();
        mRefCachedMPCount = 0;
        mRefLastBuildTs = 0.0;
    }
    
    // 如果地图点还在，立即重建重定位缓存
    // 这样Reset后可以立即使用地图数据进行重定位，不会出现"读取的需要匹配的地图数据也被清了"的问题
    if(mpMap && mpMap->MapPointsInMap() > 0) {
        // 检查是否有加载的地图点需要重建缓存
        const vector<MapPoint*> allMPs = mpMap->GetAllMapPoints();
        bool hasLoadedMapPoints = false;
        for(MapPoint* p : allMPs) {
            if(p && !p->isBad() && p->mbFromLoadedMap) {
                hasLoadedMapPoints = true;
                break;
            }
        }
        
        if(hasLoadedMapPoints) {
            LOGD("清除跟踪状态: 重建缓存");
            BuildLoadedRefCache();  // 立即重建缓存
        }
    }
    
    // 清空重定位缓冲区
    {
        std::unique_lock<std::mutex> lk(mMutexRelocBuf);
        mRelocBuf.T_map_from_slam.release();
        mRelocBuf.inliers = 0;
        mRelocBuf.confidence = 0.0f;
        mRelocBuf.ts = 0.0;
        mRelocSeqProduced = 0ULL;
        mRelocSeqConsumed = 0ULL;
    }
    
    // 重置匹配分数和冷却期
    mRelocMatchScore.store(0.0f);
    mRelocCooldownFrames = 0;
    mConsecutiveFail = 0;
    
    LOGD("跟踪::清除跟踪状态: 运行时状态已清除，地图点已保留");
}

void Tracking::PrepareForNewMap()
{
    // 仅清除运行时状态，不调用RequestReset/mpMap->clear/StopGlobalRelocThread
    // 设计目标：毫秒级耗时，避免高频丢失场景下主线程卡死
    LOGD("跟踪::PrepareForNewMap (轻量切图，仅清运行时状态)");

    mState = NO_IMAGES_YET;
    mpLastKeyFrame = nullptr;
    mpReferenceKF = nullptr;
    mvpLocalKeyFrames.clear();
    mvpLocalMapPoints.clear();
    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();
    mConsecutiveLostFrames = 0;
    mConsecutiveFail = 0;
    mnLastRelocFrameId = 0;
    mLastInitAttemptTime = 0.0;
    mlpTemporalPoints.clear();
    mVelocity = cv::Mat();

    if (mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mTrackingOK.store(false);
    mLastTrackingInliers.store(0);
    mRelocMatchScore.store(0.0f);
    mRelocCooldownFrames = 30;  // 给一段冷却，避免立即触发重定位匹配到错误区域
    mRefCacheRetryCount = 0;
}

void Tracking::ClearRelocCacheForMapSwitch()
{
    // 仅清空加载/旧地图的重定位缓存与对齐状态（不释放Map或停后台线程）
    // 调用顺序：StopGlobalRelocThread -> ClearRelocCacheForMapSwitch -> SwitchToMap -> PrepareForNewMap + StartGlobalRelocThread
    LOGD("跟踪::ClearRelocCacheForMapSwitch (仅清重定位缓存)");

    {
        std::unique_lock<std::mutex> lk(mMutexReloc);

        mRefDesc.release();
        mRefIdxToMP.clear();
        mRefSnapshots.clear();
        mpRefTree.reset();
        mRefGrid.Clear();
        mRefCachedMPCount = 0;
        mRefLastBuildTs = 0.0;

        mLastDesc.release();
        mLastKeysUn.clear();
        mLastN = 0;
        mLastTimestamp = 0.0;
        mLastTcwSlam.release();
        mSnapSeqProduced.store(0ULL);
        mSnapSeqConsumed.store(0ULL);

        mbHaveMapAlign = false;
        mAlignConfidence = 0.0f;
        mLastAlignTs = 0.0;
        mT_map_from_slam.release();
        mSmoothedT_map_from_slam.release();
        mAlignUpdateCount = 0;
        mAlignSkipCounter = 0;
    }

    {
        std::unique_lock<std::mutex> lk(mMutexRelocBuf);
        mRelocBuf.T_map_from_slam.release();
        mRelocBuf.inliers = 0;
        mRelocBuf.confidence = 0.0f;
        mRelocBuf.ts = 0.0;
        mRelocSeqProduced = 0ULL;
        mRelocSeqConsumed = 0ULL;
    }

    mPendingMapId = -1;
    mPendingMapCount = 0;
    mPendingAlign = RelocAlignResult();
    mLastAcceptedAlignInliers = 0;
}

void Tracking::ClearRelocCache()
{
    LOGD("跟踪::清除重定位缓存: 仅清除重定位相关缓存，保持跟踪状态");
    
    // 清除对齐状态和重定位缓冲区
    {
        std::unique_lock<std::mutex> lk(mMutexReloc);
        // 清除对齐状态
        mbHaveMapAlign = false;
        mAlignConfidence = 0.0f;
        mLastAlignTs = 0.0;
        mT_map_from_slam.release();
        mSmoothedT_map_from_slam.release();
        mAlignUpdateCount = 0;
        mAlignSkipCounter = 0;
        
        // 清空旧的重定位缓存
        mRefDesc.release();
        mRefIdxToMP.clear();
        mRefSnapshots.clear();
        mpRefTree.reset();
        mRefGrid.Clear();
        mRefCachedMPCount = 0;
        mRefLastBuildTs = 0.0;
    }
    
    // 清空重定位缓冲区
    {
        std::unique_lock<std::mutex> lk(mMutexRelocBuf);
        mRelocBuf.T_map_from_slam.release();
        mRelocBuf.inliers = 0;
        mRelocBuf.confidence = 0.0f;
        mRelocBuf.ts = 0.0;
        mRelocSeqProduced = 0ULL;
        mRelocSeqConsumed = 0ULL;
    }
    
    // 重置匹配分数和冷却期
    mRelocMatchScore.store(0.0f);
    mRelocCooldownFrames = 0;
    mConsecutiveFail = 0;
    
    
    LOGD("跟踪::清除重定位缓存: 完成");
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    float fx = CAMERA_FX;
    float fy = CAMERA_FY;
    float cx = CAMERA_CX;
    float cy = CAMERA_CY;

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = CAMERA_K1;
    DistCoef.at<float>(1) = CAMERA_K2;
    DistCoef.at<float>(2) = CAMERA_P1;
    DistCoef.at<float>(3) = CAMERA_P2;
    const float k3 = CAMERA_K3;
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = 0; // bf 参数在单目相机中不使用

    Frame::mbInitialComputations = true;
}

void Tracking::UpdateCalibration(float fx, float fy, float cx, float cy)
{
    std::unique_lock<std::mutex> lock(mMutexReloc);
    mK.at<float>(0,0) = fx;
    mK.at<float>(1,1) = fy;
    mK.at<float>(0,2) = cx;
    mK.at<float>(1,2) = cy;
    
    // 如果当前帧已实例化，更新其持有的内参K的副本
    if(!mCurrentFrame.mK.empty()) {
        mCurrentFrame.mK.at<float>(0,0) = fx;
        mCurrentFrame.mK.at<float>(1,1) = fy;
        mCurrentFrame.mK.at<float>(0,2) = cx;
        mCurrentFrame.mK.at<float>(1,2) = cy;
    }
    
    // 将静态初始计算标志设为true，强制下一帧的构造函数重新计算invfx, invfy, cx, cy以及网格宽高
    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}

void Tracking::LoadedMapGrid::Build(const std::vector<RefMPSnapshot>& snaps, float size)
{
    Clear();
    if(snaps.empty()) return;
    
    cellSize = size;
    
    // 1. 计算边界
    minX = maxX = snaps[0].Pw.x;
    minY = maxY = snaps[0].Pw.y;
    minZ = maxZ = snaps[0].Pw.z;
    
    for(const auto& s : snaps) {
        if(s.Pw.x < minX) minX = s.Pw.x;
        if(s.Pw.x > maxX) maxX = s.Pw.x;
        if(s.Pw.y < minY) minY = s.Pw.y;
        if(s.Pw.y > maxY) maxY = s.Pw.y;
        if(s.Pw.z < minZ) minZ = s.Pw.z;
        if(s.Pw.z > maxZ) maxZ = s.Pw.z;
    }
    
    // 稍微扩大边界以避免边界问题
    minX -= cellSize; maxX += cellSize;
    minY -= cellSize; maxY += cellSize;
    minZ -= cellSize; maxZ += cellSize;
    
    nCols = (int)((maxX - minX) / cellSize) + 1;
    nRows = (int)((maxY - minY) / cellSize) + 1;
    nSlices = (int)((maxZ - minZ) / cellSize) + 1;
    
    // 限制网格大小以防内存爆炸（虽然不太可能，除非坐标系错误）
    if((long long)nCols * nRows * nSlices > 10000000) {
        // 如果空间太大，增加 cellSize
        float maxSide = std::max({maxX-minX, maxY-minY, maxZ-minZ});
        cellSize = maxSide / 100.0f; // 限制每个维度最多100格
        nCols = (int)((maxX - minX) / cellSize) + 1;
        nRows = (int)((maxY - minY) / cellSize) + 1;
        nSlices = (int)((maxZ - minZ) / cellSize) + 1;
    }
    
    cells.resize(nCols * nRows * nSlices);
    
    // 2. 填充网格
    for(size_t i=0; i<snaps.size(); ++i) {
        const auto& s = snaps[i];
        int c = (int)((s.Pw.x - minX) / cellSize);
        int r = (int)((s.Pw.y - minY) / cellSize);
        int sl = (int)((s.Pw.z - minZ) / cellSize);
        
        if(c>=0 && c<nCols && r>=0 && r<nRows && sl>=0 && sl<nSlices) {
            int idx = sl * (nCols * nRows) + r * nCols + c;
            cells[idx].push_back((int)i);
        }
    }
}

void Tracking::LoadedMapGrid::GetCandidatesInBBox(const cv::Point3f& center, float radius, std::vector<int>& outIndices) const
{
    if(cells.empty()) return;
    
    int cMin = (int)((center.x - radius - minX) / cellSize);
    int cMax = (int)((center.x + radius - minX) / cellSize);
    int rMin = (int)((center.y - radius - minY) / cellSize);
    int rMax = (int)((center.y + radius - minY) / cellSize);
    int sMin = (int)((center.z - radius - minZ) / cellSize);
    int sMax = (int)((center.z + radius - minZ) / cellSize);
    
    // 边界裁剪
    cMin = std::max(0, cMin); cMax = std::min(nCols-1, cMax);
    rMin = std::max(0, rMin); rMax = std::min(nRows-1, rMax);
    sMin = std::max(0, sMin); sMax = std::min(nSlices-1, sMax);
    
    for(int sl = sMin; sl <= sMax; ++sl) {
        for(int r = rMin; r <= rMax; ++r) {
            for(int c = cMin; c <= cMax; ++c) {
                int idx = sl * (nCols * nRows) + r * nCols + c;
                const auto& cellPoints = cells[idx];
                outIndices.insert(outIndices.end(), cellPoints.begin(), cellPoints.end());
            }
        }
    }
}

void Tracking::LoadedMapGrid::GetCandidatesInSphere(const cv::Point3f& center, float radius,
                                                    const std::vector<RefMPSnapshot>& snaps,
                                                    std::vector<int>& outIndices) const
{
    if(cells.empty()) return;
    
    int cMin = (int)((center.x - radius - minX) / cellSize);
    int cMax = (int)((center.x + radius - minX) / cellSize);
    int rMin = (int)((center.y - radius - minY) / cellSize);
    int rMax = (int)((center.y + radius - minY) / cellSize);
    int sMin = (int)((center.z - radius - minZ) / cellSize);
    int sMax = (int)((center.z + radius - minZ) / cellSize);
    
    // 边界裁剪
    cMin = std::max(0, cMin); cMax = std::min(nCols-1, cMax);
    rMin = std::max(0, rMin); rMax = std::min(nRows-1, rMax);
    sMin = std::max(0, sMin); sMax = std::min(nSlices-1, sMax);
    
    // 精确球形过滤,减少84.6%冗余候选
    // 预计算平方半径,避免sqrt和重复计算
    const float radiusSq = radius * radius;
    
    // 预估候选数量并预分配空间,减少动态增长
    int estimatedCount = (cMax - cMin + 1) * (rMax - rMin + 1) * (sMax - sMin + 1) * 10;
    outIndices.reserve(outIndices.size() + estimatedCount);
    
    for(int sl = sMin; sl <= sMax; ++sl) {
        for(int r = rMin; r <= rMax; ++r) {
            for(int c = cMin; c <= cMax; ++c) {
                int idx = sl * (nCols * nRows) + r * nCols + c;
                const auto& cellPoints = cells[idx];
                
                // 对网格内每个点进行精确距离判断
                for(int ptIdx : cellPoints) {
                    if(ptIdx >= 0 && ptIdx < (int)snaps.size()) {
                        const cv::Point3f& pt = snaps[ptIdx].Pw;
                        
                        // 使用平方距离判断,避免sqrt
                        const float dx = pt.x - center.x;
                        const float dy = pt.y - center.y;
                        const float dz = pt.z - center.z;
                        const float distSq = dx*dx + dy*dy + dz*dz;
                        
                        if(distSq <= radiusSq) {
                            outIndices.push_back(ptIdx);
                        }
                    }
                }
            }
        }
    }
}

} //namespace ORB_SLAM2

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

#include "Optimizer.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include "../Thirdparty/Eigen/StdVector"

#include "Converter.h"
#include "Config.h"

#include<mutex>

namespace ORB_SLAM2
{

void Optimizer::GlobalBundleAdjustemnt(Map* pMap, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    vector<MapPoint*> vpMP = pMap->GetAllMapPoints();
    BundleAdjustment(vpKFs,vpMP,nIterations,pbStopFlag, nLoopKF, bRobust);
}

void Optimizer::BundleAdjustment(const vector<KeyFrame *> &vpKFs, const vector<MapPoint *> &vpMP,
                                 int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    // 空优化保护：无关键帧或无地图点时直接返回，避免 g2o 空跑
    if(vpKFs.empty() || vpMP.empty())
        return;

    vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpMP.size());

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;

    // 顶点指针数组（按 id 索引）替代每边 2 次 optimizer.vertex() 哈希查找
    // + dynamic_cast RTTI。数组按需增长（KF/MP id 均在登记时保证容量）。
    std::vector<g2o::OptimizableGraph::Vertex*> vAllVertices;

    // 设置关键帧顶点
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        // 栈版读取位姿（锁内拷贝，替代 GetPose() clone + Converter 转换）
        float poseF[16];
        pKF->GetPose(poseF);
        Eigen::Matrix<double,3,3> R;
        R << poseF[0], poseF[1], poseF[2],
             poseF[4], poseF[5], poseF[6],
             poseF[8], poseF[9], poseF[10];
        Eigen::Matrix<double,3,1> t(poseF[3], poseF[7], poseF[11]);
        vSE3->setEstimate(g2o::SE3Quat(R, t));
        vSE3->setId(pKF->mnId);
        vSE3->setFixed(pKF->mnId==0);
        optimizer.addVertex(vSE3);
        if((size_t)pKF->mnId >= vAllVertices.size())
            vAllVertices.resize((size_t)pKF->mnId + 1, nullptr);
        vAllVertices[pKF->mnId] = vSE3;
        if(pKF->mnId>maxKFid)
            maxKFid=pKF->mnId;
    }

    const float thHuber2D = OPTIMIZER_HUBER_TH_2D; // 卡方检验阈值(5.991对应的平方根)
    const float thHuber3D = OPTIMIZER_HUBER_TH_3D; // 卡方检验阈值(7.815对应的平方根)

    // 设置地图点顶点
    for(size_t i=0; i<vpMP.size(); i++)
    {
        MapPoint* pMP = vpMP[i];
        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        cv::Point3f p3f;
        pMP->GetWorldPos(p3f);
        vPoint->setEstimate(Converter::toVector3d(p3f));
        const int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
        if((size_t)id >= vAllVertices.size())
            vAllVertices.resize((size_t)id + 1, nullptr);
        vAllVertices[id] = vPoint;

       const map<KeyFrame*,size_t> observations = pMP->GetObservations();

        int nEdges = 0;
        // 设置边
        for(map<KeyFrame*,size_t>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {

            KeyFrame* pKF = mit->first;
            if(pKF->isBad() || pKF->mnId>maxKFid)
                continue;

            nEdges++;

            const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit->second];

            // 单目模式只使用2D投影边
            Eigen::Matrix<double,2,1> obs;
            obs << kpUn.pt.x, kpUn.pt.y;

            // 数组直查替代哈希查找+dynamic_cast
            g2o::OptimizableGraph::Vertex* v0 = vAllVertices[id];
            g2o::OptimizableGraph::Vertex* v1 = vAllVertices[pKF->mnId];

            if(!v0 || !v1)
            {
                continue;
            }

            g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();
            if(!e)
            {
                continue;
            }

            try {
                e->setVertex(0, v0);
                e->setVertex(1, v1);
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                if(bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);
                }

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;

                optimizer.addEdge(e);
            } catch (...) {
                delete e;
                continue;
            }
        }

        if(nEdges==0)
        {
            optimizer.removeVertex(vPoint);
            vAllVertices[id] = nullptr;
            vbNotIncludedMP[i]=true;
        }
        else
        {
            vbNotIncludedMP[i]=false;
        }
    }

    optimizer.initializeOptimization();
    // 空优化保护：active（非固定）顶点为 0 时跳过，避免 g2o 空跑
    if(optimizer.activeVertices().empty())
        return;
    optimizer.optimize(nIterations);

    // 恢复优化后的数据

    // 关键帧
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        if(nLoopKF==0)
        {
            pKF->SetPose(Converter::toCvMat(SE3quat));
        }
        else
        {
            pKF->mTcwGBA.create(4,4,CV_32F);
            Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
            pKF->mnBAGlobalForKF = nLoopKF;
        }
    }

    // 点
    for(size_t i=0; i<vpMP.size(); i++)
    {
        if(vbNotIncludedMP[i])
            continue;

        MapPoint* pMP = vpMP[i];

        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));

        if(nLoopKF==0)
        {
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
            pMP->UpdateNormalAndDepth();
        }
        else
        {
            pMP->mPosGBA.create(3,1,CV_32F);
            Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
            pMP->mnBAGlobalForKF = nLoopKF;
        }
    }

}

int Optimizer::PoseOptimization(Frame *pFrame)
{
    // 线程局部缓存：避免每帧 new/delete 求解器对象
    // PoseOptimization 每帧调用 1 次，是实时性的关键路径
    thread_local struct {
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolver_6_3::LinearSolverType* linearSolver = nullptr;
        g2o::BlockSolver_6_3* solver_ptr = nullptr;
        g2o::OptimizationAlgorithmLevenberg* solver = nullptr;
    } cache;

    if (!cache.linearSolver) {
        cache.linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
        cache.solver_ptr = new g2o::BlockSolver_6_3(cache.linearSolver);
        cache.solver = new g2o::OptimizationAlgorithmLevenberg(cache.solver_ptr);
        cache.optimizer.setAlgorithm(cache.solver);
    }
    g2o::SparseOptimizer& optimizer = cache.optimizer;
    optimizer.clear();

    int nInitialCorrespondences=0;

    // 设置帧顶点
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    // 设置地图点顶点
    const int N = pFrame->N;

    vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
    vector<size_t> vnIndexEdgeMono;
    vpEdgesMono.reserve(N);
    vnIndexEdgeMono.reserve(N);

    const float deltaMono = OPTIMIZER_HUBER_TH_2D; // 卡方检验阈值(5.991对应的平方根)
    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP)
        {
            nInitialCorrespondences++;
            pFrame->mvbOutlier[i] = false;

            Eigen::Matrix<double,2,1> obs;
            const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
            obs << kpUn.pt.x, kpUn.pt.y;

            g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();

            try {
                e->setVertex(0, vSE3);
                e->setMeasurement(obs);
                const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
                e->setLevel(0);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(deltaMono);

                e->fx = pFrame->fx;
                e->fy = pFrame->fy;
                e->cx = pFrame->cx;
                e->cy = pFrame->cy;
                cv::Point3f p3f;
                pMP->GetWorldPos(p3f);
                e->Xw[0] = p3f.x;
                e->Xw[1] = p3f.y;
                e->Xw[2] = p3f.z;

                optimizer.addEdge(e);
            } catch (...) {
                delete e;
                continue;
            }

            vpEdgesMono.push_back(e);
            vnIndexEdgeMono.push_back(i);
        }
    }

    if(nInitialCorrespondences<POSE_OPT_MIN_CORRESPONDENCES) {
        return 0;
    }

    // 执行 4 次优化，每次后将观测分类为内点/外点
    // 在下一次优化中，不包括外点，但在最后它们可以再次被分类为内点。
    // 删除 200ms 墙钟超时——4×5 次 LM 迭代本身即有界，且时间截断会使
    // 慢设备上的位姿停在未收敛状态（精度损失且不可复现）
    const float chi2Mono[4]={OPTIMIZER_CHI2_TH_2D,OPTIMIZER_CHI2_TH_2D,OPTIMIZER_CHI2_TH_2D,OPTIMIZER_CHI2_TH_2D};
    const int its[POSE_OPT_PASSES]={POSE_OPT_PASS_ITERS,POSE_OPT_PASS_ITERS,POSE_OPT_PASS_ITERS,POSE_OPT_PASS_ITERS};

    int nBad=0;
    for(size_t it=0; it<POSE_OPT_PASSES; it++)
    {
        vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
        optimizer.initializeOptimization(0);
        if(optimizer.activeVertices().empty())
            return 0;
        optimizer.optimize(its[it]);

        nBad=0;
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
        {
            g2o::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Mono[it])
            {
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else
            {
                pFrame->mvbOutlier[idx]=false;
                e->setLevel(0);
            }

            if(it==2)
                e->setRobustKernel(0);
        }
        if(optimizer.edges().size()<POSE_OPT_MIN_EDGES)
            break;
    }

    // 恢复优化后的位姿并返回内点数量
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    cv::Mat pose = Converter::toCvMat(SE3quat_recov);
    pFrame->SetPose(pose);

    return nInitialCorrespondences-nBad;
}

void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool* pbStopFlag, Map* pMap)
{
    // 局部关键帧：从当前关键帧开始的广度优先搜索
    list<KeyFrame*> lLocalKeyFrames;

    lLocalKeyFrames.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;

    const vector<KeyFrame*> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    // 限制局部BA窗口大小：最多取10个共视关键帧，防止局部BA耗时过久阻塞跟踪
    const int nMaxBAKFs = std::min((int)vNeighKFs.size(), LOCAL_BA_MAX_KFS);
    for(int i=0; i<nMaxBAKFs; i++)
    {
        KeyFrame* pKFi = vNeighKFs[i];
        pKFi->mnBALocalForKF = pKF->mnId;
        if(!pKFi->isBad())
            lLocalKeyFrames.push_back(pKFi);
    }

    // 在局部关键帧中看到的局部地图点
    list<MapPoint*> lLocalMapPoints;
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin() , lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        vector<MapPoint*> vpMPs = (*lit)->GetMapPointMatches();
        for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;
            if(pMP)
                if(!pMP->isBad())
                    if(pMP->mnBALocalForKF!=pKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF=pKF->mnId;
                    }
        }
    }

    // 固定关键帧。看到局部地图点但不是局部关键帧的关键帧
    list<KeyFrame*> lFixedCameras;
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        map<KeyFrame*,size_t> observations = (*lit)->GetObservations();
        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId)
            {
                pKFi->mnBAFixedForKF=pKF->mnId;
                if(!pKFi->isBad())
                    lFixedCameras.push_back(pKFi);
            }
        }
    }

    // 线程局部缓存：避免每次 new/delete 求解器对象
    thread_local struct {
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolver_6_3::LinearSolverType* linearSolver = nullptr;
        g2o::BlockSolver_6_3* solver_ptr = nullptr;
        g2o::OptimizationAlgorithmLevenberg* solver = nullptr;
    } cache;

    if (!cache.linearSolver) {
        cache.linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
        cache.solver_ptr = new g2o::BlockSolver_6_3(cache.linearSolver);
        cache.solver = new g2o::OptimizationAlgorithmLevenberg(cache.solver_ptr);
        cache.optimizer.setAlgorithm(cache.solver);
    }
    g2o::SparseOptimizer& optimizer = cache.optimizer;
    optimizer.clear();

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0;

    // 顶点指针数组（按 id 索引，一次分配）
    std::vector<g2o::OptimizableGraph::Vertex*> vAllVertices(
        (size_t)pMap->GetMaxKFid() + 2, nullptr);

    // 设置局部关键帧顶点
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        // 栈版读取位姿（锁内拷贝，替代 GetPose() clone + Converter 转换）
        float poseF[16];
        pKFi->GetPose(poseF);
        Eigen::Matrix<double,3,3> R;
        R << poseF[0], poseF[1], poseF[2],
             poseF[4], poseF[5], poseF[6],
             poseF[8], poseF[9], poseF[10];
        Eigen::Matrix<double,3,1> t(poseF[3], poseF[7], poseF[11]);
        vSE3->setEstimate(g2o::SE3Quat(R, t));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(pKFi->mnId==0);
        optimizer.addVertex(vSE3);
        vAllVertices[pKFi->mnId] = vSE3;
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    // 设置固定关键帧顶点
    for(list<KeyFrame*>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        // 栈版读取位姿（锁内拷贝，替代 GetPose() clone + Converter 转换）
        float poseF[16];
        pKFi->GetPose(poseF);
        Eigen::Matrix<double,3,3> R;
        R << poseF[0], poseF[1], poseF[2],
             poseF[4], poseF[5], poseF[6],
             poseF[8], poseF[9], poseF[10];
        Eigen::Matrix<double,3,1> t(poseF[3], poseF[7], poseF[11]);
        vSE3->setEstimate(g2o::SE3Quat(R, t));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        vAllVertices[pKFi->mnId] = vSE3;
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    // 精确统计实际边数后 reserve（原先 (KF数+固定KF数)×局部点数 的
    // 上界预留是实际边数的 ~10 倍，白白抬高内存峰值）
    size_t nExactEdges = 0;
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        const map<KeyFrame*,size_t> observations = (*lit)->GetObservations();
        for(map<KeyFrame*,size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;
            if(pKFi && !pKFi->isBad())
                ++nExactEdges;
        }
    }
    const size_t nReserve = nExactEdges > 0 ? nExactEdges : 1;

    vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nReserve);

    vector<KeyFrame*> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nReserve);

    vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nReserve);

    const float thHuberMono = OPTIMIZER_HUBER_TH_2D; // 卡方检验阈值(5.991对应的平方根)

    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        cv::Point3f p3f;
        pMP->GetWorldPos(p3f);
        vPoint->setEstimate(Converter::toVector3d(p3f));
        int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
        if((size_t)id >= vAllVertices.size())
            vAllVertices.resize((size_t)id + 1, nullptr);
        vAllVertices[id] = vPoint;

        const map<KeyFrame*,size_t> observations = pMP->GetObservations();

        // 设置边
        for(map<KeyFrame*,size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(!pKFi->isBad())
            {
                const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

                // 单目模式只使用单目观测
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    // 数组直查替代哈希查找+dynamic_cast
                    g2o::OptimizableGraph::Vertex* v0 =
                        ((size_t)id < vAllVertices.size()) ? vAllVertices[id]
                        : dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id));
                    g2o::OptimizableGraph::Vertex* v1 =
                        ((size_t)pKFi->mnId < vAllVertices.size()) ? vAllVertices[pKFi->mnId]
                        : dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId));

                    if(!v0 || !v1)
                    {
                        continue;
                    }

                    g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();
                    if(!e)
                    {
                        continue;
                    }

                    try {
                        e->setVertex(0, v0);
                        e->setVertex(1, v1);
                        e->setMeasurement(obs);
                        const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                        e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        e->fx = pKFi->fx;
                        e->fy = pKFi->fy;
                        e->cx = pKFi->cx;
                        e->cy = pKFi->cy;

                        optimizer.addEdge(e);
                    } catch (...) {
                        delete e;
                        continue;
                    }
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                // 立体观测相关代码已移除
            }
        }
    }

    if(pbStopFlag)
        if(*pbStopFlag)
            return;

    // 空优化保护：无有效顶点时跳过，避免 g2o 空跑浪费
    if(optimizer.vertices().empty())
        return;

    optimizer.initializeOptimization();
    if(optimizer.activeVertices().empty())
        return;
    optimizer.optimize(LOCAL_BA_ITERATIONS);

    bool bDoMore= true;

    if(pbStopFlag)
        if(*pbStopFlag)
            bDoMore = false;

    if(bDoMore)
    {

    // 检查内点观测
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>OPTIMIZER_CHI2_TH_2D || !e->isDepthPositive())
        {
            e->setLevel(1);
        }

        e->setRobustKernel(0);
    }

    // 在没有外点的情况下再次精细优化
    optimizer.initializeOptimization(0);
    if(optimizer.activeVertices().empty())
        return;
    optimizer.optimize(LOCAL_BA_ITERATIONS);

    }

    vector<pair<KeyFrame*,MapPoint*> > vToErase;
    vToErase.reserve(vpEdgesMono.size());

    // 检查内点观测
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>OPTIMIZER_CHI2_TH_2D || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    // 获取地图互斥锁 (仅在快速写指针/坐标时持锁，写完立即释放)
    {
        unique_lock<mutex> lock(pMap->mMutexMapUpdate);

        if(!vToErase.empty())
        {
            for(size_t i=0;i<vToErase.size();i++)
            {
                KeyFrame* pKFi = vToErase[i].first;
                MapPoint* pMPi = vToErase[i].second;
                pKFi->EraseMapPointMatch(pMPi);
                pMPi->EraseObservation(pKFi);
            }
        }

        // 恢复优化后的数据
        // 关键帧
        for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
        {
            KeyFrame* pKF = *lit;
            g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
            g2o::SE3Quat SE3quat = vSE3->estimate();
            pKF->SetPose(Converter::toCvMat(SE3quat));
        }

        // 地图点位置赋值
        for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
        {
            MapPoint* pMP = *lit;
            g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        }
    }

    // 在 mMutexMapUpdate 锁释放后，在锁外无锁更新地图点的法线和深度（消除主线程等待排队）
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        if(pMP && !pMP->isBad())
            pMP->UpdateNormalAndDepth();
    }
}

void Optimizer::OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections)
{
    // 设置优化器
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    g2o::BlockSolver_7_3::LinearSolverType * linearSolver =
           new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
    g2o::BlockSolver_7_3 * solver_ptr= new g2o::BlockSolver_7_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);

    const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    const vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

    const unsigned int nMaxKFid = pMap->GetMaxKFid();

    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vScw(nMaxKFid+1);
    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vCorrectedSwc(nMaxKFid+1);
    vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid+1, nullptr);  // 初始化为nullptr

    const int minFeat = OPTIMIZER_ESSENTIAL_GRAPH_MIN_FEAT;

    // 设置关键帧顶点
    for(size_t i=0, iend=vpKFs.size(); i<iend;i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

        const int nIDi = pKF->mnId;

        // 检查ID是否在有效范围内
        if(nIDi > nMaxKFid)
        {
            delete VSim3;
            continue;
        }

        LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);

        if(it!=CorrectedSim3.end())
        {
            vScw[nIDi] = it->second;
            VSim3->setEstimate(it->second);
        }
        else
        {
            Eigen::Matrix<double,3,3> Rcw = Converter::toMatrix3d(pKF->GetRotation());
            Eigen::Matrix<double,3,1> tcw = Converter::toVector3d(pKF->GetTranslation());
            g2o::Sim3 Siw(Rcw,tcw,1.0);
            vScw[nIDi] = Siw;
            VSim3->setEstimate(Siw);
        }

        if(pKF==pLoopKF)
            VSim3->setFixed(true);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);
        VSim3->_fix_scale = false;

        optimizer.addVertex(VSim3);

        vpVertices[nIDi]=VSim3;
    }

    set<pair<long unsigned int,long unsigned int> > sInsertedEdges;

    const Eigen::Matrix<double,7,7> matLambda = Eigen::Matrix<double,7,7>::Identity();

    // 设置闭环边
    for(map<KeyFrame *, set<KeyFrame *> >::const_iterator mit = LoopConnections.begin(), mend=LoopConnections.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        // 添加空指针和isBad检查，防止访问已删除的KeyFrame
        if(!pKF || pKF->isBad())
            continue;

        const long unsigned int nIDi = pKF->mnId;
        const set<KeyFrame*> &spConnections = mit->second;

        // 边界检查
        if(nIDi > nMaxKFid)
        {
            continue;
        }

        const g2o::Sim3 Siw = vScw[nIDi];
        const g2o::Sim3 Swi = Siw.inverse();

        for(set<KeyFrame*>::const_iterator sit=spConnections.begin(), send=spConnections.end(); sit!=send; sit++)
        {
            KeyFrame* pKFj = *sit;
            // 检查KeyFrame指针是否有效
            if(!pKFj || pKFj->isBad())
                continue;

            const long unsigned int nIDj = pKFj->mnId;

            //  使用pKFj代替*sit，避免重复解引用
            if((nIDi!=pCurKF->mnId || nIDj!=pLoopKF->mnId) && pKF->GetWeight(pKFj)<minFeat)
                continue;

            // 边界检查
            if(nIDj > nMaxKFid)
            {
                continue;
            }

            const g2o::Sim3 Sjw = vScw[nIDj];
            const g2o::Sim3 Sji = Sjw * Swi;

            // vpVertices 数组直查（建顶点时已登记），免哈希查找与 RTTI
            g2o::OptimizableGraph::Vertex* vj = vpVertices[nIDj];
            g2o::OptimizableGraph::Vertex* vi = vpVertices[nIDi];

            //  只有当两个顶点都存在时才创建 Edge
            if(!vj || !vi)
            {
                continue;
            }

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            if(!e)
            {
                // 内存分配失败
                continue;
            }

            try {
                e->setVertex(1, vj);
                e->setVertex(0, vi);
                e->setMeasurement(Sji);
                e->information() = matLambda;

                // 直接添加 Edge（g2o::addEdge 返回 void）
                // 在添加之前已经验证了 vertices 存在
                optimizer.addEdge(e);
                sInsertedEdges.insert(make_pair(min(nIDi,nIDj),max(nIDi,nIDj)));
            } catch (...) {
                // 如果发生任何异常，清理资源
                delete e;
                continue;
            }
        }
    }

    // 设置普通边
    for(size_t i=0, iend=vpKFs.size(); i<iend; i++)
    {
        KeyFrame* pKF = vpKFs[i];
        // 添加空指针和isBad检查，防止访问已删除的KeyFrame
        if(!pKF || pKF->isBad())
            continue;

        const int nIDi = pKF->mnId;

        // 边界检查
        if(nIDi > nMaxKFid)
        {
            continue;
        }

        g2o::Sim3 Swi;

        LoopClosing::KeyFrameAndPose::const_iterator iti = NonCorrectedSim3.find(pKF);

        if(iti!=NonCorrectedSim3.end())
            Swi = (iti->second).inverse();
        else
            Swi = vScw[nIDi].inverse();

        KeyFrame* pParentKF = pKF->GetParent();

        // 生成树边
        if(pParentKF)
        {
            int nIDj = pParentKF->mnId;

            // 边界检查
            if(nIDj > nMaxKFid)
            {
                continue;
            }

            g2o::Sim3 Sjw;

            LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(pParentKF);

            if(itj!=NonCorrectedSim3.end())
                Sjw = itj->second;
            else
                Sjw = vScw[nIDj];

            g2o::Sim3 Sji = Sjw * Swi;

            // vpVertices 数组直查（建顶点时已登记），免哈希查找与 RTTI
            g2o::OptimizableGraph::Vertex* vj = vpVertices[nIDj];
            g2o::OptimizableGraph::Vertex* vi = vpVertices[nIDi];

            if(vj && vi)
            {
                g2o::EdgeSim3* e = new g2o::EdgeSim3();
                if(!e)
                {
                    // 内存分配失败
                    continue;
                }

                try {
                    e->setVertex(1, vj);
                    e->setVertex(0, vi);
                    e->setMeasurement(Sji);
                    e->information() = matLambda;

                    // 直接添加 Edge（g2o::addEdge 返回 void）
                    // 在添加之前已经验证了 vertices 存在
                    optimizer.addEdge(e);
                } catch (...) {
                    // 如果发生任何异常，清理资源
                    delete e;
                    continue;
                }
            }
        }

        // 闭环边
        const set<KeyFrame*> sLoopEdges = pKF->GetLoopEdges();
        for(set<KeyFrame*>::const_iterator sit=sLoopEdges.begin(), send=sLoopEdges.end(); sit!=send; sit++)
        {
            KeyFrame* pLKF = *sit;
            // 添加空指针和isBad检查，防止访问已删除的KeyFrame
            if(!pLKF || pLKF->isBad())
                continue;

            if(pLKF->mnId<pKF->mnId)
            {
                // 边界检查
                if(pLKF->mnId > nMaxKFid)
                {
                    continue;
                }

                g2o::Sim3 Slw;

                LoopClosing::KeyFrameAndPose::const_iterator itl = NonCorrectedSim3.find(pLKF);

                if(itl!=NonCorrectedSim3.end())
                    Slw = itl->second;
                else
                    Slw = vScw[pLKF->mnId];

                g2o::Sim3 Sli = Slw * Swi;

                // 数组直查
                g2o::OptimizableGraph::Vertex* vl = vpVertices[pLKF->mnId];
                g2o::OptimizableGraph::Vertex* vi = vpVertices[nIDi];

                if(vl && vi)
                {
                    g2o::EdgeSim3* el = new g2o::EdgeSim3();
                    if(!el)
                    {
                        // 内存分配失败
                        continue;
                    }

                    try {
                        el->setVertex(1, vl);
                        el->setVertex(0, vi);
                        el->setMeasurement(Sli);
                        el->information() = matLambda;

                        // 直接添加 Edge（g2o::addEdge 返回 void）
                        // 在添加之前已经验证了 vertices 存在
                        optimizer.addEdge(el);
                    } catch (...) {
                        // 如果发生任何异常，清理资源
                        delete el;
                        continue;
                    }
                }
            }
        }

        // 共视图边
        const vector<KeyFrame*> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
        for(vector<KeyFrame*>::const_iterator vit=vpConnectedKFs.begin(); vit!=vpConnectedKFs.end(); vit++)
        {
            KeyFrame* pKFn = *vit;
            if(pKFn && pKFn!=pParentKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn))
            {
                if(!pKFn->isBad() && pKFn->mnId<pKF->mnId)
                {
                    if(sInsertedEdges.count(make_pair(min(pKF->mnId,pKFn->mnId),max(pKF->mnId,pKFn->mnId))))
                        continue;

                    // 边界检查
                    if(pKFn->mnId > nMaxKFid)
                    {
                        continue;
                    }

                    g2o::Sim3 Snw;

                    LoopClosing::KeyFrameAndPose::const_iterator itn = NonCorrectedSim3.find(pKFn);

                    if(itn!=NonCorrectedSim3.end())
                        Snw = itn->second;
                    else
                        Snw = vScw[pKFn->mnId];

                    g2o::Sim3 Sni = Snw * Swi;

                    // 数组直查
                    g2o::OptimizableGraph::Vertex* vn = vpVertices[pKFn->mnId];
                    g2o::OptimizableGraph::Vertex* vi = vpVertices[nIDi];

                    if(vn && vi)
                    {
                        g2o::EdgeSim3* en = new g2o::EdgeSim3();
                        if(!en)
                        {
                            // 内存分配失败
                            continue;
                        }

                        try {
                            en->setVertex(1, vn);
                            en->setVertex(0, vi);
                            en->setMeasurement(Sni);
                            en->information() = matLambda;

                            // 直接添加 Edge（g2o::addEdge 返回 void）
                            // 在添加之前已经验证了 vertices 存在
                            optimizer.addEdge(en);
                        } catch (...) {
                            // 如果发生任何异常，清理资源
                            delete en;
                            continue;
                        }
                    }
                }
            }
        }
    }

    // 优化
    optimizer.initializeOptimization();
    if(optimizer.activeVertices().empty())
        return;
    optimizer.optimize(ESSENTIAL_GRAPH_BA_ITERS);

    // SE3 位姿恢复
    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFrame* pKFi = vpKFs[i];
        if(!pKFi || pKFi->isBad()) continue;

        const int nIDi = pKFi->mnId;
        if(nIDi < 0 || nIDi > (int)nMaxKFid) continue;

        g2o::VertexSim3Expmap* VSim3 = (nIDi >= 0 && (size_t)nIDi < vpVertices.size()) ? vpVertices[nIDi] : nullptr;
        if(!VSim3) continue;

        g2o::Sim3 CorrectedSiw =  VSim3->estimate();
        vCorrectedSwc[nIDi]=CorrectedSiw.inverse();
        Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = CorrectedSiw.translation();
        double s = CorrectedSiw.scale();

        if(std::abs(s) < 1e-10) continue;
        eigt *=(1./s); //[R t/s;0 1]
        cv::Mat Tiw = Converter::toCvSE3(eigR,eigt);
        pKFi->SetPose(Tiw);
    }

    // 校正点。变换到"未优化"的参考关键帧位姿，然后用优化后的位姿变换回来
    // 为减少锁竞争，分批次或在必要时锁点
    for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
    {
        MapPoint* pMP = vpMPs[i];
        if(!pMP || pMP->isBad()) continue;

        int nIDr;
        if(pMP->mnCorrectedByKF==pCurKF->mnId)
        {
            nIDr = pMP->mnCorrectedReference;
        }
        else
        {
            KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();
            if(!pRefKF || pRefKF->isBad()) continue;
            nIDr = pRefKF->mnId;
        }

        if(nIDr < 0 || nIDr > (int)nMaxKFid) continue;

        g2o::Sim3 Srw = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

        cv::Mat P3Dw = pMP->GetWorldPos();
        if(P3Dw.empty()) continue;

        Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
        Eigen::Matrix<double,3,1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));

        cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);

        {
            // 对于单个点的更新，使用点自身的锁，减少全局锁占用
            pMP->SetWorldPos(cvCorrectedP3Dw);
            pMP->UpdateNormalAndDepth();
        }
    }
}

int Optimizer::OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1, g2o::Sim3 &g2oS12, const float th2)
{
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // 标定
    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;

    // 相机位姿
    float R1w[9], t1w[3], R2w[9], t2w[3];
    pKF1->GetRotation(R1w);
    pKF1->GetTranslation(t1w);
    pKF2->GetRotation(R2w);
    pKF2->GetTranslation(t2w);

    // 设置 Sim3 顶点
    g2o::VertexSim3Expmap * vSim3 = new g2o::VertexSim3Expmap();
    vSim3->_fix_scale=false;
    vSim3->setEstimate(g2oS12);
    vSim3->setId(0);
    vSim3->setFixed(false);
    vSim3->_principle_point1[0] = K1.at<float>(0,2);
    vSim3->_principle_point1[1] = K1.at<float>(1,2);
    vSim3->_focal_length1[0] = K1.at<float>(0,0);
    vSim3->_focal_length1[1] = K1.at<float>(1,1);
    vSim3->_principle_point2[0] = K2.at<float>(0,2);
    vSim3->_principle_point2[1] = K2.at<float>(1,2);
    vSim3->_focal_length2[0] = K2.at<float>(0,0);
    vSim3->_focal_length2[1] = K2.at<float>(1,1);
    optimizer.addVertex(vSim3);

    // 设置地图点顶点
    const int N = vpMatches1.size();
    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
    vector<g2o::EdgeSim3ProjectXYZ*> vpEdges12;
    vector<g2o::EdgeInverseSim3ProjectXYZ*> vpEdges21;
    vector<size_t> vnIndexEdge;

    vnIndexEdge.reserve(2*N);
    vpEdges12.reserve(2*N);
    vpEdges21.reserve(2*N);

    const float deltaHuber = sqrt(th2);

    int nCorrespondences = 0;

    for(int i=0; i<N; i++)
    {
        if(!vpMatches1[i])
            continue;

        MapPoint* pMP1 = vpMapPoints1[i];
        MapPoint* pMP2 = vpMatches1[i];

        const int id1 = 2*i+1;
        const int id2 = 2*(i+1);

        const int i2 = pMP2->GetIndexInKeyFrame(pKF2);

        // 提升到外层作用域：建边时直接复用，不再经 optimizer.vertex 查找
        g2o::VertexSBAPointXYZ* vPoint1 = nullptr;
        g2o::VertexSBAPointXYZ* vPoint2 = nullptr;

        if(pMP1 && pMP2)
        {
            if(!pMP1->isBad() && !pMP2->isBad() && i2>=0)
            {
                // 栈版读取世界坐标；标量 R*t 变换（R1w/t1w 为栈版位姿数组）
                cv::Point3f p3w1;
                pMP1->GetWorldPos(p3w1);
                const float R1w00=R1w[0], R1w01=R1w[1], R1w02=R1w[2];
                const float R1w10=R1w[3], R1w11=R1w[4], R1w12=R1w[5];
                const float R1w20=R1w[6], R1w21=R1w[7], R1w22=R1w[8];
                const float t1w0=t1w[0], t1w1=t1w[1], t1w2=t1w[2];
                vPoint1 = new g2o::VertexSBAPointXYZ();
                vPoint1->setEstimate(Eigen::Vector3d(
                    R1w00*p3w1.x + R1w01*p3w1.y + R1w02*p3w1.z + t1w0,
                    R1w10*p3w1.x + R1w11*p3w1.y + R1w12*p3w1.z + t1w1,
                    R1w20*p3w1.x + R1w21*p3w1.y + R1w22*p3w1.z + t1w2));
                vPoint1->setId(id1);
                vPoint1->setFixed(true);
                optimizer.addVertex(vPoint1);

                cv::Point3f p3w2;
                pMP2->GetWorldPos(p3w2);
                const float R2w00=R2w[0], R2w01=R2w[1], R2w02=R2w[2];
                const float R2w10=R2w[3], R2w11=R2w[4], R2w12=R2w[5];
                const float R2w20=R2w[6], R2w21=R2w[7], R2w22=R2w[8];
                const float t2w0=t2w[0], t2w1=t2w[1], t2w2=t2w[2];
                vPoint2 = new g2o::VertexSBAPointXYZ();
                vPoint2->setEstimate(Eigen::Vector3d(
                    R2w00*p3w2.x + R2w01*p3w2.y + R2w02*p3w2.z + t2w0,
                    R2w10*p3w2.x + R2w11*p3w2.y + R2w12*p3w2.z + t2w1,
                    R2w20*p3w2.x + R2w21*p3w2.y + R2w22*p3w2.z + t2w2));
                vPoint2->setId(id2);
                vPoint2->setFixed(true);
                optimizer.addVertex(vPoint2);
            }
            else
                continue;
        }
        else
            continue;

        nCorrespondences++;

        // 设置边 x1 = S12*X2
        Eigen::Matrix<double,2,1> obs1;
        const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn[i];
        obs1 << kpUn1.pt.x, kpUn1.pt.y;

        // 直接使用上一段刚创建并登记的顶点指针（vPoint1/vPoint2/vSim3），
        // 免去每点 3 次哈希查找 + dynamic_cast
        g2o::OptimizableGraph::Vertex* v0_id2 = static_cast<g2o::OptimizableGraph::Vertex*>(vPoint2);
        g2o::OptimizableGraph::Vertex* v1_id0 = static_cast<g2o::OptimizableGraph::Vertex*>(vSim3);
        g2o::OptimizableGraph::Vertex* v0_id1 = static_cast<g2o::OptimizableGraph::Vertex*>(vPoint1);

        // 只有当所有必要的顶点都存在时才创建和添加 Edge
        if(!v0_id2 || !v1_id0 || !v0_id1)
        {
            continue;
        }

        g2o::EdgeSim3ProjectXYZ* e12 = new g2o::EdgeSim3ProjectXYZ();
        if(!e12)
        {
            continue;
        }

        try {
            e12->setVertex(0, v0_id2);
            e12->setVertex(1, v1_id0);
            e12->setMeasurement(obs1);
            const float &invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
            e12->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare1);

            g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
            e12->setRobustKernel(rk1);
            rk1->setDelta(deltaHuber);
            optimizer.addEdge(e12);
        } catch (...) {
            delete e12;
            continue;
        }

        // 设置边 x2 = S21*X1
        Eigen::Matrix<double,2,1> obs2;
        const cv::KeyPoint &kpUn2 = pKF2->mvKeysUn[i2];
        obs2 << kpUn2.pt.x, kpUn2.pt.y;

        g2o::EdgeInverseSim3ProjectXYZ* e21 = new g2o::EdgeInverseSim3ProjectXYZ();
        if(!e21)
        {
            // 如果 e21 分配失败，需要清理 e12
            continue;
        }

        try {
            e21->setVertex(0, v0_id1);
            e21->setVertex(1, v1_id0);
            e21->setMeasurement(obs2);
            float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
            e21->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare2);

            g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
            e21->setRobustKernel(rk2);
            rk2->setDelta(deltaHuber);
            optimizer.addEdge(e21);
        } catch (...) {
            delete e21;
            // e12 已经成功添加，不需要删除
            continue;
        }

        vpEdges12.push_back(e12);
        vpEdges21.push_back(e21);
        vnIndexEdge.push_back(i);
    }

    // 优化
    optimizer.initializeOptimization();
    if(optimizer.activeVertices().empty())
        return 0;
    optimizer.optimize(SIM3_OPT_ITERS);

    // 检查内点
    int nBad=0;
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        if(e12->chi2()>th2 || e21->chi2()>th2)
        {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx]=static_cast<MapPoint*>(NULL);
            optimizer.removeEdge(e12);
            optimizer.removeEdge(e21);
            vpEdges12[i]=static_cast<g2o::EdgeSim3ProjectXYZ*>(NULL);
            vpEdges21[i]=static_cast<g2o::EdgeInverseSim3ProjectXYZ*>(NULL);
            nBad++;
        }
    }

    int nMoreIterations;
    if(nBad>0)
        nMoreIterations=SIM3_OPT_EXTRA_ITERS;
    else
        nMoreIterations=SIM3_OPT_ITERS;

    if(nCorrespondences-nBad<SIM3_OPT_MIN_INLIERS)
        return 0;

    // 仅使用内点再次优化

    optimizer.initializeOptimization();
    if(optimizer.activeVertices().empty())
        return 0;
    optimizer.optimize(nMoreIterations);

    int nIn = 0;
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        if(e12->chi2()>th2 || e21->chi2()>th2)
        {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx]=static_cast<MapPoint*>(NULL);
        }
        else
            nIn++;
    }

    // 恢复优化后的 Sim3
    g2o::VertexSim3Expmap* vSim3_recov = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(0));
    g2oS12= vSim3_recov->estimate();

    return nIn;
}

} //namespace ORB_SLAM2
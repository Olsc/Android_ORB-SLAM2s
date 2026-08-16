/**
* This file is part of ORB-SLAM2.
* This file is based on the file orb.cpp from the OpenCV library (see BSD license below).
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

/**
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
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

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

#include "ORBextractor.h"
#include "Config.h"
#include "Common.h"
#include "MenthaProfiler.h" // 性能分析器

using namespace cv;
using namespace std;

namespace ORB_SLAM2
{

// 描述子旋转偏移量查找表 (LUT)
struct DescriptorOffset {
    int dy;  // y方向偏移 (行偏移)
    int dx;  // x方向偏移 (列偏移)
};
// [360个角度][512个采样点] = 184320 个预计算偏移量
static DescriptorOffset descriptorOffsetLUT[360][ORB_BRIEF_NUM_POINTS];
static bool bDescriptorLUTInit = false;

// E-1 修正说明：
// 原实现用单值 cachedStep 缓存 step 缩放后的偏移 LUT——但 8 层金字塔 step 各不相同，
// 逐层切换导致每层 miss、每帧最多 8 次全表重建（360×512=184,320 次乘加 ×8 ≈ 1.47M/帧）。
// 修正为在描述子循环内直接计算 dy*step+dx：每关键点 512 次乘加（1000 点/帧 ≈ 0.5M/帧），
// 数学完全等价、零表重建、零额外内存（原先的 thread_local LUT 另占 1.4MB/线程，一并删除）。

static uint32_t reciprocal_table_q24[1025];
static uint16_t arctan_table_q10[1025];
static bool bArctanLUTInit = false;

static void InitArctanLUT() {
    if (bArctanLUTInit) return;
    reciprocal_table_q24[0] = 0;
    for (int i = 1; i <= 1024; ++i) {
        reciprocal_table_q24[i] = (uint32_t)((1ULL << 24) / i);
    }
    for (int i = 0; i <= 1024; ++i) {
        double r = (double)i / 1024.0;
        double rad = std::atan(r);
        arctan_table_q10[i] = (uint16_t)std::round(rad * (180.0 / CV_PI) * 10.0);
    }
    bArctanLUTInit = true;
}

static inline float FastIC_Angle_LUT(int m_01, int m_10)
{
    if (m_10 == 0 && m_01 == 0) return 0.0f;

    int abs_m10 = std::abs(m_10);
    int abs_m01 = std::abs(m_01);

    int angle_q10 = 0;
    if (abs_m10 >= abs_m01) {
        int idx = (abs_m10 <= 1024) ? (int)(((uint64_t)abs_m01 * reciprocal_table_q24[abs_m10]) >> 14) : ((abs_m01 << 10) / abs_m10);
        if (idx > 1024) idx = 1024;
        angle_q10 = arctan_table_q10[idx];
    } else {
        int idx = (abs_m01 <= 1024) ? (int)(((uint64_t)abs_m10 * reciprocal_table_q24[abs_m01]) >> 14) : ((abs_m10 << 10) / abs_m01);
        if (idx > 1024) idx = 1024;
        angle_q10 = 900 - arctan_table_q10[idx];
    }

    if (m_10 < 0) angle_q10 = 1800 - angle_q10;
    if (m_01 < 0) angle_q10 = 3600 - angle_q10;
    if (angle_q10 < 0) angle_q10 += 3600;
    if (angle_q10 >= 3600) angle_q10 -= 3600;

    return (float)angle_q10 * 0.1f;
}

static float IC_Angle(const Mat& image, Point2f pt,  const vector<int> & u_max)
{
    int m_01 = 0, m_10 = 0;

    // 快速舍入
    const int py = (int)(pt.y + 0.5f);
    const int px = (int)(pt.x + 0.5f);
    const uchar* center = &image.at<uchar>(py, px);

    // v=0 中心线：用后缀和消除 u * f(u) 中的乘法
    // 数学等价：sum_{u=1}^{d} u·f(u) == sum_{u=1}^{d} (sum_{k=u}^{d} f(k))
    // 实现：从右向左扫描，累加 f(k)，每次 m_10 += 累加器
    {
        int suffix = 0;
        for (int u = ORB_HALF_PATCH_SIZE; u >= 1; --u) {
            suffix += (center[u] - center[-u]);
            m_10 += suffix;
        }
    }

    int step = (int)image.step1();

    // m_01 后缀和累加器：sum_{v=1}^{h} v * v_sum(v) = sum_{v=1}^{h} suffix_{k=v}^{h} v_sum(k)
    // 从右向左扫描，累计 v_sum 的后缀和，每次 m_01 += 累加器，用加法完全替代乘法
    int suffix_m01 = 0;

    for (int v = ORB_HALF_PATCH_SIZE; v >= 1; --v)
    {
        int v_sum = 0;
        int d = u_max[v];

        int offset = v * step;
        const uchar* ptr_plus = center + offset;
        const uchar* ptr_minus = center - offset;

        // 中心列 (u=0)
        v_sum += (ptr_plus[0] - ptr_minus[0]);

        // 后缀和消除 m_10 内层乘法：sum_{u=1}^{d} u·f(u) = 后缀和累加
        int suffix = 0;
        for (int u = d; u >= 1; --u)
        {
            int val_plus_pos = ptr_plus[u];
            int val_plus_neg = ptr_plus[-u];
            int val_minus_pos = ptr_minus[u];
            int val_minus_neg = ptr_minus[-u];

            v_sum += (val_plus_pos - val_minus_pos) + (val_plus_neg - val_minus_neg);

            int val_u_sum = (val_plus_pos + val_minus_pos);
            int val_neg_u_sum = (val_plus_neg + val_minus_neg);

            suffix += (val_u_sum - val_neg_u_sum);
            m_10 += suffix;
        }

        // m_01 后缀和：suffix_m01 = sum_{k=v}^{h} v_sum(k)，逐层累加
        suffix_m01 += v_sum;
        m_01 += suffix_m01;
    }

    return FastIC_Angle_LUT(m_01, m_10);
}

const float factorPI = (float)(CV_PI/180.f);
static void computeOrbDescriptor(const KeyPoint& kpt,
                                 const Mat& img, const Point* pattern,
                                 uchar* desc)
{
    int angleIdx = (int)(kpt.angle + 0.5f);
    if(angleIdx < 0) angleIdx += 360;
    if(angleIdx >= 360) angleIdx -= 360;

    // 快速舍入优化
    const int kpy = (int)(kpt.pt.y + 0.5f);
    const int kpx = (int)(kpt.pt.x + 0.5f);
    const uchar* center = &img.at<uchar>(kpy, kpx);
    const int step = (int)img.step;

    // E-1：直接由 (dy,dx) 基表计算当前层 step 的偏移（消除每帧 8 次全表重建的 thrash）
    const DescriptorOffset* off = descriptorOffsetLUT[angleIdx];

    #define GET_VALUE(idx) center[off[idx].dy * step + off[idx].dx]

    for (int i = 0; i < ORB_DESC_COLS; ++i, off += 16)
    {
        int t0, t1, val;
        t0 = GET_VALUE(0); t1 = GET_VALUE(1);
        val = t0 < t1;
        t0 = GET_VALUE(2); t1 = GET_VALUE(3);
        val |= (t0 < t1) << 1;
        t0 = GET_VALUE(4); t1 = GET_VALUE(5);
        val |= (t0 < t1) << 2;
        t0 = GET_VALUE(6); t1 = GET_VALUE(7);
        val |= (t0 < t1) << 3;
        t0 = GET_VALUE(8); t1 = GET_VALUE(9);
        val |= (t0 < t1) << 4;
        t0 = GET_VALUE(10); t1 = GET_VALUE(11);
        val |= (t0 < t1) << 5;
        t0 = GET_VALUE(12); t1 = GET_VALUE(13);
        val |= (t0 < t1) << 6;
        t0 = GET_VALUE(14); t1 = GET_VALUE(15);
        val |= (t0 < t1) << 7;

        desc[i] = (uchar)val;
    }

    #undef GET_VALUE
}

static int bit_pattern_31_[ORB_BRIEF_NUM_PAIRS*4] =
{
    8,-3, 9,5/*mean (0), correlation (0)*/,
    4,2, 7,-12/*mean (1.12461e-05), correlation (0.0437584)*/,
    -11,9, -8,2/*mean (3.37382e-05), correlation (0.0617409)*/,
    7,-12, 12,-13/*mean (5.62303e-05), correlation (0.0636977)*/,
    2,-13, 2,12/*mean (0.000134953), correlation (0.085099)*/,
    1,-7, 1,6/*mean (0.000528565), correlation (0.0857175)*/,
    -2,-10, -2,-4/*mean (0.0188821), correlation (0.0985774)*/,
    -13,-13, -11,-8/*mean (0.0363135), correlation (0.0899616)*/,
    -13,-3, -12,-9/*mean (0.121806), correlation (0.099849)*/,
    10,4, 11,9/*mean (0.122065), correlation (0.093285)*/,
    -13,-8, -8,-9/*mean (0.162787), correlation (0.0942748)*/,
    -11,7, -9,12/*mean (0.21561), correlation (0.0974438)*/,
    7,7, 12,6/*mean (0.160583), correlation (0.130064)*/,
    -4,-5, -3,0/*mean (0.228171), correlation (0.132998)*/,
    -13,2, -12,-3/*mean (0.00997526), correlation (0.145926)*/,
    -9,0, -7,5/*mean (0.198234), correlation (0.143636)*/,
    12,-6, 12,-1/*mean (0.0676226), correlation (0.16689)*/,
    -3,6, -2,12/*mean (0.166847), correlation (0.171682)*/,
    -6,-13, -4,-8/*mean (0.101215), correlation (0.179716)*/,
    11,-13, 12,-8/*mean (0.200641), correlation (0.192279)*/,
    4,7, 5,1/*mean (0.205106), correlation (0.186848)*/,
    5,-3, 10,-3/*mean (0.234908), correlation (0.192319)*/,
    3,-7, 6,12/*mean (0.0709964), correlation (0.210872)*/,
    -8,-7, -6,-2/*mean (0.0939834), correlation (0.212589)*/,
    -2,11, -1,-10/*mean (0.127778), correlation (0.20866)*/,
    -13,12, -8,10/*mean (0.14783), correlation (0.206356)*/,
    -7,3, -5,-3/*mean (0.182141), correlation (0.198942)*/,
    -4,2, -3,7/*mean (0.188237), correlation (0.21384)*/,
    -10,-12, -6,11/*mean (0.14865), correlation (0.23571)*/,
    5,-12, 6,-7/*mean (0.222312), correlation (0.23324)*/,
    5,-6, 7,-1/*mean (0.229082), correlation (0.23389)*/,
    1,0, 4,-5/*mean (0.241577), correlation (0.215286)*/,
    9,11, 11,-13/*mean (0.00338507), correlation (0.251373)*/,
    4,7, 4,12/*mean (0.131005), correlation (0.257622)*/,
    2,-1, 4,4/*mean (0.152755), correlation (0.255205)*/,
    -4,-12, -2,7/*mean (0.182771), correlation (0.244867)*/,
    -8,-5, -7,-10/*mean (0.186898), correlation (0.23901)*/,
    4,11, 9,12/*mean (0.226226), correlation (0.258255)*/,
    0,-8, 1,-13/*mean (0.0897886), correlation (0.274827)*/,
    -13,-2, -8,2/*mean (0.148774), correlation (0.28065)*/,
    -3,-2, -2,3/*mean (0.153048), correlation (0.283063)*/,
    -6,9, -4,-9/*mean (0.169523), correlation (0.278248)*/,
    8,12, 10,7/*mean (0.225337), correlation (0.282851)*/,
    0,9, 1,3/*mean (0.226687), correlation (0.278734)*/,
    7,-5, 11,-10/*mean (0.00693882), correlation (0.305161)*/,
    -13,-6, -11,0/*mean (0.0227283), correlation (0.300181)*/,
    10,7, 12,1/*mean (0.125517), correlation (0.31089)*/,
    -6,-3, -6,12/*mean (0.131748), correlation (0.312779)*/,
    10,-9, 12,-4/*mean (0.144827), correlation (0.292797)*/,
    -13,8, -8,-12/*mean (0.149202), correlation (0.308918)*/,
    -13,0, -8,-4/*mean (0.160909), correlation (0.310013)*/,
    3,3, 7,8/*mean (0.177755), correlation (0.309394)*/,
    5,7, 10,-7/*mean (0.212337), correlation (0.310315)*/,
    -1,7, 1,-12/*mean (0.214429), correlation (0.311933)*/,
    3,-10, 5,6/*mean (0.235807), correlation (0.313104)*/,
    2,-4, 3,-10/*mean (0.00494827), correlation (0.344948)*/,
    -13,0, -13,5/*mean (0.0549145), correlation (0.344675)*/,
    -13,-7, -12,12/*mean (0.103385), correlation (0.342715)*/,
    -13,3, -11,8/*mean (0.134222), correlation (0.322922)*/,
    -7,12, -4,7/*mean (0.153284), correlation (0.337061)*/,
    6,-10, 12,8/*mean (0.154881), correlation (0.329257)*/,
    -9,-1, -7,-6/*mean (0.200967), correlation (0.33312)*/,
    -2,-5, 0,12/*mean (0.201518), correlation (0.340635)*/,
    -12,5, -7,5/*mean (0.207805), correlation (0.335631)*/,
    3,-10, 8,-13/*mean (0.224438), correlation (0.34504)*/,
    -7,-7, -4,5/*mean (0.239361), correlation (0.338053)*/,
    -3,-2, -1,-7/*mean (0.240744), correlation (0.344322)*/,
    2,9, 5,-11/*mean (0.242949), correlation (0.34145)*/,
    -11,-13, -5,-13/*mean (0.244028), correlation (0.336861)*/,
    -1,6, 0,-1/*mean (0.247571), correlation (0.343684)*/,
    5,-3, 5,2/*mean (0.000697256), correlation (0.357265)*/,
    -4,-13, -4,12/*mean (0.00213675), correlation (0.373827)*/,
    -9,-6, -9,6/*mean (0.0126856), correlation (0.373938)*/,
    -12,-10, -8,-4/*mean (0.0152497), correlation (0.364237)*/,
    10,2, 12,-3/*mean (0.0299933), correlation (0.345292)*/,
    7,12, 12,12/*mean (0.0307242), correlation (0.366299)*/,
    -7,-13, -6,5/*mean (0.0534975), correlation (0.368357)*/,
    -4,9, -3,4/*mean (0.099865), correlation (0.372276)*/,
    7,-1, 12,2/*mean (0.117083), correlation (0.364529)*/,
    -7,6, -5,1/*mean (0.126125), correlation (0.369606)*/,
    -13,11, -12,5/*mean (0.130364), correlation (0.358502)*/,
    -3,7, -2,-6/*mean (0.131691), correlation (0.375531)*/,
    7,-8, 12,-7/*mean (0.160166), correlation (0.379508)*/,
    -13,-7, -11,-12/*mean (0.167848), correlation (0.353343)*/,
    1,-3, 12,12/*mean (0.183378), correlation (0.371916)*/,
    2,-6, 3,0/*mean (0.228711), correlation (0.371761)*/,
    -4,3, -2,-13/*mean (0.247211), correlation (0.364063)*/,
    -1,-13, 1,9/*mean (0.249325), correlation (0.378139)*/,
    7,1, 8,-6/*mean (0.000652272), correlation (0.411682)*/,
    1,-1, 3,12/*mean (0.00248538), correlation (0.392988)*/,
    9,1, 12,6/*mean (0.0206815), correlation (0.386106)*/,
    -1,-9, -1,3/*mean (0.0364485), correlation (0.410752)*/,
    -13,-13, -10,5/*mean (0.0376068), correlation (0.398374)*/,
    7,7, 10,12/*mean (0.0424202), correlation (0.405663)*/,
    12,-5, 12,9/*mean (0.0942645), correlation (0.410422)*/,
    6,3, 7,11/*mean (0.1074), correlation (0.413224)*/,
    5,-13, 6,10/*mean (0.109256), correlation (0.408646)*/,
    2,-12, 2,3/*mean (0.131691), correlation (0.416076)*/,
    3,8, 4,-6/*mean (0.165081), correlation (0.417569)*/,
    2,6, 12,-13/*mean (0.171874), correlation (0.408471)*/,
    9,-12, 10,3/*mean (0.175146), correlation (0.41296)*/,
    -8,4, -7,9/*mean (0.183682), correlation (0.402956)*/,
    -11,12, -4,-6/*mean (0.184672), correlation (0.416125)*/,
    1,12, 2,-8/*mean (0.191487), correlation (0.386696)*/,
    6,-9, 7,-4/*mean (0.192668), correlation (0.394771)*/,
    2,3, 3,-2/*mean (0.200157), correlation (0.408303)*/,
    6,3, 11,0/*mean (0.204588), correlation (0.411762)*/,
    3,-3, 8,-8/*mean (0.205904), correlation (0.416294)*/,
    7,8, 9,3/*mean (0.213237), correlation (0.409306)*/,
    -11,-5, -6,-4/*mean (0.243444), correlation (0.395069)*/,
    -10,11, -5,10/*mean (0.247672), correlation (0.413392)*/,
    -5,-8, -3,12/*mean (0.24774), correlation (0.411416)*/,
    -10,5, -9,0/*mean (0.00213675), correlation (0.454003)*/,
    8,-1, 12,-6/*mean (0.0293635), correlation (0.455368)*/,
    4,-6, 6,-11/*mean (0.0404971), correlation (0.457393)*/,
    -10,12, -8,7/*mean (0.0481107), correlation (0.448364)*/,
    4,-2, 6,7/*mean (0.050641), correlation (0.455019)*/,
    -2,0, -2,12/*mean (0.0525978), correlation (0.44338)*/,
    -5,-8, -5,2/*mean (0.0629667), correlation (0.457096)*/,
    7,-6, 10,12/*mean (0.0653846), correlation (0.445623)*/,
    -9,-13, -8,-8/*mean (0.0858749), correlation (0.449789)*/,
    -5,-13, -5,-2/*mean (0.122402), correlation (0.450201)*/,
    8,-8, 9,-13/*mean (0.125416), correlation (0.453224)*/,
    -9,-11, -9,0/*mean (0.130128), correlation (0.458724)*/,
    1,-8, 1,-2/*mean (0.132467), correlation (0.440133)*/,
    7,-4, 9,1/*mean (0.132692), correlation (0.454)*/,
    -2,1, -1,-4/*mean (0.135695), correlation (0.455739)*/,
    11,-6, 12,-11/*mean (0.142904), correlation (0.446114)*/,
    -12,-9, -6,4/*mean (0.146165), correlation (0.451473)*/,
    3,7, 7,12/*mean (0.147627), correlation (0.456643)*/,
    5,5, 10,8/*mean (0.152901), correlation (0.455036)*/,
    0,-4, 2,8/*mean (0.167083), correlation (0.459315)*/,
    -9,12, -5,-13/*mean (0.173234), correlation (0.454706)*/,
    0,7, 2,12/*mean (0.18312), correlation (0.433855)*/,
    -1,2, 1,7/*mean (0.185504), correlation (0.443838)*/,
    5,11, 7,-9/*mean (0.185706), correlation (0.451123)*/,
    3,5, 6,-8/*mean (0.188968), correlation (0.455808)*/,
    -13,-4, -8,9/*mean (0.191667), correlation (0.459128)*/,
    -5,9, -3,-3/*mean (0.193196), correlation (0.458364)*/,
    -4,-7, -3,-12/*mean (0.196536), correlation (0.455782)*/,
    6,5, 8,0/*mean (0.1972), correlation (0.450481)*/,
    -7,6, -6,12/*mean (0.199438), correlation (0.458156)*/,
    -13,6, -5,-2/*mean (0.211224), correlation (0.449548)*/,
    1,-10, 3,10/*mean (0.211718), correlation (0.440606)*/,
    4,1, 8,-4/*mean (0.213034), correlation (0.443177)*/,
    -2,-2, 2,-13/*mean (0.234334), correlation (0.455304)*/,
    2,-12, 12,12/*mean (0.235684), correlation (0.443436)*/,
    -2,-13, 0,-6/*mean (0.237674), correlation (0.452525)*/,
    4,1, 9,3/*mean (0.23962), correlation (0.444824)*/,
    -6,-10, -3,-5/*mean (0.248459), correlation (0.439621)*/,
    -3,-13, -1,1/*mean (0.249505), correlation (0.456666)*/,
    7,5, 12,-11/*mean (0.00119208), correlation (0.495466)*/,
    4,-2, 5,-7/*mean (0.00372245), correlation (0.484214)*/,
    -13,9, -9,-5/*mean (0.00741116), correlation (0.499854)*/,
    7,1, 8,6/*mean (0.0208952), correlation (0.499773)*/,
    7,-8, 7,6/*mean (0.0220085), correlation (0.501609)*/,
    -7,-4, -7,1/*mean (0.0233806), correlation (0.496568)*/,
    -8,11, -7,-8/*mean (0.0236505), correlation (0.489719)*/,
    -13,6, -12,-8/*mean (0.0268781), correlation (0.503487)*/,
    2,4, 3,9/*mean (0.0323324), correlation (0.501938)*/,
    10,-5, 12,3/*mean (0.0399235), correlation (0.494029)*/,
    -6,-5, -6,7/*mean (0.0420153), correlation (0.486579)*/,
    8,-3, 9,-8/*mean (0.0548021), correlation (0.484237)*/,
    2,-12, 2,8/*mean (0.0616622), correlation (0.496642)*/,
    -11,-2, -10,3/*mean (0.0627755), correlation (0.498563)*/,
    -12,-13, -7,-9/*mean (0.0829622), correlation (0.495491)*/,
    -11,0, -10,-5/*mean (0.0843342), correlation (0.487146)*/,
    5,-3, 11,8/*mean (0.0929937), correlation (0.502315)*/,
    -2,-13, -1,12/*mean (0.113327), correlation (0.48941)*/,
    -1,-8, 0,9/*mean (0.132119), correlation (0.467268)*/,
    -13,-11, -12,-5/*mean (0.136269), correlation (0.498771)*/,
    -10,-2, -10,11/*mean (0.142173), correlation (0.498714)*/,
    -3,9, -2,-13/*mean (0.144141), correlation (0.491973)*/,
    2,-3, 3,2/*mean (0.14892), correlation (0.500782)*/,
    -9,-13, -4,0/*mean (0.150371), correlation (0.498211)*/,
    -4,6, -3,-10/*mean (0.152159), correlation (0.495547)*/,
    -4,12, -2,-7/*mean (0.156152), correlation (0.496925)*/,
    -6,-11, -4,9/*mean (0.15749), correlation (0.499222)*/,
    6,-3, 6,11/*mean (0.159211), correlation (0.503821)*/,
    -13,11, -5,5/*mean (0.162427), correlation (0.501907)*/,
    11,11, 12,6/*mean (0.16652), correlation (0.497632)*/,
    7,-5, 12,-2/*mean (0.169141), correlation (0.484474)*/,
    -1,12, 0,7/*mean (0.169456), correlation (0.495339)*/,
    -4,-8, -3,-2/*mean (0.171457), correlation (0.487251)*/,
    -7,1, -6,7/*mean (0.175), correlation (0.500024)*/,
    -13,-12, -8,-13/*mean (0.175866), correlation (0.497523)*/,
    -7,-2, -6,-8/*mean (0.178273), correlation (0.501854)*/,
    -8,5, -6,-9/*mean (0.181107), correlation (0.494888)*/,
    -5,-1, -4,5/*mean (0.190227), correlation (0.482557)*/,
    -13,7, -8,10/*mean (0.196739), correlation (0.496503)*/,
    1,5, 5,-13/*mean (0.19973), correlation (0.499759)*/,
    1,0, 10,-13/*mean (0.204465), correlation (0.49873)*/,
    9,12, 10,-1/*mean (0.209334), correlation (0.49063)*/,
    5,-8, 10,-9/*mean (0.211134), correlation (0.503011)*/,
    -1,11, 1,-13/*mean (0.212), correlation (0.499414)*/,
    -9,-3, -6,2/*mean (0.212168), correlation (0.480739)*/,
    -1,-10, 1,12/*mean (0.212731), correlation (0.502523)*/,
    -13,1, -8,-10/*mean (0.21327), correlation (0.489786)*/,
    8,-11, 10,-6/*mean (0.214159), correlation (0.488246)*/,
    2,-13, 3,-6/*mean (0.216993), correlation (0.50287)*/,
    7,-13, 12,-9/*mean (0.223639), correlation (0.470502)*/,
    -10,-10, -5,-7/*mean (0.224089), correlation (0.500852)*/,
    -10,-8, -8,-13/*mean (0.228666), correlation (0.502629)*/,
    4,-6, 8,5/*mean (0.22906), correlation (0.498305)*/,
    3,12, 8,-13/*mean (0.233378), correlation (0.503825)*/,
    -4,2, -3,-3/*mean (0.234323), correlation (0.476692)*/,
    5,-13, 10,-12/*mean (0.236392), correlation (0.475462)*/,
    4,-13, 5,-1/*mean (0.236842), correlation (0.504132)*/,
    -9,9, -4,3/*mean (0.236977), correlation (0.497739)*/,
    0,3, 3,-9/*mean (0.24314), correlation (0.499398)*/,
    -12,1, -6,1/*mean (0.243297), correlation (0.489447)*/,
    3,2, 4,-8/*mean (0.00155196), correlation (0.553496)*/,
    -10,-10, -10,9/*mean (0.00239541), correlation (0.54297)*/,
    8,-13, 12,12/*mean (0.0034413), correlation (0.544361)*/,
    -8,-12, -6,-5/*mean (0.003565), correlation (0.551225)*/,
    2,2, 3,7/*mean (0.00835583), correlation (0.55285)*/,
    10,6, 11,-8/*mean (0.00885065), correlation (0.540913)*/,
    6,8, 8,-12/*mean (0.0101552), correlation (0.551085)*/,
    -7,10, -6,5/*mean (0.0102227), correlation (0.533635)*/,
    -3,-9, -3,9/*mean (0.0110211), correlation (0.543121)*/,
    -1,-13, -1,5/*mean (0.0113473), correlation (0.550173)*/,
    -3,-7, -3,4/*mean (0.0140913), correlation (0.554774)*/,
    -8,-2, -8,3/*mean (0.017049), correlation (0.55461)*/,
    4,2, 12,12/*mean (0.01778), correlation (0.546921)*/,
    2,-5, 3,11/*mean (0.0224022), correlation (0.549667)*/,
    6,-9, 11,-13/*mean (0.029161), correlation (0.546295)*/,
    3,-1, 7,12/*mean (0.0303081), correlation (0.548599)*/,
    11,-1, 12,4/*mean (0.0355151), correlation (0.523943)*/,
    -3,0, -3,6/*mean (0.0417904), correlation (0.543395)*/,
    4,-11, 4,12/*mean (0.0487292), correlation (0.542818)*/,
    2,-4, 2,1/*mean (0.0575124), correlation (0.554888)*/,
    -10,-6, -8,1/*mean (0.0594242), correlation (0.544026)*/,
    -13,7, -11,1/*mean (0.0597391), correlation (0.550524)*/,
    -13,12, -11,-13/*mean (0.0608974), correlation (0.55383)*/,
    6,0, 11,-13/*mean (0.065126), correlation (0.552006)*/,
    0,-1, 1,4/*mean (0.074224), correlation (0.546372)*/,
    -13,3, -9,-2/*mean (0.0808592), correlation (0.554875)*/,
    -9,8, -6,-3/*mean (0.0883378), correlation (0.551178)*/,
    -13,-6, -8,-2/*mean (0.0901035), correlation (0.548446)*/,
    5,-9, 8,10/*mean (0.0949843), correlation (0.554694)*/,
    2,7, 3,-9/*mean (0.0994152), correlation (0.550979)*/,
    -1,-6, -1,-1/*mean (0.10045), correlation (0.552714)*/,
    9,5, 11,-2/*mean (0.100686), correlation (0.552594)*/,
    11,-3, 12,-8/*mean (0.101091), correlation (0.532394)*/,
    3,0, 3,5/*mean (0.101147), correlation (0.525576)*/,
    -1,4, 0,10/*mean (0.105263), correlation (0.531498)*/,
    3,-6, 4,5/*mean (0.110785), correlation (0.540491)*/,
    -13,0, -10,5/*mean (0.112798), correlation (0.536582)*/,
    5,8, 12,11/*mean (0.114181), correlation (0.555793)*/,
    8,9, 9,-6/*mean (0.117431), correlation (0.553763)*/,
    7,-4, 8,-12/*mean (0.118522), correlation (0.553452)*/,
    -10,4, -10,9/*mean (0.12094), correlation (0.554785)*/,
    7,3, 12,4/*mean (0.122582), correlation (0.555825)*/,
    9,-7, 10,-2/*mean (0.124978), correlation (0.549846)*/,
    7,0, 12,-2/*mean (0.127002), correlation (0.537452)*/,
    -1,-6, 0,-11/*mean (0.127148), correlation (0.547401)*/
};

// 初始化描述子偏移量查找表 (内存预生成)
void ORBextractor::InitLUT() {
    if(bDescriptorLUTInit) return;

    LOGD("InitLUT: 开始在内存中生成 ORB LUT...");
    const Point* pattern = (const Point*)bit_pattern_31_;

    // 对每个角度 (0-359度)
    for(int angle = 0; angle < 360; angle++) {
        const float rad = angle * (float)CV_PI / 180.0f;
        const float a = cos(rad);
        const float b = sin(rad);

        // 对每个采样点 (512个点)
        for(int i = 0; i < ORB_BRIEF_NUM_POINTS; i++) {
            const float x = (float)pattern[i].x;
            const float y = (float)pattern[i].y;

            // 预计算旋转后的坐标偏移
            // 原始公式: x' = x*a - y*b, y' = x*b + y*a
            descriptorOffsetLUT[angle][i].dy = cvRound(x * b + y * a);
            descriptorOffsetLUT[angle][i].dx = cvRound(x * a - y * b);
        }
    }

    InitArctanLUT();
    bDescriptorLUTInit = true;
    LOGD("InitLUT: ORB LUT 生成完毕。");
}

ORBextractor::ORBextractor(int _nfeatures, float _scaleFactor, int _nlevels,
         int _iniThFAST, int _minThFAST):
    nfeatures(_nfeatures), scaleFactor(_scaleFactor), nlevels(_nlevels),
    iniThFAST(_iniThFAST), minThFAST(_minThFAST)
{
    mvScaleFactor.resize(nlevels);
    mvLevelSigma2.resize(nlevels);
    mvScaleFactor[0]=1.0f;
    mvLevelSigma2[0]=1.0f;
    for(int i=1; i<nlevels; i++)
    {
        mvScaleFactor[i]=mvScaleFactor[i-1]*scaleFactor;
        mvLevelSigma2[i]=mvScaleFactor[i]*mvScaleFactor[i];
    }

    mvInvScaleFactor.resize(nlevels);
    mvInvLevelSigma2.resize(nlevels);
    for(int i=0; i<nlevels; i++)
    {
        mvInvScaleFactor[i]=1.0f/mvScaleFactor[i];
        mvInvLevelSigma2[i]=1.0f/mvLevelSigma2[i];
    }

    mvImagePyramid.resize(nlevels);
    mvBlurredPyramid.resize(nlevels);
    mvBlurredPyramidPadded.resize(nlevels);

    mnFeaturesPerLevel.resize(nlevels);
    float factor = 1.0f / scaleFactor;
    float nDesiredFeaturesPerScale = nfeatures*(1 - factor)/(1 - (float)pow((double)factor, (double)nlevels));

    int sumFeatures = 0;
    for( int level = 0; level < nlevels-1; level++ )
    {
        mnFeaturesPerLevel[level] = cvRound(nDesiredFeaturesPerScale);
        sumFeatures += mnFeaturesPerLevel[level];
        nDesiredFeaturesPerScale *= factor;
    }
    mnFeaturesPerLevel[nlevels-1] = std::max(nfeatures - sumFeatures, 0);

    const int npoints = ORB_BRIEF_NUM_POINTS;
    const Point* pattern0 = (const Point*)bit_pattern_31_;
    std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern));

    // 初始化描述子偏移量查找表 (如果尚未初始化，则计算)
    InitLUT();

    // 用于计算方向
    // 预先计算圆形补丁中每一行的结束位置
    umax.resize(ORB_HALF_PATCH_SIZE + 1);

    int v, v0, vmax = cvFloor(ORB_HALF_PATCH_SIZE * sqrt(2.f) / 2 + 1);
    int vmin = cvCeil(ORB_HALF_PATCH_SIZE * sqrt(2.f) / 2);
    const double hp2 = ORB_HALF_PATCH_SIZE*ORB_HALF_PATCH_SIZE;
    for (v = 0; v <= vmax; ++v)
        umax[v] = cvRound(sqrt(hp2 - v * v));

    // 确保对称性
    for (v = ORB_HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v)
    {
        while (umax[v0] == umax[v0 + 1])
            ++v0;
        umax[v] = v0;
        ++v0;
    }
}

static void computeOrientation(const Mat& image, vector<KeyPoint>& keypoints, const vector<int>& umax)
{
    for (vector<KeyPoint>::iterator keypoint = keypoints.begin(),
         keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint)
    {
        keypoint->angle = IC_Angle(image, keypoint->pt, umax);
    }
}

void ExtractorNode::DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4)
{
    const int halfX = (UR.x - UL.x + 1) >> 1;
    const int halfY = (BR.y - UL.y + 1) >> 1;

    // 定义子节点的边界
    n1.UL = UL;
    n1.UR = cv::Point2i(UL.x+halfX,UL.y);
    n1.BL = cv::Point2i(UL.x,UL.y+halfY);
    n1.BR = cv::Point2i(UL.x+halfX,UL.y+halfY);
    const int nReserve = (vKeys.size() + 3) >> 2;
    n1.vKeys.reserve(nReserve);

    n2.UL = n1.UR;
    n2.UR = UR;
    n2.BL = n1.BR;
    n2.BR = cv::Point2i(UR.x,UL.y+halfY);
    n2.vKeys.reserve(nReserve);

    n3.UL = n1.BL;
    n3.UR = n1.BR;
    n3.BL = BL;
    n3.BR = cv::Point2i(n1.BR.x,BL.y);
    n3.vKeys.reserve(nReserve);

    n4.UL = n3.UR;
    n4.UR = n2.BR;
    n4.BL = n3.BR;
    n4.BR = BR;
    n4.vKeys.reserve(nReserve);

    // 将点关联到子节点
    for(size_t i=0;i<vKeys.size();i++)
    {
        const cv::KeyPoint &kp = vKeys[i];
        if(kp.pt.x<n1.UR.x)
        {
            if(kp.pt.y<n1.BR.y)
                n1.vKeys.push_back(kp);
            else
                n3.vKeys.push_back(kp);
        }
        else if(kp.pt.y<n1.BR.y)
            n2.vKeys.push_back(kp);
        else
            n4.vKeys.push_back(kp);
    }

    if(n1.vKeys.size()==1)
        n1.bNoMore = true;
    if(n2.vKeys.size()==1)
        n2.bNoMore = true;
    if(n3.vKeys.size()==1)
        n3.bNoMore = true;
    if(n4.vKeys.size()==1)
        n4.bNoMore = true;

}

vector<cv::KeyPoint> ORBextractor::DistributeOctTree(const vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                       const int &maxX, const int &minY, const int &maxY, const int &N, const int &level)
{
    // 计算初始节点数量   
    const int nIni = round(static_cast<float>(maxX-minX)/(maxY-minY));

    const float hX = static_cast<float>(maxX-minX)/nIni;

    list<ExtractorNode> lNodes;

    vector<ExtractorNode*> vpIniNodes;
    vpIniNodes.resize(nIni);

    for(int i=0; i<nIni; i++)
    {
        ExtractorNode ni;
        ni.UL = cv::Point2i(hX*static_cast<float>(i),0);
        ni.UR = cv::Point2i(hX*static_cast<float>(i+1),0);
        ni.BL = cv::Point2i(ni.UL.x,maxY-minY);
        ni.BR = cv::Point2i(ni.UR.x,maxY-minY);
        ni.vKeys.reserve(vToDistributeKeys.size());

        lNodes.push_back(ni);
        vpIniNodes[i] = &lNodes.back();
    }

    // 将点关联到子节点 (使用乘法代替除法优化性能)
    const float inv_hX = 1.0f / hX;
    for(size_t i=0;i<vToDistributeKeys.size();i++)
    {
        const cv::KeyPoint &kp = vToDistributeKeys[i];
        int idx = (int)(kp.pt.x * inv_hX);
        if(idx >= nIni) idx = nIni - 1;
        if(idx < 0) idx = 0;
        vpIniNodes[idx]->vKeys.push_back(kp);
    }

    list<ExtractorNode>::iterator lit = lNodes.begin();

    while(lit!=lNodes.end())
    {
        if(lit->vKeys.size()==1)
        {
            lit->bNoMore=true;
            lit++;
        }
        else if(lit->vKeys.empty())
            lit = lNodes.erase(lit);
        else
            lit++;
    }

    bool bFinish = false;

    int iteration = 0;

    vector<pair<int,ExtractorNode*> > vSizeAndPointerToNode;
    vSizeAndPointerToNode.reserve(lNodes.size()*4);

    while(!bFinish)
    {
        iteration++;

        int prevSize = lNodes.size();

        lit = lNodes.begin();

        int nToExpand = 0;

        vSizeAndPointerToNode.clear();

        while(lit!=lNodes.end())
        {
            if(lit->bNoMore)
            {
                // 如果节点只包含一个点，则不分割并继续
                lit++;
                continue;
            }
            else
            {
                // 如果有多于一个点，则分割
                ExtractorNode n1,n2,n3,n4;
                lit->DivideNode(n1,n2,n3,n4);

                // 如果子节点包含点则添加
                if(n1.vKeys.size()>0)
                {
                    lNodes.push_front(std::move(n1));                    
                    if(lNodes.front().vKeys.size()>1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(make_pair(lNodes.front().vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if(n2.vKeys.size()>0)
                {
                    lNodes.push_front(std::move(n2));
                    if(lNodes.front().vKeys.size()>1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(make_pair(lNodes.front().vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if(n3.vKeys.size()>0)
                {
                    lNodes.push_front(std::move(n3));
                    if(lNodes.front().vKeys.size()>1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(make_pair(lNodes.front().vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if(n4.vKeys.size()>0)
                {
                    lNodes.push_front(std::move(n4));
                    if(lNodes.front().vKeys.size()>1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(make_pair(lNodes.front().vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }

                lit=lNodes.erase(lit);
                continue;
            }
        }       

        // 如果节点数超过所需特征数或所有节点都只包含一个点则完成
        if((int)lNodes.size()>=N || (int)lNodes.size()==prevSize)
        {
            bFinish = true;
        }
        else if(((int)lNodes.size()+nToExpand*3)>N)
        {

            while(!bFinish)
            {

                prevSize = lNodes.size();

                vector<pair<int,ExtractorNode*> > vPrevSizeAndPointerToNode = vSizeAndPointerToNode;
                vSizeAndPointerToNode.clear();

                sort(vPrevSizeAndPointerToNode.begin(),vPrevSizeAndPointerToNode.end());
                for(int j=vPrevSizeAndPointerToNode.size()-1;j>=0;j--)
                {
                    ExtractorNode n1,n2,n3,n4;
                    vPrevSizeAndPointerToNode[j].second->DivideNode(n1,n2,n3,n4);

                    // 如果子节点包含点则添加
                    if(n1.vKeys.size()>0)
                    {
                        lNodes.push_front(std::move(n1));
                        if(lNodes.front().vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(make_pair(lNodes.front().vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n2.vKeys.size()>0)
                    {
                        lNodes.push_front(std::move(n2));
                        if(lNodes.front().vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(make_pair(lNodes.front().vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n3.vKeys.size()>0)
                    {
                        lNodes.push_front(std::move(n3));
                        if(lNodes.front().vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(make_pair(lNodes.front().vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n4.vKeys.size()>0)
                    {
                        lNodes.push_front(std::move(n4));
                        if(lNodes.front().vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(make_pair(lNodes.front().vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }

                    lNodes.erase(vPrevSizeAndPointerToNode[j].second->lit);

                    if((int)lNodes.size()>=N)
                        break;
                }

                if((int)lNodes.size()>=N || (int)lNodes.size()==prevSize)
                    bFinish = true;

            }
        }
    }

    // 保留每个节点中的最佳点
    vector<cv::KeyPoint> vResultKeys;
    vResultKeys.reserve(nfeatures);
    for(list<ExtractorNode>::iterator lit=lNodes.begin(); lit!=lNodes.end(); lit++)
    {
        vector<cv::KeyPoint> &vNodeKeys = lit->vKeys;
        cv::KeyPoint* pKP = &vNodeKeys[0];
        float maxResponse = pKP->response;

        for(size_t k=1;k<vNodeKeys.size();k++)
        {
            if(vNodeKeys[k].response>maxResponse)
            {
                pKP = &vNodeKeys[k];
                maxResponse = vNodeKeys[k].response;
            }
        }

        vResultKeys.push_back(*pKP);
    }

    return vResultKeys;
}

void ORBextractor::detectAndOrientLevels(const cv::Range& range,
                                          vector<vector<KeyPoint>>& allKeypoints)
{
    const float W = ORB_FAST_GRID_CELL;
    for (int level = range.start; level < range.end; ++level)
    {
        const int minBorderX = ORB_EDGE_THRESHOLD-ORB_FAST_BORDER_MARGIN;
        const int minBorderY = minBorderX;
        const int maxBorderX = mvImagePyramid[level].cols-ORB_EDGE_THRESHOLD+ORB_FAST_BORDER_MARGIN;
        const int maxBorderY = mvImagePyramid[level].rows-ORB_EDGE_THRESHOLD+3;

        vector<cv::KeyPoint> vToDistributeKeys;
        vToDistributeKeys.reserve(nfeatures*ORB_CANDIDATE_RESERVE_FACTOR);

        const float width = (maxBorderX-minBorderX);
        const float height = (maxBorderY-minBorderY);

        const int nCols = width/W;
        const int nRows = height/W;
        const int wCell = ceil(width/nCols);
        const int hCell = ceil(height/nRows);

        // 1. 全图检测所有可能的候选点 (使用 minThFAST)
        vector<cv::KeyPoint> vAllKeys;
        FAST(mvImagePyramid[level], vAllKeys, minThFAST, true);

        const float inv_wCell = 1.0f / (float)wCell;
        const float inv_hCell = 1.0f / (float)hCell;
        const uint32_t inv_wCell_q16 = (uint32_t)std::round(65536.0f * inv_wCell);
        const uint32_t inv_hCell_q16 = (uint32_t)std::round(65536.0f * inv_hCell);

        // 2. 线程局部平坦 CSR 分箱
        const int nCells = nRows * nCols;
        static thread_local std::vector<int> cellCounts;
        static thread_local std::vector<int> cellOffsets;
        static thread_local std::vector<int> cellHeads;
        static thread_local std::vector<int> keyCellIndices;
        static thread_local std::vector<cv::KeyPoint> sortedKeys;

        if (cellCounts.size() < (size_t)nCells) cellCounts.resize(nCells);
        std::fill(cellCounts.begin(), cellCounts.begin() + nCells, 0);

        if (keyCellIndices.size() < vAllKeys.size()) keyCellIndices.resize(vAllKeys.size());
        if (sortedKeys.size() < vAllKeys.size()) sortedKeys.resize(vAllKeys.size());

        for(size_t i = 0; i < vAllKeys.size(); ++i)
        {
            const cv::KeyPoint& kp = vAllKeys[i];
            const int x_glob = (int)kp.pt.x;
            const int y_glob = (int)kp.pt.y;
            if(x_glob < minBorderX || x_glob >= maxBorderX || y_glob < minBorderY || y_glob >= maxBorderY)
            {
                keyCellIndices[i] = -1;
                continue;
            }

            const int x_rel = x_glob - minBorderX;
            const int y_rel = y_glob - minBorderY;
            int c = (x_rel * inv_wCell_q16) >> 16;
            int r = (y_rel * inv_hCell_q16) >> 16;
            if(c < 0) c = 0; else if(c >= nCols) c = nCols - 1;
            if(r < 0) r = 0; else if(r >= nRows) r = nRows - 1;

            int cellIdx = r * nCols + c;
            keyCellIndices[i] = cellIdx;
            cellCounts[cellIdx]++;
        }

        if (cellOffsets.size() < (size_t)(nCells + 1)) cellOffsets.resize(nCells + 1);
        cellOffsets[0] = 0;
        for (int c = 0; c < nCells; ++c)
            cellOffsets[c + 1] = cellOffsets[c] + cellCounts[c];

        if (cellHeads.size() < (size_t)nCells) cellHeads.resize(nCells);
        std::memcpy(cellHeads.data(), cellOffsets.data(), nCells * sizeof(int));

        for (size_t i = 0; i < vAllKeys.size(); ++i)
        {
            int cellIdx = keyCellIndices[i];
            if (cellIdx >= 0)
            {
                int pos = cellHeads[cellIdx]++;
                sortedKeys[pos] = vAllKeys[i];
            }
        }

        // 3. 逐个网格单次遍历流式过滤 (单次遍历替代原代码双重 Pass，提升缓存命中率)
        for(int cellIdx = 0; cellIdx < nCells; ++cellIdx)
        {
            int start = cellOffsets[cellIdx];
            int end = cellOffsets[cellIdx + 1];
            if(start >= end)
                continue;

            int strongCount = 0;
            size_t beforeLen = vToDistributeKeys.size();

            for(int k = start; k < end; ++k)
            {
                if(sortedKeys[k].response >= iniThFAST)
                {
                    strongCount++;
                    cv::KeyPoint kp = sortedKeys[k];
                    kp.pt.x -= minBorderX;
                    kp.pt.y -= minBorderY;
                    vToDistributeKeys.push_back(kp);
                }
            }

            // 若无强角点，回退收集弱角点
            if(strongCount == 0)
            {
                for(int k = start; k < end; ++k)
                {
                    cv::KeyPoint kp = sortedKeys[k];
                    kp.pt.x -= minBorderX;
                    kp.pt.y -= minBorderY;
                    vToDistributeKeys.push_back(kp);
                }
            }
        }

        vector<KeyPoint> & keypoints = allKeypoints[level];
        keypoints.reserve(nfeatures);

        keypoints = DistributeOctTree(vToDistributeKeys, minBorderX, maxBorderX,
                                      minBorderY, maxBorderY,mnFeaturesPerLevel[level], level);

        const int scaledPatchSize = ORB_PATCH_SIZE*mvScaleFactor[level];

        const int nkps = keypoints.size();
        for(int i=0; i<nkps ; i++)
        {
            keypoints[i].pt.x+=minBorderX;
            keypoints[i].pt.y+=minBorderY;
            keypoints[i].octave=level;
            keypoints[i].size = scaledPatchSize;
        }

        // 合并方向计算到同一屏障
        computeOrientation(mvImagePyramid[level], allKeypoints[level], umax);
    }
}

void ORBextractor::ComputeKeyPointsOctTree(vector<vector<KeyPoint> >& allKeypoints)
{
    allKeypoints.resize(nlevels);
    // 委托给合并辅助函数，由调用者（operator()）决定是否并行
    detectAndOrientLevels(cv::Range(0, nlevels), allKeypoints);
}

void ORBextractor::operator()( InputArray _image, InputArray _mask, vector<KeyPoint>& _keypoints,
                      OutputArray _descriptors)
{ 
    VT_PROFILE_FUNCTION();
    if(_image.empty())
        return;

    Mat image = _image.getMat();
    assert(image.type() == CV_8UC1 );

    // 预计算尺度金字塔 (SLAM依赖它进行后续操作，必须保留)
    ComputePyramid(image);

    vector < vector<KeyPoint> > allKeypoints;
    allKeypoints.resize(nlevels);

    {
        VT_PROFILE_SCOPE("ORB_Detect+Orient");
        detectAndOrientLevels(cv::Range(0, nlevels), allKeypoints);
    }

    int nkeypoints = 0;
    for (int level = 0; level < nlevels; ++level)
        nkeypoints += (int)allKeypoints[level].size();

    if( nkeypoints == 0 )
    {
        _descriptors.release();
        _keypoints.clear();
        return;
    }

    _descriptors.create(nkeypoints, ORB_DESC_COLS, CV_8U);
    Mat descriptors = _descriptors.getMat();

    // 顺序地将结果收集到连续数组，并建立 level → 描述子行号的偏移映射
    _keypoints.resize(nkeypoints);
    vector<int> levelDescOffset(nlevels + 1, 0);
    int offset = 0;
    for (int level = 0; level < nlevels; ++level)
    {
        levelDescOffset[level] = offset;
        vector<KeyPoint>& keypoints = allKeypoints[level];
        int nkpsLevel = keypoints.size();
        if(nkpsLevel > 0)
        {
            std::copy(keypoints.begin(), keypoints.end(), _keypoints.begin() + offset);
            offset += nkpsLevel;
        }
    }
    levelDescOffset[nlevels] = nkeypoints;

    if (mvBlurredPyramid.size() != (size_t)nlevels)
        mvBlurredPyramid.resize(nlevels);
    if (mvBlurredPyramidPadded.size() != (size_t)nlevels)
        mvBlurredPyramidPadded.resize(nlevels);

    {
        VT_PROFILE_SCOPE("ORB_Blur+Desc");
        blurAndComputeDescriptors(cv::Range(0, nlevels), levelDescOffset,
                                  _keypoints, descriptors);
    }
}

// 纯加法与位移位组合，替代常数乘法
// 18*x = (x<<4) + (x<<1)
// 34*x = (x<<5) + (x<<1)
// 49*x = (x<<5) + (x<<4) + x
// 54*x = (x<<5) + (x<<4) + (x<<2) + (x<<1)
inline static int mul18(int x) { return (x << 4) + (x << 1); }
inline static int mul34(int x) { return (x << 5) + (x << 1); }
inline static int mul49(int x) { return (x << 5) + (x << 4) + x; }
inline static int mul54(int x) { return (x << 5) + (x << 4) + (x << 2) + (x << 1); }

// 1D 分离式定点 7x7 高斯卷积 (sigma=2.0, 核权重比 256)
// [18, 34, 49, 54, 49, 34, 18], sum = 256
static void FastIntegerGaussianBlur7x7(const cv::Mat& srcPadded, cv::Mat& dstPadded)
{
    const int rows = srcPadded.rows;
    const int cols = srcPadded.cols;

    // 线程局部平坦缓存区：避免分配
    static thread_local std::vector<int> tempBuf;
    if (tempBuf.size() < (size_t)(rows * cols)) {
        tempBuf.resize(rows * cols);
    }

    // 1. 水平平滑方向 (Horizontal Pass)
    for (int r = 0; r < rows; ++r) {
        const uchar* srcRow = srcPadded.ptr<uchar>(r);
        int* tempRow = &tempBuf[r * cols];
        // 边界内像素 (3 到 cols-4)
        for (int c = 3; c < cols - 3; ++c) {
            int val = mul18(srcRow[c-3]) + mul34(srcRow[c-2]) + mul49(srcRow[c-1]) +
                      mul54(srcRow[c])   +
                      mul49(srcRow[c+1]) + mul34(srcRow[c+2]) + mul18(srcRow[c+3]);
            tempRow[c] = (val + 128) >> 8;
        }
        // 左边界处理 (c < 3)
        for (int c = 0; c < 3; ++c) {
            int val = 0;
            for (int k = -3; k <= 3; ++k) {
                int colIdx = std::abs(c + k);
                if (colIdx >= cols) colIdx = 2 * cols - 1 - colIdx;
                int absK = std::abs(k);
                int coeff = (absK == 0) ? mul54(srcRow[colIdx]) : ((absK == 1) ? mul49(srcRow[colIdx]) : ((absK == 2) ? mul34(srcRow[colIdx]) : mul18(srcRow[colIdx])));
                val += coeff;
            }
            tempRow[c] = (val + 128) >> 8;
        }
        // 右边界处理 (c >= cols - 3)
        for (int c = cols - 3; c < cols; ++c) {
            int val = 0;
            for (int k = -3; k <= 3; ++k) {
                int colIdx = c + k;
                if (colIdx >= cols) colIdx = 2 * (cols - 1) - colIdx;
                if (colIdx < 0) colIdx = -colIdx;
                int absK = std::abs(k);
                int coeff = (absK == 0) ? mul54(srcRow[colIdx]) : ((absK == 1) ? mul49(srcRow[colIdx]) : ((absK == 2) ? mul34(srcRow[colIdx]) : mul18(srcRow[colIdx])));
                val += coeff;
            }
            tempRow[c] = (val + 128) >> 8;
        }
    }

    // 2. 垂直平滑方向 (Vertical Pass)
    for (int r = 3; r < rows - 3; ++r) {
        uchar* dstRow = dstPadded.ptr<uchar>(r);
        const int* tempRowM3 = &tempBuf[(r - 3) * cols];
        const int* tempRowM2 = &tempBuf[(r - 2) * cols];
        const int* tempRowM1 = &tempBuf[(r - 1) * cols];
        const int* tempRow0  = &tempBuf[r * cols];
        const int* tempRowP1 = &tempBuf[(r + 1) * cols];
        const int* tempRowP2 = &tempBuf[(r + 2) * cols];
        const int* tempRowP3 = &tempBuf[(r + 3) * cols];

        for (int c = 0; c < cols; ++c) {
            int val = mul18(tempRowM3[c]) + mul34(tempRowM2[c]) + mul49(tempRowM1[c]) +
                      mul54(tempRow0[c])  +
                      mul49(tempRowP1[c]) + mul34(tempRowP2[c]) + mul18(tempRowP3[c]);
            int pix = (val + 128) >> 8;
            dstRow[c] = (uchar)(pix > 255 ? 255 : (pix < 0 ? 0 : pix));
        }
    }
    // 上下边界处理 (r < 3 || r >= rows - 3)
    for (int r = 0; r < 3; ++r) {
        uchar* dstRow = dstPadded.ptr<uchar>(r);
        for (int c = 0; c < cols; ++c) {
            int val = 0;
            for (int k = -3; k <= 3; ++k) {
                int rowIdx = std::abs(r + k);
                if (rowIdx >= rows) rowIdx = 2 * rows - 1 - rowIdx;
                const int absK = std::abs(k);
                val += (absK == 0) ? mul54(tempBuf[rowIdx * cols + c])
                                   : ((absK == 1) ? mul49(tempBuf[rowIdx * cols + c])
                                                  : ((absK == 2) ? mul34(tempBuf[rowIdx * cols + c])
                                                                 : mul18(tempBuf[rowIdx * cols + c])));
            }
            int pix = (val + 128) >> 8;
            dstRow[c] = (uchar)(pix > 255 ? 255 : (pix < 0 ? 0 : pix));
        }
    }
    for (int r = rows - 3; r < rows; ++r) {
        uchar* dstRow = dstPadded.ptr<uchar>(r);
        for (int c = 0; c < cols; ++c) {
            int val = 0;
            for (int k = -3; k <= 3; ++k) {
                int rowIdx = r + k;
                if (rowIdx >= rows) rowIdx = 2 * (rows - 1) - rowIdx;
                if (rowIdx < 0) rowIdx = -rowIdx;
                const int absK = std::abs(k);
                val += (absK == 0) ? mul54(tempBuf[rowIdx * cols + c])
                                   : ((absK == 1) ? mul49(tempBuf[rowIdx * cols + c])
                                                  : ((absK == 2) ? mul34(tempBuf[rowIdx * cols + c])
                                                                 : mul18(tempBuf[rowIdx * cols + c])));
            }
            int pix = (val + 128) >> 8;
            dstRow[c] = (uchar)(pix > 255 ? 255 : (pix < 0 ? 0 : pix));
        }
    }
}

void ORBextractor::blurAndComputeDescriptors(
    const cv::Range& range, const vector<int>& levelDescOffset,
    vector<KeyPoint>& keypoints, Mat& descriptors)
{
    for (int level = range.start; level < range.end; ++level)
    {
        // E-3：本层无关键点则跳过模糊与 ROI 构造（整图 7×7 定点模糊只为
        // 描述子计算服务，空纹理层每帧的 ~百万次整型运算是纯浪费）
        const int kpStart = levelDescOffset[level];
        const int kpEnd = levelDescOffset[level + 1];
        if (kpStart == kpEnd)
            continue;

        // 确保 mvBlurredPyramidPadded 大小和类型正确
        Size sz = mvImagePyramid[level].size();
        Size wholeSize(sz.width + ORB_EDGE_THRESHOLD*2, sz.height + ORB_EDGE_THRESHOLD*2);
        mvBlurredPyramidPadded[level].create(wholeSize, mvImagePyramid[level].type());

        // 对带边框的图像进行定点 1D 可分离高斯模糊优化
        FastIntegerGaussianBlur7x7(mvImagePyramidPadded[level], mvBlurredPyramidPadded[level]);

        // 让 mvBlurredPyramid[level] 成为 mvBlurredPyramidPadded[level] 的子矩阵 ROI
        mvBlurredPyramid[level] = mvBlurredPyramidPadded[level](
            Rect(ORB_EDGE_THRESHOLD, ORB_EDGE_THRESHOLD, sz.width, sz.height));

        // 本层关键点的描述子计算
        int start = levelDescOffset[level];
        int end = levelDescOffset[level + 1];
        for (int i = start; i < end; ++i)
        {
            KeyPoint& kp = keypoints[i];
            uchar* desc = descriptors.ptr<uchar>(i);
            computeOrbDescriptor(kp, mvBlurredPyramid[level], &pattern[0], desc);

            // 缩放坐标到第0层
            if (level != 0) {
                kp.pt *= mvScaleFactor[level];
            }
        }
    }
}

void ORBextractor::ComputePyramid(cv::Mat image)
{
    VT_PROFILE_FUNCTION();
    // 确保 padded 存储与金字塔层数匹配，首次或层数变化时 resize
    if (mvImagePyramidPadded.size() != (size_t)nlevels)
        mvImagePyramidPadded.resize(nlevels);
    for (int level = 0; level < nlevels; ++level)
    {
        float scale = mvInvScaleFactor[level];
        Size sz(cvRound((float)image.cols*scale), cvRound((float)image.rows*scale));
        Size wholeSize(sz.width + ORB_EDGE_THRESHOLD*2, sz.height + ORB_EDGE_THRESHOLD*2);

        // 复用 padded mat：create() 在 size/type 不变时不会重新分配内存
        mvImagePyramidPadded[level].create(wholeSize, image.type());
        cv::Mat& temp = mvImagePyramidPadded[level];
        mvImagePyramid[level] = temp(Rect(ORB_EDGE_THRESHOLD, ORB_EDGE_THRESHOLD, sz.width, sz.height));

        // 计算调整大小后的图像
        if( level != 0 )
        {
            resize(mvImagePyramid[level-1], mvImagePyramid[level], sz, 0, 0, cv::INTER_LINEAR);

            copyMakeBorder(mvImagePyramid[level], temp, ORB_EDGE_THRESHOLD, ORB_EDGE_THRESHOLD, ORB_EDGE_THRESHOLD, ORB_EDGE_THRESHOLD,
                           BORDER_REFLECT_101+BORDER_ISOLATED);
        }
        else
        {
            copyMakeBorder(image, temp, ORB_EDGE_THRESHOLD, ORB_EDGE_THRESHOLD, ORB_EDGE_THRESHOLD, ORB_EDGE_THRESHOLD,
                           BORDER_REFLECT_101);
        }
    }

}

} //namespace ORB_SLAM2

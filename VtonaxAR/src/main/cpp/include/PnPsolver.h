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

#ifndef PNPSOLVER_H
#define PNPSOLVER_H

#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/core/types_c.h>
#include "MapPoint.h"
#include "Frame.h"
#include "Config.h"
#include <vector>

using namespace std;

namespace ORB_SLAM2
{

class PnPsolver {
 public:
  PnPsolver(const Frame &F, const vector<MapPoint*> &vpMapPointMatches);

  ~PnPsolver();

  void SetRansacParameters(double probability = PNP_RANSAC_PROB, int minInliers = PNP_RANSAC_MIN_INLIERS , int maxIterations = PNP_RANSAC_MAX_ITERS, int minSet = PNP_RANSAC_MIN_SET, float epsilon = PNP_RANSAC_EPSILON,
                           float th2 = PNP_RANSAC_TH2);

  cv::Mat find(vector<bool> &vbInliers, int &nInliers);

  cv::Mat iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers);

 private:

  void CheckInliers();
  bool Refine();

  // 来自原始EPnP代码的函数
  void set_maximum_number_of_correspondences(const int n);
  void reset_correspondences(void);
  void add_correspondence(const double X, const double Y, const double Z,
              const double u, const double v);

  double compute_pose(double R[3][3], double T[3]);

  void relative_error(double & rot_err, double & transl_err,
              const double Rtrue[3][3], const double ttrue[3],
              const double Rest[3][3],  const double test[3]);

  // void print_pose(const double R[3][3], const double t[3]);  // 已注释 - 调试函数
  double reprojection_error(const double R[3][3], const double t[3]);

  void choose_control_points(void);
  void compute_barycentric_coordinates(void);
  void fill_M(CvMat * M, const int row, const double * alphas, const double u, const double v);
  void compute_ccs(const double * betas, const double * ut);
  void compute_pcs(void);

  void solve_for_sign(void);

  void find_betas_approx_1(const CvMat * L_6x10, const CvMat * Rho, double * betas);
  void find_betas_approx_2(const CvMat * L_6x10, const CvMat * Rho, double * betas);
  void find_betas_approx_3(const CvMat * L_6x10, const CvMat * Rho, double * betas);
  void qr_solve(CvMat * A, CvMat * b, CvMat * X);

  double dot(const double * v1, const double * v2);
  double dist2(const double * p1, const double * p2);

  void compute_rho(double * rho);
  void compute_L_6x10(const double * ut, double * l_6x10);

  void gauss_newton(const CvMat * L_6x10, const CvMat * Rho, double current_betas[4]);
  void compute_A_and_b_gauss_newton(const double * l_6x10, const double * rho,
				    double cb[4], CvMat * A, CvMat * b);

  double compute_R_and_t(const double * ut, const double * betas,
			 double R[3][3], double t[3]);

  void estimate_R_and_t(double R[3][3], double t[3]);

  void copy_R_and_t(const double R_dst[3][3], const double t_dst[3],
		    double R_src[3][3], double t_src[3]);

  void mat_to_quat(const double R[3][3], double q[4]);


  double uc, vc, fu, fv;

  double * pws, * us, * alphas, * pcs;
  int maximum_number_of_correspondences;  // 最大对应点数量
  int number_of_correspondences;  // 对应点数量

  double cws[4][3], ccs[4][3];
  double cws_determinant;  // cws行列式

  vector<MapPoint*> mvpMapPointMatches;

  // 2D点
  vector<cv::Point2f> mvP2D;
  vector<float> mvSigma2;

  // 3D点
  vector<cv::Point3f> mvP3Dw;

  // 帧中的索引
  vector<size_t> mvKeyPointIndices;

  // 当前估计
  double mRi[3][3];
  double mti[3];
  cv::Mat mTcwi;
  vector<bool> mvbInliersi;
  int mnInliersi;

  // 当前Ransac状态
  int mnIterations;
  vector<bool> mvbBestInliers;
  int mnBestInliers;
  cv::Mat mBestTcw;

  // 优化后的结果
  cv::Mat mRefinedTcw;
  vector<bool> mvbRefinedInliers;
  int mnRefinedInliers;

  // 对应点数量
  int N;

  // 随机选择的索引 [0 .. N-1]
  vector<size_t> mvAllIndices;

  // RANSAC概率
  double mRansacProb;

  // RANSAC最小内点数
  int mRansacMinInliers;

  // RANSAC最大迭代次数
  int mRansacMaxIts;

  // RANSAC期望的内点/总数比率
  float mRansacEpsilon;

  // RANSAC内点/外点阈值。最大误差 e = dist(P1,T_12*P2)^2
  float mRansacTh;

  // 每次迭代使用的RANSAC最小集合
  int mRansacMinSet;

  // 与尺度级别相关的最大平方误差。最大误差 = th*th*sigma(level)*sigma(level)
  vector<float> mvMaxError;

  // RANSAC 内部循环防止动态内存分配的缓冲区
  std::vector<double> m_M_buffer;
  std::vector<double> m_PW0_buffer;
};

} //namespace ORB_SLAM2

#endif //PNPSOLVER_H

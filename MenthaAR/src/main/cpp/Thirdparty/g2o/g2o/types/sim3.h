// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef G2O_SIM_3
#define G2O_SIM_3

#include "se3_ops.h"
#include "../../../../Thirdparty/Eigen/Geometry"

namespace g2o
{
  using namespace Eigen;

  typedef  Matrix <double, 7, 1> Vector7d;
  typedef  Matrix <double, 7, 7> Matrix7d;
  

  struct Sim3
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  protected:
    Quaterniond r;
    Vector3d t;
    double s;


public:
    Sim3()
    {
      r.setIdentity();
      t.fill(0.);
      s=1.;
    }

    Sim3(const Quaterniond & r, const Vector3d & t, double s)
      : r(r),t(t),s(s)
    {
    }

    Sim3(const Matrix3d & R, const Vector3d & t, double s)
      : r(Quaterniond(R)),t(t),s(s)
    {
    }


    Sim3(const Vector7d & update)
    {

      Vector3d omega;
      for (int i=0; i<3; i++)
        omega[i]=update[i];

      Vector3d upsilon;
      for (int i=0; i<3; i++)
        upsilon[i]=update[i+3];

      double sigma = update[6];
      double theta = omega.norm();
      Matrix3d Omega = skew(omega);
      s = std::exp(sigma);
      Matrix3d Omega2 = Omega*Omega;
      Matrix3d I;
      I.setIdentity();
      Matrix3d R;

      double eps = 0.00001;
      double A,B,C;
      if (fabs(sigma)<eps)
      {
        C = 1;
        if (theta<eps)
        {
          A = 1./2.;
          B = 1./6.;
          R = (I + Omega + Omega*Omega);
        }
        else
        {
          double theta2= theta*theta;
          A = (1-cos(theta))/(theta2);
          B = (theta-sin(theta))/(theta2*theta);
          R = I + sin(theta)/theta *Omega + (1-cos(theta))/(theta*theta)*Omega2;
        }
      }
      else
      {
        C=(s-1)/sigma;
        if (theta<eps)
        {
          double sigma2= sigma*sigma;
          A = ((sigma-1)*s+1)/sigma2;
          B= ((0.5*sigma2-sigma+1)*s)/(sigma2*sigma);
          R = (I + Omega + Omega2);
        }
        else
        {
          R = I + sin(theta)/theta *Omega + (1-cos(theta))/(theta*theta)*Omega2;



          double a=s*sin(theta);
          double b=s*cos(theta);
          double theta2= theta*theta;
          double sigma2= sigma*sigma;

          double c=theta2+sigma2;
          A = (a*sigma+ (1-b)*theta)/(theta*c);
          B = (C-((b-1)*sigma+a*theta)/(c))*1./(theta2);

        }
      }
      r = Quaterniond(R);



      Matrix3d W = A*Omega + B*Omega2 + C*I;
      t = W*upsilon;
    }

     Vector3d map (const Vector3d& xyz) const {
      return s*(r*xyz) + t;
    }

    Vector7d log() const
    {
      Vector7d res;
      double sigma = std::log(s);

      

   
      Vector3d omega;
      Vector3d upsilon;


      Matrix3d R = r.toRotationMatrix();
      double d =  0.5*(R(0,0)+R(1,1)+R(2,2)-1);

      Matrix3d Omega;

      double eps = 0.00001;
      Matrix3d I = Matrix3d::Identity();

      // deltaR(R) = (R32-R23, R13-R31, R21-R12) = 2 sinθ · axis，所以 |deltaR|/2 = sinθ，d = cosθ。
      // θ = atan2(sinθ, cosθ) 在整个 [0, π) 上都良态；原来的 acos(d) 在 θ→0 时条件数发散
      // （docs/bench/algo_bench.py §3：ω 最大相对误差 3.3e-6 → 2.2e-16）。
      const Vector3d dR = deltaR(R);
      const double sinTheta = 0.5*dR.norm();
      const double theta = atan2(sinTheta, d);
      const double theta2 = theta*theta;
      const bool smallTheta = theta < 1e-2;

      // ω = θ/(2 sinθ) · deltaR；小角度用级数 θ/sinθ = 1 + θ²/6 + 7θ⁴/360
      if (smallTheta)
        omega = (0.5*(1.0 + theta2/6.0 + 7.0*theta2*theta2/360.0))*dR;
      else
        omega = (theta/(2.0*sinTheta))*dR;
      Omega = skew(omega);

      double A,B,C;
      if (fabs(sigma)<eps)
      {
        // (e^σ - 1)/σ = 1 + σ/2 + σ²/6 + …（原来直接取 1）
        C = 1.0 + 0.5*sigma + sigma*sigma/6.0;
        if (smallTheta)
        {
          // 原来取 θ→0 的极限 1/2、1/6，相对误差 ≈ θ²/12、θ²/20；级数到 θ⁴ 项后误差 ~θ⁶
          A = 0.5 - theta2/24.0 + theta2*theta2/720.0;
          B = 1.0/6.0 - theta2/120.0 + theta2*theta2/5040.0;
        }
        else
        {
          // 1 - cosθ = 2 sin²(θ/2)，避免直接相减的相消
          const double sh = sin(0.5*theta);
          A = 2.0*sh*sh/theta2;
          B = (theta-sin(theta))/(theta2*theta);
        }
      }
      else
      {
        C = expm1(sigma)/sigma; // 与 (s-1)/σ 恒等，σ 小时无相消
        if (smallTheta)
        {
          double sigma2 = sigma*sigma;
          A = ((sigma-1)*s+1)/(sigma2);
          B = ((0.5*sigma2-sigma+1)*s)/(sigma2*sigma);
        }
        else
        {
          double a=s*sin(theta);
          double b=s*cos(theta);
          double c=theta2 + sigma*sigma;
          A = (a*sigma+ (1-b)*theta)/(theta*c);
          B = (C-((b-1)*sigma+a*theta)/(c))*1./(theta2);
        }
      }

      Matrix3d W = A*Omega + B*Omega*Omega + C*I;

      upsilon = W.lu().solve(t);


      for (int i=0; i<3; i++)
        res[i] = omega[i];

       for (int i=0; i<3; i++)
        res[i+3] = upsilon[i];

      res[6] = sigma;

      return res;
      
    }


    Sim3 inverse() const
    {
      return Sim3(r.conjugate(), r.conjugate()*((-1./s)*t), 1./s);
    }
    

    double operator[](int i) const
    {
      assert(i<8);
      if (i<4){

        return r.coeffs()[i];
      }
      if (i<7){
        return t[i-4];
      }
      return s;
    }

    double& operator[](int i)
    {
      assert(i<8);
      if (i<4){

        return r.coeffs()[i];
      }
      if (i<7)
      {
        return t[i-4];
      }
      return s;
    }

    Sim3 operator *(const Sim3& other) const {
      Sim3 ret;
      ret.r = r*other.r;
      ret.t=s*(r*other.t)+t;
      ret.s=s*other.s;
      return ret;
    }

    Sim3& operator *=(const Sim3& other){
      Sim3 ret=(*this)*other;
      *this=ret;
      return *this;
    }

    inline const Vector3d& translation() const {return t;}

    inline Vector3d& translation() {return t;}

    inline const Quaterniond& rotation() const {return r;}

    inline Quaterniond& rotation() {return r;}

    inline const double& scale() const {return s;}

    inline double& scale() {return s;}

  };

  inline std::ostream& operator <<(std::ostream& out_str,
                                   const Sim3& sim3)
  {
    out_str << sim3.rotation().coeffs() << std::endl;
    out_str << sim3.translation() << std::endl;
    out_str << sim3.scale() << std::endl;

    return out_str;
  }

} // end namespace


#endif

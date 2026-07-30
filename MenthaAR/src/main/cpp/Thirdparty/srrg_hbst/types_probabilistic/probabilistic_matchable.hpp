#pragma once
#include <Eigen/Core>

#include "srrg_hbst/types/binary_matchable.hpp"

namespace srrg_hbst {

  template <typename ObjectType_,
            uint32_t descriptor_size_bits_ = 256,
            typename real_precision_       = double>
  class ProbabilisticMatchable : public BinaryMatchable<ObjectType_, descriptor_size_bits_> {
    // 模板类型转发
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef ObjectType_ ObjectType;
    typedef typename BinaryMatchable<ObjectType_, descriptor_size_bits_>::Descriptor Descriptor;
    typedef Eigen::Matrix<real_precision_, descriptor_size_bits_, 1> BitStatisticsVector;

    // 构造/析构
  public:
    ProbabilisticMatchable(ObjectType object_,
                           const Descriptor& descriptor_,
                           const uint64_t& image_identifier_ = 0) :
      BinaryMatchable<ObjectType_, descriptor_size_bits_>(object_, descriptor_, image_identifier_),
      bit_probabilities(BitStatisticsVector()),
      bit_volatility(BitStatisticsVector()) {
    }

    ProbabilisticMatchable(ObjectType object_,
                           const Descriptor& descriptor_,
                           const BitStatisticsVector& bit_probabilities_,
                           const BitStatisticsVector& bit_volatility_,
                           const uint64_t& image_identifier_ = 0) :
      BinaryMatchable<ObjectType_, descriptor_size_bits_>(object_, descriptor_, image_identifier_),
      bit_probabilities(bit_probabilities_),
      bit_volatility(bit_volatility_) {
    }

    // 包装构造函数：仅在构建系统有 OpenCV 时可用
  #ifdef SRRG_HBST_HAS_OPENCV
    ProbabilisticMatchable(ObjectType object_,
                           const cv::Mat& descriptor_,
                           const uint64_t& image_identifier_ = 0) :
      BinaryMatchable<ObjectType_, descriptor_size_bits_>(object_, descriptor_, image_identifier_),
      bit_probabilities(BitStatisticsVector()),
      bit_volatility(BitStatisticsVector()) {
    }

    ProbabilisticMatchable(ObjectType object_,
                           const cv::Mat& descriptor_,
                           const BitStatisticsVector& bit_probabilities_,
                           const BitStatisticsVector& bit_volatility_,
                           const uint64_t& image_identifier_ = 0) :
      BinaryMatchable<ObjectType_, descriptor_size_bits_>(object_, descriptor_, image_identifier_),
      bit_probabilities(bit_probabilities_),
      bit_volatility(bit_volatility_) {
    }
  #endif

    ~ProbabilisticMatchable() {
    }

    // 属性
  public:
    // 统计信息：比特概率
    BitStatisticsVector bit_probabilities;

    // 统计信息：比特稳定度，当前未使用
    BitStatisticsVector bit_volatility;
  };

} // namespace srrg_hbst

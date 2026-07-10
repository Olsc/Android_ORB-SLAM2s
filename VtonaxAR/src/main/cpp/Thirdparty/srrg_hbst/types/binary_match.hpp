#pragma once
#include "binary_matchable.hpp"

namespace srrg_hbst {

  //! @class 基础匹配对象：参考 opencv cv::DMatch
  //! (docs.opencv.org/trunk/d4/de0/classcv_1_1DMatch.html)
  //! @param MatchableType_ 匹配对象类型
  //! @param real_precision_ 匹配距离精度
  template <typename BinaryMatchableType_, typename real_type_ = double>
  struct BinaryMatch {
    using Matchable  = BinaryMatchableType_;
    using ObjectType = typename Matchable::ObjectType;
    using real_type  = real_type_;

    //! @brief 默认构造：创建一个未初始化的匹配
    BinaryMatch() : matchable_query(nullptr), distance(0) {
    }

    //! @brief 完整构造函数
    //! @returns 完全初始化的匹配对象
    BinaryMatch(const Matchable* matchable_query_,
                const Matchable* matchable_reference_,
                ObjectType pointer_query_,
                ObjectType pointer_reference_,
                const real_type_& distance_) :
      matchable_query(matchable_query_),
      object_query(pointer_query_),
      distance(distance_) {
      matchable_references.push_back(matchable_reference_);
      object_references.push_back(std::move(pointer_reference_));
    }

    //! @brief 拷贝构造函数
    //! @param[in] match_ 被拷贝的匹配对象
    //! @returns 匹配对象的拷贝
    BinaryMatch(const BinaryMatch& match_) :
      matchable_query(match_.matchable_query),
      matchable_references(std::move(match_.matchable_references)),
      object_query(std::move(match_.object_query)),
      object_references(std::move(match_.object_references)),
      distance(match_.distance) {
    }

    //! @brief 析构函数：无需清理
    ~BinaryMatch() {
    }

    //! @brief 属性
    const Matchable* matchable_query;
    std::vector<const Matchable*>
      matchable_references; // 当匹配距离相同时可能有多个参考对象
    ObjectType object_query;
    std::vector<ObjectType>
      object_references; // 当匹配距离相同时可能有多个参考索引
    real_type distance;
  };
} // namespace srrg_hbst

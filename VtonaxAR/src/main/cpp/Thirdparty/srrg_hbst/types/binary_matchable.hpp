#pragma once
#include <assert.h>
#include <bitset>
#include <map>
#include <stdint.h>
#include <vector>

// 如果构建系统上有 OpenCV
#ifdef SRRG_HBST_HAS_OPENCV
#include <opencv2/core/version.hpp>
#include <opencv2/opencv.hpp>
#endif

namespace srrg_hbst {

  //! @class 默认匹配对象（包装输入描述子等）
  //! @param descriptor_size_bits_ 描述子的位数
  template <typename ObjectType_, uint32_t descriptor_size_bits_ = 256>
  class BinaryMatchable {
    // 类型导出
  public:
    //! @brief 描述子类型
    using Descriptor = std::bitset<descriptor_size_bits_>;
    using ObjectType = ObjectType_;
    using ObjectMap  = std::map<uint64_t, ObjectType>;

    // 共享属性
  public:
    //! @brief 描述子位数（所有 matchable 共享）
    static constexpr uint32_t descriptor_size_bits = descriptor_size_bits_;

    //! @brief 描述子字节数
    static constexpr uint32_t raw_descriptor_size_bytes = descriptor_size_bits_ / 8;

    //! @brief 整字节对应的位数
    static constexpr uint32_t descriptor_size_bits_in_bytes = raw_descriptor_size_bytes * 8;

    //! @brief 溢出的零散位（通常为0）
    static constexpr uint32_t descriptor_size_bits_overflow =
      descriptor_size_bits - descriptor_size_bits_in_bytes;

    // 构造/析构
  public:
    //! @brief 默认构造：禁用
    BinaryMatchable() = delete;

    //! @brief 带对象指针的构造函数
    //! @param[in] object_ 关联对象
    //! @param[in] descriptor_ HBST 描述子
    //! @param[in] image_identifier_ 图像标识（可选）
    BinaryMatchable(ObjectType object_,
                    const Descriptor& descriptor_,
                    const uint64_t& image_identifier_ = 0) :
      descriptor(descriptor_),
      number_of_objects(1),
      _image_identifier(image_identifier_),
      _object(std::move(object_)) {
      objects.insert(std::make_pair(_image_identifier, _object));
      assert(number_of_objects == objects.size());
    }

    //! @brief 从对象映射构造
    BinaryMatchable(ObjectMap objects_, const Descriptor& descriptor_) :
      descriptor(descriptor_),
      objects(objects_),
      number_of_objects(objects_.size()),
      _image_identifier(objects_.begin()->first),
      _object(objects_.begin()->second) {
      assert(number_of_objects == objects.size());
    }

  // 包装构造函数：仅在构建系统有 OpenCV 时可用
  // 此方式较慢，因为需要执行 getDescriptor() 转换
  #ifdef SRRG_HBST_HAS_OPENCV

    //! @brief 带对象指针的构造函数（使用 OpenCV 描述子）
    //! @param[in] object_ 关联对象
    //! @param[in] descriptor_ OpenCV 描述子
    //! @param[in] image_identifier_ 图像标识（可选）
    BinaryMatchable(ObjectType object_,
                    const cv::Mat& descriptor_,
                    const uint64_t& image_identifier_ = 0) :
      BinaryMatchable(object_, getDescriptor(descriptor_), image_identifier_) {
    }
  #endif

    //! @brief 析构函数
    virtual ~BinaryMatchable() {
      objects.clear();
    }

    // 功能函数
  public:
    //! @brief 计算经典的 Hamming 描述子距离
    //! @param[in] matchable_query_ 与之比较的 matchable
    //! @returns 匹配距离（整型）
    inline const uint32_t
    distance(const BinaryMatchable<ObjectType_, descriptor_size_bits_>* matchable_query_) const {
      return (matchable_query_->descriptor ^ this->descriptor).count();
    }

  #ifdef SRRG_MERGE_DESCRIPTORS
    //! @brief 将另一个 matchable 合并到当前对象（用于存储相同描述子时去重）
    //! @param[in] matchable_ 要合并的 matchable
    inline void merge(const BinaryMatchable<ObjectType_, descriptor_size_bits_>* matchable_) {
      objects.insert(matchable_->objects.begin(), matchable_->objects.end());
      number_of_objects += matchable_->objects.size();
      assert(number_of_objects == objects.size());
    }

    //! @brief 快速合并且仅含单条目的 matchable
    inline void mergeSingle(const BinaryMatchable<ObjectType_, descriptor_size_bits_>* matchable_) {
      objects.insert(std::make_pair(matchable_->_image_identifier, std::move(matchable_->_object)));
      ++number_of_objects;
      assert(number_of_objects == objects.size());
    }
  #endif

    //! @brief 手动更新内部链接的对象
    inline void setObject(ObjectType object_) {
      _object = std::move(object_);
    }

    //! @brief 更新所有关联对象
    inline void setObjects(ObjectType object_) {
      setObject(object_);
      for (auto& object : objects) {
        object.second = std::move(object_);
      }
    }

  #ifdef SRRG_HBST_HAS_OPENCV
    //! @brief 描述子转换：将 OpenCV 描述子转为 HBST 格式
    //! @param[in] descriptor_cv_ 要转换的 OpenCV 描述子
    static inline Descriptor getDescriptor(const cv::Mat& descriptor_cv_) {
      Descriptor binary_descriptor(descriptor_size_bits_);

      const uchar* p = descriptor_cv_.ptr<uchar>(0);

      // 完全展开：逐字节拷贝8个比特，避免循环计数分支以提升效率
      for (uint32_t byte_index = 0; byte_index < raw_descriptor_size_bytes; ++byte_index) {
        const uint32_t bit_index_start = byte_index * 8;
        const uchar b = p[byte_index];

        binary_descriptor[bit_index_start + 0] = b & 1;
        binary_descriptor[bit_index_start + 1] = (b >> 1) & 1;
        binary_descriptor[bit_index_start + 2] = (b >> 2) & 1;
        binary_descriptor[bit_index_start + 3] = (b >> 3) & 1;
        binary_descriptor[bit_index_start + 4] = (b >> 4) & 1;
        binary_descriptor[bit_index_start + 5] = (b >> 5) & 1;
        binary_descriptor[bit_index_start + 6] = (b >> 6) & 1;
        binary_descriptor[bit_index_start + 7] = (b >> 7) & 1;
      }

      // 处理剩余零散位
      if (descriptor_size_bits_overflow > 0) {
        const uchar b = p[raw_descriptor_size_bytes];
        for (uint32_t v = 0; v < descriptor_size_bits_overflow; ++v) {
          binary_descriptor[descriptor_size_bits_in_bytes + v] =
            (b >> (8 - descriptor_size_bits_overflow + v)) & 1;
        }
      }
      return binary_descriptor;
    }
  #endif

    // 属性
  public:
    //! @brief 描述子数据
    const Descriptor descriptor;

    //! @brief 关联的对象映射 — 使用此字段时需确保引用对象持久有效
    ObjectMap objects;

    //! @brief 包含的对象/图像标识数量（默认: 1）
    uint64_t number_of_objects;

    // 快速访问（仅含单值的 matchable，内部使用）
  protected:
    //! @brief 单值访问：关联对象组（如图像或图像索引）
    const uint64_t _image_identifier;

    //! @brief 单值访问：描述子关联对象（如关键点或索引）
    ObjectType _object;

    //! @brief 允许 BinaryTree 直接访问
    template <typename BinaryNodeType_>
    friend class BinaryTree;
  };

  template <typename ObjectType_, uint32_t descriptor_size_bits_>
  constexpr uint32_t BinaryMatchable<ObjectType_, descriptor_size_bits_>::descriptor_size_bits;
  template <typename ObjectType_, uint32_t descriptor_size_bits_>
  constexpr uint32_t BinaryMatchable<ObjectType_, descriptor_size_bits_>::raw_descriptor_size_bytes;
  template <typename ObjectType_, uint32_t descriptor_size_bits_>
  constexpr uint32_t
    BinaryMatchable<ObjectType_, descriptor_size_bits_>::descriptor_size_bits_in_bytes;
  template <typename ObjectType_, uint32_t descriptor_size_bits_>
  constexpr uint32_t
    BinaryMatchable<ObjectType_, descriptor_size_bits_>::descriptor_size_bits_overflow;

  template <typename ObjectType_>
  using BinaryMatchable128 = BinaryMatchable<ObjectType_, 128>;
  template <typename ObjectType_>
  using BinaryMatchable256 = BinaryMatchable<ObjectType_, 256>;
  template <typename ObjectType_>
  using BinaryMatchable512 = BinaryMatchable<ObjectType_, 512>;

} // namespace srrg_hbst

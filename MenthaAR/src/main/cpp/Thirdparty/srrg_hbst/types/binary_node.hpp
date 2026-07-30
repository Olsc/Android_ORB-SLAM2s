#pragma once
#include <cmath>
#include <random>

#include "binary_match.hpp"

namespace srrg_hbst {

  //! @brief 叶子生成模式
  enum SplittingStrategy { DoNothing, SplitEven, SplitUneven, SplitRandomUniform };

  template <typename BinaryMatchableType_, typename real_type_ = double>
  class BinaryNode {
    // ds 可读性
    using Node = BinaryNode<BinaryMatchableType_, real_type_>;

    // ds 导出类型
  public:
    using BaseNode        = Node;
    using Matchable       = BinaryMatchableType_;
    using MatchableVector = std::vector<Matchable*>;
    using Descriptor      = typename Matchable::Descriptor;
    using real_type       = real_type_;
    using Match           = BinaryMatch<Matchable, real_type>;

    //! @brief 序列化/反序列化的头部，TODO 与属性合并
    struct Header {
      Header(const uint64_t& depth_) : depth(depth_) {
      }
      Header() : Header(0) {
      }
      uint64_t depth;
      uint64_t number_of_matchables_uncompressed = 0;
      uint64_t number_of_matchables_compressed   = 0;
    };

    // ds 构造函数/析构函数
  public:
    // ds 仅通过此构造函数访问：未提供掩码
    BinaryNode(const MatchableVector& matchables_,
               const SplittingStrategy& train_mode_ = SplittingStrategy::SplitEven) :
      Node(nullptr, 0, matchables_, Descriptor().set(), train_mode_) {
    }

    // ds 仅通过此构造函数访问：提供了掩码
    BinaryNode(const MatchableVector& matchables_,
               Descriptor bit_mask_,
               const SplittingStrategy& train_mode_ = SplittingStrategy::SplitEven) :
      Node(nullptr, 0, matchables_, bit_mask_, train_mode_) {
    }

    // ds 默认构造函数由子类触发 - 属性初始化的责任留给子类
    // ds 这是必需的，因为我们不想在子类中触发基类的自动叶子生成
    BinaryNode() {
    }

    // ds 析构函数：递归销毁子节点（有风险但可读性好）
    virtual ~BinaryNode() {
      delete left;
      delete right;
    }

    // ds 访问接口
  public:
    // ds 创建叶子节点（供外部使用）
    virtual const bool spawnLeafs(const SplittingStrategy& train_mode_) {
      assert(!has_leafs);
      _header.number_of_matchables_compressed = matchables.size();

      // ds 如果达到最大深度则退出
      if (_header.depth == maximum_depth) {
        return false;
      }

      // ds 如果数据不足则退出
      if (_header.number_of_matchables_uncompressed < maximum_leaf_size) {
        return false;
      }

      // ds 确认初始状态
      index_split_bit         = -1;
      number_of_on_bits_total = 0;
      partitioning            = maximum_partitioning;

      // ds 用于平衡分割
      switch (train_mode_) {
        case SplittingStrategy::SplitEven: {
          // ds 必须找到此节点的分割点 - 扫描所有索引
          for (uint32_t bit_index = 0; bit_index < Matchable::descriptor_size_bits; ++bit_index) {
            // ds 如果此索引在掩码中可用
            if (bit_mask[bit_index]) {
              // ds 临时置位位数
              uint64_t number_of_set_bits = 0;

              // ds 计算该索引的划分程度（0.0 为最优）
              const double partitioning_current =
                std::fabs(0.5 - _getSetBitFraction(bit_index, matchables, number_of_set_bits));

              // ds 如果更好
              if (partitioning_current < partitioning) {
                partitioning            = partitioning_current;
                number_of_on_bits_total = number_of_set_bits;
                index_split_bit         = bit_index;

                // ds 如果达到最大目标则结束循环
                if (partitioning == 0)
                  break;
              }
            }
          }
          break;
        }
        case SplittingStrategy::SplitUneven: {
          partitioning = 0;

          // ds 必须找到此节点的分割点 - 扫描所有索引
          for (uint32_t bit_index = 0; bit_index < Matchable::descriptor_size_bits; ++bit_index) {
            // ds 如果此索引在掩码中可用
            if (bit_mask[bit_index]) {
              // ds 临时置位位数
              uint64_t number_of_set_bits = 0;

              // ds 计算该索引的划分程度（0.0 为最优）
              const double partitioning_current =
                std::fabs(0.5 - _getSetBitFraction(bit_index, matchables, number_of_set_bits));

              // ds 如果更差
              if (partitioning_current > partitioning) {
                partitioning            = partitioning_current;
                number_of_on_bits_total = number_of_set_bits;
                index_split_bit         = bit_index;

                // ds 如果达到最大目标则结束循环
                if (partitioning == 0.5)
                  break;
              }
            }
          }
          break;
        }
        case SplittingStrategy::SplitRandomUniform: {
          // ds 计算可用位
          std::vector<uint32_t> available_bits;
          for (uint32_t bit_index = 0; bit_index < Matchable::descriptor_size_bits; ++bit_index) {
            // ds 如果此索引在掩码中可用
            if (bit_mask[bit_index]) {
              available_bits.push_back(bit_index);
            }
          }

          // ds 如果有可用位
          if (available_bits.size() > 0) {
            std::uniform_int_distribution<uint32_t> available_indices(0, available_bits.size() - 1);

            // ds 均匀随机采样
            index_split_bit = available_bits[available_indices(Node::random_number_generator)];

            // ds 计算该索引的划分程度（0.0 为最优）
            partitioning = std::fabs(
              0.5 - _getSetBitFraction(index_split_bit, matchables, number_of_on_bits_total));
          }
          break;
        }
        default: { throw std::runtime_error("invalid leaf spawning mode"); }
      }

      // ds 如果找到了最佳分割且划分程度足够（0 到 0.5） - 可以生成叶子节点
      if (index_split_bit != -1 && partitioning < maximum_partitioning) {
        // ds 获取掩码副本
        Descriptor bit_mask_previous(bit_mask);

        // ds 更新叶子节点的掩码
        bit_mask_previous[index_split_bit] = 0;

        // ds 首先必须根据找到的索引分割描述符 - 预先分配向量，因为知道有多少个置位
        MatchableVector matchables_ones(number_of_on_bits_total);
        MatchableVector matchables_zeros(matchables.size() - number_of_on_bits_total);

        // ds 遍历所有描述符，根据位状态将它们分配到新向量中
        uint64_t index_ones  = 0;
        uint64_t index_zeros = 0;
        for (Matchable* matchable : matchables) {
          if (matchable->descriptor[index_split_bit]) {
            matchables_ones[index_ones] = matchable;
            ++index_ones;
          } else {
            matchables_zeros[index_zeros] = matchable;
            ++index_zeros;
          }
        }
        assert(matchables_ones.size() == index_ones);
        assert(matchables_zeros.size() == index_zeros);

        // ds 此叶子节点变为常规节点，因此不再携带 matchables
        has_leafs = true;
        matchables.clear();
        _header.number_of_matchables_compressed = 0;

        // ds 如果有元素用于叶子节点
        assert(0 < matchables_ones.size());
        right = new Node(this, _header.depth + 1, matchables_ones, bit_mask_previous, train_mode_);

        assert(0 < matchables_zeros.size());
        left = new Node(this, _header.depth + 1, matchables_zeros, bit_mask_previous, train_mode_);

        // ds 成功
        return true;
      } else {
        // ds 生成叶子节点失败 - 终止递归
        return false;
      }
    }

    // ds 获取函数
  public:
    const MatchableVector& getMatchables() const {
      return matchables;
    }
    const uint64_t& getDepth() const {
      return _header.depth;
    }
    const int32_t& indexSplitBit() const {
      return index_split_bit;
    }
    const uint64_t& getNumberOfSetBits() const {
      return number_of_on_bits_total;
    }
    const bool& hasLeafs() const {
      return has_leafs;
    }

    // ds 内部构造函数（用于递归构建树）
  protected:
    // ds 仅在内部调用：单个 matchable 的默认构造函数
    BinaryNode(Node* parent_,
               const uint64_t& depth_,
               const MatchableVector& matchables_,
               Descriptor bit_mask_,
               const SplittingStrategy& train_mode_) :
      parent(parent_),
      _header(depth_),
      matchables(matchables_),
      bit_mask(bit_mask_) {
#ifdef SRRG_MERGE_DESCRIPTORS
      // ds 重新计算当前包含的已合并 matchables 数量 TODO 使其不那么浪费
      _header.number_of_matchables_uncompressed = 0;
      for (const Matchable* matchable : matchables) {
        _header.number_of_matchables_uncompressed += matchable->number_of_objects;
      }
#else
      _header.number_of_matchables_uncompressed = matchables.size();
#endif
      spawnLeafs(train_mode_);
    }

    // ds 辅助函数
  protected:
    const real_type _getSetBitFraction(const uint32_t& index_split_bit_,
                                       const MatchableVector& matchables_,
                                       uint64_t& number_of_set_bits_total_) const {
      assert(0 < matchables_.size());
      assert(0 < _header.number_of_matchables_uncompressed);
      assert(matchables_.size() <= _header.number_of_matchables_uncompressed);

      // ds 计算此节点中所有 matchable 的置位位数
      uint64_t number_of_set_bits = 0;
#ifdef SRRG_MERGE_DESCRIPTORS
      uint64_t number_of_set_bits_actual = 0;
      for (const Matchable* matchable : matchables_) {
        // ds 累加置位的 matchable 计数
        if (matchable->descriptor[index_split_bit_]) {
          // ds 确保加权已合并的 matchable！如果未合并，默认为 1
          number_of_set_bits += matchable->number_of_objects;
          ++number_of_set_bits_actual;
        }
      }

      // ds 此数字用于划分分配
      // ds 因此不能在此处重复计算已合并的 matchable
      number_of_set_bits_total_ = number_of_set_bits_actual;
#else
      for (const Matchable* matchable : matchables_) {
        // ds 只累加位（置位自动计为 1）
        number_of_set_bits += matchable->descriptor[index_split_bit_];
      }
      number_of_set_bits_total_ = number_of_set_bits;
#endif
      assert(number_of_set_bits <= _header.number_of_matchables_uncompressed);

      // ds 返回比例
      return (static_cast<real_type>(number_of_set_bits) /
              _header.number_of_matchables_uncompressed);
    }

    // ds 公有字段
  public:
    //! @brief 包含所有未置位位的叶子节点
    Node* left = nullptr;

    //! @brief 包含所有已置位位的叶子节点
    Node* right = nullptr;

    //! @brief 父节点（如果有，根节点 parent=0）
    Node* parent = nullptr;

    //! @brief 节点分割前的最小 matchable 数量
    static uint64_t maximum_leaf_size;

    //! @brief 使用 index_split_bit 达到的最大描述符组划分程度
    static real_type maximum_partitioning;

    //! @brief 最大树深度（达到后停止生成叶子节点，默认值：描述符维度）
    static uint32_t maximum_depth;

    // ds 字段
  protected:
    //! @brief 可序列化的头部，携带核心属性
    Header _header;

    //! @brief 此节点包含的 matchables
    MatchableVector matchables;

    //! @brief 分割当前节点潜在叶子节点的分割位
    int32_t index_split_bit = -1;

    //! @brief 值为 1 的位数
    uint64_t number_of_on_bits_total = 0;

    //! @brief 当前节点是否有 2 个叶子节点的标志
    bool has_leafs = false;

    //! @brief 使用 index_split_bit 达到的描述符组划分程度
    real_type partitioning = 1;

    //! @brief 在选择 index_split_bit 之前考虑的位分割掩码
    Descriptor bit_mask;

    // ds 随机数生成器，用于随机分割（适用于所有节点）
    static std::mt19937 random_number_generator;

    //! @brief 允许处理类直接访问
    template <typename BinaryNodeType_>
    friend class BinaryTree;
  };

  // ds 默认配置
  template <typename BinaryMatchableType_, typename real_type_>
  uint64_t BinaryNode<BinaryMatchableType_, real_type_>::maximum_leaf_size = 30;
  template <typename BinaryMatchableType_, typename real_type_>
  real_type_ BinaryNode<BinaryMatchableType_, real_type_>::maximum_partitioning = 0.1;
  template <typename BinaryMatchableType_, typename real_type_>
  uint32_t BinaryNode<BinaryMatchableType_, real_type_>::maximum_depth =
    BinaryMatchableType_::descriptor_size_bits;
  template <typename BinaryMatchableType_, typename real_type_>
  std::mt19937 BinaryNode<BinaryMatchableType_, real_type_>::random_number_generator;

  template <typename ObjectType_>
  using BinaryNode128 = BinaryNode<BinaryMatchable128<ObjectType_>>;
  template <typename ObjectType_>
  using BinaryNode256 = BinaryNode<BinaryMatchable256<ObjectType_>>;
  template <typename ObjectType_>
  using BinaryNode512 = BinaryNode<BinaryMatchable512<ObjectType_>>;

} // namespace srrg_hbst

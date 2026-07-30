#pragma once
#include "probabilistic_matchable.hpp"
#include "srrg_hbst/types/binary_node.hpp"

namespace srrg_hbst {

  // 针对 CDescriptorBinaryProbabilistic 描述子类型的节点模板特化
  template <typename ProbabilisticMatchableType_, typename real_precision_ = double>
  class ProbabilisticNode : public BinaryNode<ProbabilisticMatchableType_, real_precision_> {
    // 可读性
    using Node = ProbabilisticNode<ProbabilisticMatchableType_, real_precision_>;

    // 模板类型转发
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef BinaryNode<ProbabilisticMatchableType_, real_precision_> BaseNode;
    typedef ProbabilisticMatchableType_ Matchable;
    typedef typename Matchable::Descriptor Descriptor;
    typedef std::vector<Matchable*> MatchableVector;
    typedef real_precision_ precision;
    typedef BinaryMatch<Matchable, precision> Match;
    typedef typename Matchable::BitStatisticsVector BitStatisticsVector;

    // 构造/析构
  public:
    // 只能通过此构造函数访问：无掩码
    ProbabilisticNode(const MatchableVector& matchables_,
                      const SplittingStrategy& train_mode_ = SplittingStrategy::SplitEven) :
      Node(0, matchables_, Descriptor().set(), train_mode_) {
    }

    // 默认构造函数由子类触发，属性初始化由子类负责
    // 需要默认构造函数，因为子类中不想触发基类的自动叶子生成
    ProbabilisticNode() {
    }

    // 析构函数：无需清理（叶子将由树释放）
    virtual ~ProbabilisticNode() {
    }

    // 访问
  public:
    // 实现新的叶子生成函数以处理修改后的描述子
    virtual const bool spawnLeafs(const SplittingStrategy& train_mode_) override {
      // 缓存描述子数量
      const uint64_t number_of_matchables = this->matchables.size();

      // 至少有2个描述子时才分裂
      if (1 < number_of_matchables) {
        assert(!this->has_leafs);

        // 初始化
        this->index_split_bit            = -1;
        this->number_of_on_bits_total    = 0;
        this->partitioning               = 1;
        real_precision_ variance_maximum = 0;

        // 方差计算统计
        BitStatisticsVector bit_probabilities_accumulated(BitStatisticsVector::Zero());

        // 遍历此节点中的所有描述子
        for (const Matchable* matchable : this->matchables) {
          bit_probabilities_accumulated += matchable->bit_probabilities;
        }

        // 计算均值
        const BitStatisticsVector bit_probabilities_mean(bit_probabilities_accumulated /
                                                         number_of_matchables);

        // 计算方差
        for (uint32_t index_bit = 0; index_bit < Matchable::descriptor_size_bits; ++index_bit) {
          if (this->bit_mask[index_bit]) {
            real_precision_ variance_current = 0.0;

            for (const Matchable* matchable : this->matchables) {
              const real_precision_ delta =
                matchable->bit_probabilities[index_bit] - bit_probabilities_mean[index_bit];
              variance_current += delta * delta;
            }

            variance_current /= number_of_matchables;

            if (variance_maximum < variance_current) {
              variance_maximum      = variance_current;
              this->index_split_bit = index_bit;
            }
          }
        }

        // 找到最佳分裂位
        if (-1 != this->index_split_bit) {
          this->partitioning = std::fabs(
            0.5 - this->_getSetBitFraction(
                    this->index_split_bit, this->matchables, this->number_of_on_bits_total));

          // 检查是否有足够数据分裂
          if (0 < this->number_of_on_bits_total && 0.5 > this->partitioning) {
            this->has_leafs = true;

            Descriptor mask(this->bit_mask);
            mask[this->index_split_bit] = 0;

            // 按分裂位分割描述子
            MatchableVector matchables_leaf_ones;
            matchables_leaf_ones.reserve(this->number_of_on_bits_total);
            MatchableVector matchables_leaf_zeroes;
            matchables_leaf_zeroes.reserve(number_of_matchables - this->number_of_on_bits_total);

            for (Matchable* matchable : this->matchables) {
              if (matchable->descriptor[this->index_split_bit]) {
                matchables_leaf_ones.push_back(matchable);
              } else {
                matchables_leaf_zeroes.push_back(matchable);
              }
            }

            assert(0 < matchables_leaf_ones.size());
            this->right = new Node(this->depth + 1, matchables_leaf_ones, mask, train_mode_);

            assert(0 < matchables_leaf_zeroes.size());
            this->left = new Node(this->depth + 1, matchables_leaf_zeroes, mask, train_mode_);

            return true;
          } else {
            return false;
          }
        } else {
          return false;
        }
      } else {
        return false;
      }
    }

    // 内部构造函数（用于递归建树）
  protected:
    // 仅内部调用：无分裂顺序
    ProbabilisticNode(const uint64_t& depth_,
                      const MatchableVector& matchables_,
                      Descriptor bit_mask_,
                      const SplittingStrategy& train_mode_) {
      this->depth      = depth_;
      this->matchables = matchables_;
      this->has_leafs  = false;
      this->bit_mask   = bit_mask_;
      this->right      = 0;
      this->left       = 0;

      spawnLeafs(train_mode_);
    }
  };
} // namespace srrg_hbst

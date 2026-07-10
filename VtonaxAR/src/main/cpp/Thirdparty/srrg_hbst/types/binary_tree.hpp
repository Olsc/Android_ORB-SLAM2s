#pragma once
#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <unordered_map>

#include "binary_node.hpp"

// ds 受控读写操作的辅助宏
#define GUARDED_IO(FILE, IO_OPERATION, VARIABLE, SIZE, ERROR_MESSAGE) \
  if (!FILE.IO_OPERATION(VARIABLE, SIZE)) {                           \
    std::cerr << ERROR_MESSAGE << std::endl;                          \
    FILE.close();                                                     \
    return false;                                                     \
  }

namespace srrg_hbst {

  //! @class 二叉樹類，由持有二值描述子的二值節點組成
  template <typename BinaryNodeType_>
  class BinaryTree {
    // ds 導出類型
  public:
    //! @brief 直接導出的類型（提高可讀性）
    using Node                  = typename BinaryNodeType_::BaseNode;
    using Matchable             = typename Node::Matchable;
    using MatchableVector       = typename Node::MatchableVector;
    using Descriptor            = typename Node::Descriptor;
    using Match                 = typename Node::Match;
    using real_type             = typename Node::real_type;
    using ObjectType            = typename Matchable::ObjectType;
    using ObjectMap             = typename Matchable::ObjectMap;
    using ObjectMapElement      = std::pair<uint64_t, ObjectType>;
    using MatchVector           = std::vector<Match>;
    using MatchVectorMap        = std::unordered_map<uint64_t, std::vector<Match>>;
    using MatchVectorMapElement = std::pair<uint64_t, std::vector<Match>>;
    using MatchMap              = std::vector<std::pair<uint64_t, Match>>;

#ifdef SRRG_MERGE_DESCRIPTORS
    //! @brief 用於可匹配對象合併的組件對象
    struct MatchableMerge {
      MatchableMerge(const Matchable* query_, ObjectType query_object_, Matchable* reference_) :
        query(query_),
        query_object(query_object_),
        reference(reference_) {
      }
      const Matchable* query;  // 將被吸收的可匹配對象
      ObjectType query_object; // 受影響的對象，可能需要更新其可匹配對象引用
      Matchable* reference;    // 吸收方的可匹配對象
    };
    typedef std::vector<MatchableMerge> MatchableMergeVector;
#endif

    //! @brief 用於樹訓練的組件對象
    struct Trainable {
      Node* node           = nullptr;
      Matchable* matchable = nullptr;
    };

    //! @brief 參考圖像的圖像得分（已添加）
    struct Score {
      uint64_t number_of_matches    = 0;
      real_type matching_ratio      = 0;
      uint64_t identifier_reference = 0;
    };
    typedef std::vector<Score> ScoreVector;

    //! @brief 包含主要屬性的對象頭部
    struct Header {
      Header(const uint64_t& identifier_ = 0) :
        identifier(identifier_),
        number_of_matchables_compressed(0),
        number_of_training_entries(0),
        number_of_matchables_uncompressed(0),
        number_of_leafs(0) {
      }
      uint64_t identifier;
      uint64_t number_of_matchables_compressed;
      uint64_t number_of_training_entries;
      uint64_t number_of_matchables_uncompressed;
      uint64_t number_of_leafs;
#ifdef SRRG_MERGE_DESCRIPTORS
      static constexpr bool srrg_merge_descriptors = true;
#else
      static constexpr bool srrg_merge_descriptors = false;
#endif
      static constexpr size_t descriptor_size_bits = Matchable::descriptor_size_bits;
    };

    // ds 構造函數/析構函數
  public:
    // ds 使用特定標識符的空樹實例化
    BinaryTree(const uint64_t& identifier_) : _header(identifier_), _root(nullptr) {
      _matchables.clear();
      _matchables_to_train.clear();
      _added_identifiers_train.clear();
      _trainables.clear();
#ifdef SRRG_MERGE_DESCRIPTORS
      _merged_matchables.clear();
#endif
    }

    // ds 空樹實例化
    BinaryTree() : BinaryTree(0) {
    }

    // ds 基於篩選後的描述子構建樹
    BinaryTree(const uint64_t& identifier_,
               const MatchableVector& matchables_,
               const SplittingStrategy& train_mode_ = SplittingStrategy::SplitEven) :
      _header(identifier_),
      _root(new Node(matchables_, train_mode_)) {
      _matchables.clear();
      _matchables.insert(_matchables.end(), matchables_.begin(), matchables_.end());
      _matchables_to_train.clear();
      _added_identifiers_train.clear();
      _added_identifiers_train.insert(_header.identifier);
      _trainables.clear();
#ifdef SRRG_MERGE_DESCRIPTORS
      _merged_matchables.clear();
#endif
    }

    // ds 基於篩選後的描述子構建樹
    BinaryTree(const MatchableVector& matchables_,
               const SplittingStrategy& train_mode_ = SplittingStrategy::SplitEven) :
      BinaryTree(0, matchables_, train_mode_) {
    }

    // ds 基於篩選後的描述子構建樹：帶位掩碼
    BinaryTree(const uint64_t& identifier_,
               const MatchableVector& matchables_,
               Descriptor bit_mask_,
               const SplittingStrategy& train_mode_ = SplittingStrategy::SplitEven) :
      _header(identifier_),
      _root(new Node(matchables_, bit_mask_, train_mode_)) {
      _matchables.clear();
      _matchables.insert(_matchables.end(), matchables_.begin(), matchables_.end());
      _matchables_to_train.clear();
      _added_identifiers_train.clear();
      _added_identifiers_train.insert(_header.identifier);
      _trainables.clear();
#ifdef SRRG_MERGE_DESCRIPTORS
      _merged_matchables.clear();
#endif
    }

    // ds 刪除拷貝構造函數和賦值操作符以避免淺拷貝錯誤
    BinaryTree(const BinaryTree&) = delete;
    BinaryTree& operator=(const BinaryTree&) = delete;

    // ds 移動構造函數
    BinaryTree(BinaryTree&& other) noexcept :
      _header(std::move(other._header)),
      _root(other._root),
      _matchables(std::move(other._matchables)),
      _matchables_to_train(std::move(other._matchables_to_train)),
      _added_identifiers_train(std::move(other._added_identifiers_train)),
      _trainables(std::move(other._trainables)) {
#ifdef SRRG_MERGE_DESCRIPTORS
      _merged_matchables = std::move(other._merged_matchables);
      _number_of_merged_matchables_last_training = other._number_of_merged_matchables_last_training;
#endif
      other._root = nullptr;
      other._matchables.clear();
      other._matchables_to_train.clear();
      other._added_identifiers_train.clear();
      other._trainables.clear();
#ifdef SRRG_MERGE_DESCRIPTORS
      other._merged_matchables.clear();
      other._number_of_merged_matchables_last_training = 0;
#endif
    }

    // ds 移動賦值操作符
    BinaryTree& operator=(BinaryTree&& other) noexcept {
      if (this != &other) {
        clear();
        _header = std::move(other._header);
        _root = other._root;
        _matchables = std::move(other._matchables);
        _matchables_to_train = std::move(other._matchables_to_train);
        _added_identifiers_train = std::move(other._added_identifiers_train);
        _trainables = std::move(other._trainables);
#ifdef SRRG_MERGE_DESCRIPTORS
        _merged_matchables = std::move(other._merged_matchables);
        _number_of_merged_matchables_last_training = other._number_of_merged_matchables_last_training;
#endif
        other._root = nullptr;
        other._matchables.clear();
        other._matchables_to_train.clear();
        other._added_identifiers_train.clear();
        other._trainables.clear();
#ifdef SRRG_MERGE_DESCRIPTORS
        other._merged_matchables.clear();
        other._number_of_merged_matchables_last_training = 0;
#endif
      }
      return *this;
    }

    // ds 釋放樹中所有節點而不釋放可匹配對象 - 調用 clear(true)
    ~BinaryTree() {
      clear();
    }

    // ds 共享指針訪問包裝器
  public:
    const uint64_t
    getNumberOfMatches(const std::shared_ptr<const MatchableVector> matchables_query_,
                       const uint32_t& maximum_distance_ = 25) const {
      return getNumberOfMatches(*matchables_query_, maximum_distance_);
    }

    const typename Node::real_type
    getMatchingRatio(const std::shared_ptr<const MatchableVector> matchables_query_,
                     const uint32_t& maximum_distance_ = 25) const {
      return getMatchingRatio(*matchables_query_, maximum_distance_);
    }

    const uint64_t
    getNumberOfMatchesLazy(const std::shared_ptr<const MatchableVector> matchables_query_,
                           const uint32_t& maximum_distance_ = 25) const {
      return getNumberOfMatchesLazy(*matchables_query_, maximum_distance_);
    }

    const typename Node::real_type
    getMatchingRatioLazy(const std::shared_ptr<const MatchableVector> matchables_query_,
                         const uint32_t& maximum_distance_ = 25) const {
      return getMatchingRatioLazy(*matchables_query_, maximum_distance_);
    }

    const typename Node::real_type
    getMatchingRatioLazy(const MatchableVector& matchables_query_,
                         const uint32_t& maximum_distance_ = 25) const {
      return static_cast<typename Node::real_type>(
               getNumberOfMatchesLazy(matchables_query_, maximum_distance_)) /
             matchables_query_.size();
    }

    const typename Node::real_type getMatchingRatio(const MatchableVector& matchables_query_,
                                                    const uint32_t& maximum_distance_ = 25) const {
      return static_cast<typename Node::real_type>(
               getNumberOfMatches(matchables_query_, maximum_distance_)) /
             matchables_query_.size();
    }

    //! @brief 返回數據庫大小（即已添加的具有唯一標識符的圖像/可匹配對象向量的數量）
    //! @returns 已添加的參考可匹配對象向量的數量
    const size_t size() const {
      assert(_header.number_of_training_entries == _added_identifiers_train.size());
      return _header.number_of_training_entries;
    }

    const std::set<uint64_t>& trainedIdentifiers() const {
      return _added_identifiers_train;
    }

    //! @brief 返回為訓練添加的可匹配對象數量（壓縮前）
    //! @returns 可匹配對象數量
    const size_t numberOfMatchablesUncompressed() const {
      return _header.number_of_matchables_uncompressed;
    }

    //! @brief 返回實際存儲的可匹配對象數量（壓縮後）
    //! 若 SRRG_MERGE_DESCRIPTORS 未啟用，該數值始終等於未壓縮數量
    //! @returns 可匹配對象數量
    const size_t numberOfMatchablesCompressed() const {
      return _header.number_of_matchables_compressed;
    }

    //! @brief 根節點的常量訪問器
    const Node* root() const {
      return _root;
    }

    // 获取存储的描述子向量以进行快速直接读取，避免重复从Mat转换
    const MatchableVector& matchables() const {
      return _matchables;
    }

    //! 上次調用中合併的可匹配對象數量
    const size_t numberOfMergedMatchablesLastTraining() const {
#ifdef SRRG_MERGE_DESCRIPTORS
      return _number_of_merged_matchables_last_training;
#else
      // ds 若未啟用始終為零
      return 0;
#endif
    }

#ifdef SRRG_MERGE_DESCRIPTORS
    //! @brief 上次 match/add/train 調用中的可匹配對象合併 - 僅用於外部簿記更新
    //! @returns 可匹配對象間有效合併的向量（注意：Mergable.query 的內存已被釋放）
    const MatchableMergeVector getMerges() const {
      return _merged_matchables;
    }
#endif

    const uint64_t getNumberOfMatches(const MatchableVector& matchables_query_,
                                      const uint32_t& maximum_distance_ = 25) const {
      if (matchables_query_.empty()) {
        return 0;
      }
      uint64_t number_of_matches = 0;

      // ds 對每個描述子
      for (const Matchable* matchable_query : matchables_query_) {
        // ds 遍歷樹以查找此描述子
        const Node* node_current = _root;
        while (node_current) {
          // ds 若此節點有葉子（可分裂）
          if (node_current->has_leafs) {
            // ds 檢查分裂位並深入
            if (matchable_query->descriptor[node_current->index_split_bit]) {
              node_current = node_current->right;
            } else {
              node_current = node_current->left;
            }
          } else {
            // ds 檢查此節點中的當前描述子並退出
            for (const Matchable* matchable_reference : node_current->matchables) {
              if (maximum_distance_ > matchable_query->distance(matchable_reference)) {
                ++number_of_matches;
                break;
              }
            }
            break;
          }
        }
      }
      return number_of_matches;
    }

    const ScoreVector getScorePerImage(const MatchableVector& matchables_query_,
                                       const bool sort_output           = false,
                                       const uint32_t maximum_distance_ = 25) const {
      if (matchables_query_.empty()) {
        return ScoreVector(0);
      }
      ScoreVector scores_per_image(_added_identifiers_train.size());

      // ds 標識符到向量索引的映射 - 同時初始化結果向量
      std::map<uint64_t, uint64_t> mapping_identifier_image_to_score;
      for (const uint64_t& identifier_reference : _added_identifiers_train) {
        scores_per_image[mapping_identifier_image_to_score.size()].identifier_reference =
          identifier_reference;
        mapping_identifier_image_to_score.insert(
          std::make_pair(identifier_reference, mapping_identifier_image_to_score.size()));
      }

      // ds 對每個查詢描述子
      for (const Matchable* matchable_query : matchables_query_) {
        // ds 遍歷樹以查找此描述子
        const Node* node_current = _root;
        while (node_current) {
          // ds 若此節點有葉子（可分裂）
          if (node_current->has_leafs) {
            // ds 檢查分裂位並深入
            if (matchable_query->descriptor[node_current->index_split_bit]) {
              node_current = node_current->right;
            } else {
              node_current = node_current->left;
            }
          } else {
            // ds 檢查此節點中每個參考圖像的當前描述子並退出
            std::set<uint64_t> matched_references;
            for (const Matchable* matchable_reference : node_current->matchables) {
              if (matchable_query->distance(matchable_reference) < maximum_distance_) {
#ifdef SRRG_MERGE_DESCRIPTORS
                for (const ObjectMapElement& object : matchable_reference->objects) {
                  const uint64_t& identifier_reference = object.first;
#else
                const uint64_t& identifier_reference = matchable_reference->_image_identifier;
#endif

                  // ds 查詢可匹配對象只能與每個參考圖像匹配一次
                  if (matched_references.count(identifier_reference) == 0) {
                    ++scores_per_image[mapping_identifier_image_to_score.at(identifier_reference)]
                        .number_of_matches;
                    matched_references.insert(identifier_reference);
                  }
#ifdef SRRG_MERGE_DESCRIPTORS
                }
#endif
              }
            }
            break;
          }
        }
      }

      // ds 計算相對得分
      const real_type number_of_query_descriptors = matchables_query_.size();
      for (Score& score : scores_per_image) {
        score.matching_ratio = score.number_of_matches / number_of_query_descriptors;
      }

      // ds 若需要，按匹配率降序排序
      if (sort_output) {
        std::sort(
          scores_per_image.begin(), scores_per_image.end(), [](const Score& a, const Score& b) {
            return a.matching_ratio > b.matching_ratio;
          });
      }
      return scores_per_image;
    }

    const uint64_t getNumberOfMatchesLazy(const MatchableVector& matchables_query_,
                                          const uint32_t& maximum_distance_ = 25) const {
      if (matchables_query_.empty()) {
        return 0;
      }
      uint64_t number_of_matches = 0;

      // ds 對每個描述子
      for (const Matchable* matchable_query : matchables_query_) {
        // ds 遍歷樹以查找此描述子
        const Node* node_current = _root;
        while (node_current) {
          // ds 若此節點有葉子（可分裂）
          if (node_current->has_leafs) {
            // ds 檢查分裂位並深入
            if (matchable_query->descriptor[node_current->index_split_bit]) {
              node_current = node_current->right;
            } else {
              node_current = node_current->left;
            }
          } else {
            // ds 檢查此節點中的當前描述子並退出
            if (maximum_distance_ > matchable_query->distance(node_current->matchables.front())) {
              ++number_of_matches;
            }
            break;
          }
        }
      }
      return number_of_matches;
    }

    // ds 此樹上的直接匹配函數
    void matchLazy(const MatchableVector& matchables_query_,
                   MatchVector& matches_,
                   const uint32_t& maximum_distance_ = 25) const {
      if (matchables_query_.empty()) {
        return;
      }

      // ds 對每個描述子
      for (const Matchable* matchable_query : matchables_query_) {
        // ds 遍歷樹以查找此描述子
        const Node* node_current = _root;
        while (node_current) {
          // ds 若此節點有葉子（可分裂）
          if (node_current->has_leafs) {
            // ds 檢查分裂位並深入
            if (matchable_query->descriptor[node_current->index_split_bit]) {
              node_current = node_current->right;
            } else {
              node_current = node_current->left;
            }
          } else {
            // ds 檢查此節點中的當前描述子並退出
            for (const Matchable* matchable_reference : node_current->matchables) {
              const real_type distance = matchable_query->distance(matchable_reference);
              if (distance < maximum_distance_) {
                matches_.push_back(Match(matchable_query,
                                         matchable_reference,
                                         matchable_query->objects.begin()->second,
                                         matchable_reference->objects.begin()->second,
                                         distance));
                break;
              }
            }
            break;
          }
        }
      }
    }

    // ds 此樹上的直接匹配函數
    void match(const MatchableVector& matchables_query_,
               MatchVector& matches_,
               const uint32_t& maximum_distance_ = 25) const {
      if (matchables_query_.empty()) {
        return;
      }

      // ds 對每個描述子
      for (const Matchable* matchable_query : matchables_query_) {
        // ds 遍歷樹以查找此描述子
        const Node* node_current = _root;
        while (node_current) {
          // ds 若此節點有葉子（可分裂）
          if (node_current->has_leafs) {
            // ds 檢查分裂位並深入
            if (matchable_query->descriptor[node_current->index_split_bit]) {
              node_current = node_current->right;
            } else {
              node_current = node_current->left;
            }
          } else {
            // ds 當前最佳匹配（若無則為 nullptr）
            const Matchable* matchable_reference_best = nullptr;
            uint32_t distance_best                    = maximum_distance_;

            // ds 檢查此節點中的當前描述子並退出
            for (const Matchable* matchable_reference : node_current->matchables) {
              const uint32_t& distance = matchable_query->distance(matchable_reference);
              if (distance < distance_best) {
                matchable_reference_best = matchable_reference;
                distance_best            = distance;
              }
            }

            // ds 若找到匹配
            if (matchable_reference_best) {
              matches_.push_back(Match(matchable_query,
                                       matchable_reference_best,
                                       matchable_query->objects.begin()->second,
                                       matchable_reference_best->objects.begin()->second,
                                       distance_best));
            }
            break;
          }
        }
      }
    }

    // ds 直接返回匹配結果
    const std::shared_ptr<const MatchVector>
    getMatchesLazy(const std::shared_ptr<const MatchableVector> matchables_query_,
                   const uint32_t& maximum_distance_ = 25) const {
      // ds 包裝調用
      MatchVector matches;
      matchLazy(*matchables_query_, matches, maximum_distance_);
      return std::make_shared<const MatchVector>(matches);
    }

    //! @brief knn 多匹配函數
    //! @param[in] matchables_query_ 查詢可匹配對象
    //! @param[out] matches_ 輸出匹配結果：包含樹中所有已添加訓練圖像的所有可用匹配
    //! @param[in] maximum_distance_ 正向匹配響應允許的最大距離
    void match(const MatchableVector& matchables_query_,
               MatchVectorMap& matches_,
               const uint32_t& maximum_distance_matching_ = 25) const {
      if (matchables_query_.empty() || _added_identifiers_train.empty()) {
        return;
      }

      // ds 為樹中所有標識符準備匹配向量映射
      matches_.clear();
      for (const uint64_t identifier_tree : _added_identifiers_train) {
        matches_.insert(std::make_pair(identifier_tree, MatchVector()));

        // ds 預分配空間以加速匹配添加
        matches_.at(identifier_tree).reserve(matchables_query_.size());
      }

      // 在循环外部定义并预分配 MatchMap 内存，实现单帧查找的零堆内存分配
      MatchMap best_matches;
      best_matches.reserve(16);

      // ds 對每個描述子
      for (const Matchable* matchable_query : matchables_query_) {
        // ds 遍歷樹以查找此描述子
        const Node* node_current = _root;
        while (node_current) {
          // ds 若此節點有葉子（可分裂）
          if (node_current->has_leafs) {
            // ds 檢查分裂位並深入
            if (matchable_query->descriptor[node_current->index_split_bit]) {
              node_current = node_current->right;
            } else {
              node_current = node_current->left;
            }
          } else {
            // ds 通過暴力搜索在當前葉子中獲取最佳匹配
            best_matches.clear();
            _matchExhaustive(
              matchable_query, node_current->matchables, maximum_distance_matching_, best_matches);

            // ds 在輸出結構中註冊所有匹配
            for (const auto& best_match : best_matches) {
              matches_.at(best_match.first).push_back(best_match.second);
            }
            break;
          }
        }
      }
    }

    //! @brief 增量式增長樹
    //! @param[in] matchables_ 要整合到當前樹中的新輸入可匹配對象（轉移所有權！）
    //! @param[in] train_mode_ 訓練模式
    void add(const MatchableVector& matchables_,
             const SplittingStrategy& train_mode_ = SplittingStrategy::SplitEven) {
      if (matchables_.empty()) {
        return;
      }

      // ds 為訓練準備簿記
      assert(matchables_.front()->_image_identifier == matchables_.back()->_image_identifier);
      _added_identifiers_train.insert(matchables_.front()->_image_identifier);
      ++_header.number_of_training_entries;
      _matchables_to_train.insert(
        _matchables_to_train.end(), matchables_.begin(), matchables_.end());

      // ds 基於設定的可匹配對象進行訓練（若 SplittingStrategy::DoNothing 則無效）
      train(train_mode_);
    }

    //! @brief 根據選定模式用當前的 _trainable_matchables 訓練樹
    //! @param[in] train_mode_ 期望的訓練模式
    void train(const SplittingStrategy& train_mode_ = SplittingStrategy::SplitEven) {
      if (_matchables_to_train.empty() || train_mode_ == SplittingStrategy::DoNothing) {
        return;
      }
      _header.number_of_matchables_uncompressed += _matchables_to_train.size();

      // ds 檢查是否需要先構建初始樹（之後不再訓練）
      if (!_root) {
        _root = new Node(_matchables_to_train, train_mode_);
        assert(_matchables.empty());
        _matchables.insert(
          _matchables.end(), _matchables_to_train.begin(), _matchables_to_train.end());
        _header.number_of_matchables_compressed = _matchables_to_train.size();
        _matchables_to_train.clear();
        return;
      }

      // ds 若選擇了隨機分裂
      if (train_mode_ == SplittingStrategy::SplitRandomUniform) {
        // ds 用新種子初始化隨機數生成器
        std::random_device random_device;
        Node::random_number_generator = std::mt19937(random_device());
      }

      // ds 可匹配對象添加到葉子後需要更新的節點
      std::set<Node*> leafs_to_update;

#ifdef SRRG_MERGE_DESCRIPTORS
      // ds 需要延遲插入，因為我們需要持續掃描當前引用以進行合併
      _trainables.resize(_matchables_to_train.size());

      // ds 需要合併的匹配（描述子距離 == SRRG_MERGE_DESCRIPTORS）
      _merged_matchables.clear();
      _merged_matchables.reserve(_matchables_to_train.size());

      // ds 目前每個參考可匹配對象最多允許合併一次
      std::set<const Matchable*> merged_reference_matchables;
#endif

      // ds 對每個新描述子 - 緩衝新可匹配對象並合併相同的
      uint64_t index_new_matchable = 0;
      for (Matchable* matchable_to_insert : _matchables_to_train) {
        // ds 遍歷樹以找到此描述子的葉子
        Node* node_current = _root;
        while (node_current) {
          // ds 若此節點有葉子（可遍歷）
          if (node_current->has_leafs) {
            // ds 檢查分裂位並遍歷樹
            if (matchable_to_insert->descriptor[node_current->index_split_bit]) {
              node_current = node_current->right;
            } else {
              node_current = node_current->left;
            }
          } else {
            // ds 到達葉子
#ifdef SRRG_MERGE_DESCRIPTORS
            bool insertion_required = true;

            // ds 檢查是否可以吸收此可匹配對象而非插入
            for (const Matchable* matchable_reference : node_current->matchables) {
              // ds 若滿足合併距離
              // ds 且此參考在此次調用中尚未吸收過可匹配對象
              if (matchable_reference->distance(matchable_to_insert) <=
                    maximum_distance_for_merge &&
                  merged_reference_matchables.count(matchable_reference) == 0) {
                assert(matchable_reference != matchable_to_insert);
                assert(matchable_to_insert->objects.size() == 1);
                _merged_matchables.emplace_back(
                  MatchableMerge(matchable_to_insert,
                                 std::move(matchable_to_insert->_object),
                                 const_cast<Matchable*>(matchable_reference)));
                merged_reference_matchables.insert(matchable_reference);
                insertion_required = false;
                break;
              }
            }

            // ds 若需要插入 - 無法合併查詢可匹配對象
            if (insertion_required) {
              // ds 簿記可匹配對象以便添加
              // ds 需要簿記節點以便後續將其添加到其可匹配對象列表中
              _trainables[index_new_matchable].node      = node_current;
              _trainables[index_new_matchable].matchable = matchable_to_insert;
              _matchables_to_train[index_new_matchable]  = matchable_to_insert;
              ++index_new_matchable;
            }
#else
            // ds 可以立即將描述子放入葉子中
            node_current->matchables.push_back(matchable_to_insert);
            _matchables_to_train[index_new_matchable] = matchable_to_insert;
            ++index_new_matchable;
#endif
            // ds 無論是否合併，葉子始終需要更新
            ++node_current->_header.number_of_matchables_uncompressed;
            leafs_to_update.insert(node_current);
            break;
          }
        }
      }
      _matchables_to_train.resize(index_new_matchable);
#ifdef SRRG_MERGE_DESCRIPTORS

      // ds 合併可匹配對象
      for (MatchableMerge& mergable : _merged_matchables) {
        assert(mergable.reference != mergable.query);

        // ds 執行合併
        mergable.reference->mergeSingle(mergable.query);

        // ds 釋放查詢對象（！）注意樹擁有可匹配對象的所有權
        delete mergable.query;
      }
      _number_of_merged_matchables_last_training = _merged_matchables.size();
      _merged_matchables.clear();

      // ds 將可匹配對象插入節點
      _trainables.resize(index_new_matchable);
      assert(_matchables_to_train.size() == _trainables.size());
      for (const Trainable& trainable : _trainables) {
        trainable.node->matchables.push_back(trainable.matchable);
      }
#endif
      // ds 檢查受影響葉子的分裂情況
      for (Node* leaf : leafs_to_update) {
        leaf->spawnLeafs(train_mode_);
      }

      // ds 簿記
      _matchables.insert(
        _matchables.end(), _matchables_to_train.begin(), _matchables_to_train.end());
      _header.number_of_matchables_compressed += _matchables_to_train.size();
      _matchables_to_train.clear();
    }

    //! @brief knn 多匹配函數，同時添加
    //! @param[in] matchables_ 查詢可匹配對象，將自動添加到樹中（轉移所有權！）
    //! @param[out] matches_ 輸出匹配結果：包含樹中所有已添加訓練圖像的所有可用匹配
    //! @param[in] maximum_distance_matching_ 正向匹配響應允許的最大距離
    void matchAndAdd(const MatchableVector& matchables_,
                     MatchVectorMap& matches_,
                     const uint32_t maximum_distance_matching_ = 25,
                     const SplittingStrategy& train_mode_      = SplittingStrategy::SplitEven) {
      if (matchables_.empty()) {
        return;
      }
      const uint64_t identifier_image_query = matchables_.front()->_image_identifier;

      // ds 檢查是否需要先構建初始樹
      if (!_root) {
        _root = new Node(matchables_);
        assert(_matchables.empty());
        _matchables.insert(_matchables.end(), matchables_.begin(), matchables_.end());
        _header.number_of_matchables_compressed = matchables_.size();
        _added_identifiers_train.insert(identifier_image_query);
        assert(_added_identifiers_train.size() == 1);
        _header.number_of_training_entries = 1;
        return;
      }

      // ds 為樹中所有標識符準備匹配向量映射
      matches_.clear();
      for (const uint64_t identifier_tree : _added_identifiers_train) {
        matches_.insert(std::make_pair(identifier_tree, MatchVector()));

        // ds 預分配空間以加速匹配添加
        matches_.at(identifier_tree).reserve(matchables_.size());
      }

      // ds 準備要整合的節點/可匹配對象列表
      _trainables.resize(matchables_.size());
      std::set<Node*> leafs_to_update;

#ifdef SRRG_MERGE_DESCRIPTORS
      // ds 需要合併的匹配（描述子距離 == SRRG_MERGE_DESCRIPTORS）
      _merged_matchables.clear();
      _merged_matchables.reserve(matchables_.size());

      // ds 目前每個參考可匹配對象最多允許合併一次
      std::set<const Matchable*> merged_reference_matchables;
#endif

      // 在循环外部定义并预分配 MatchMap 内存，实现单帧查找的零堆内存分配
      MatchMap best_matches;
      best_matches.reserve(16);

      // ds 對每個描述子
      uint64_t index_trainable = 0;
      for (Matchable* matchable_query : matchables_) {
        // ds 遍歷樹以查找此描述子
        Node* node_current = _root;
        while (node_current) {
          // ds 若此節點有葉子（可分裂）
          if (node_current->has_leafs) {
            // ds 檢查分裂位並深入
            if (matchable_query->descriptor[node_current->index_split_bit]) {
              node_current = node_current->right;
            } else {
              node_current = node_current->left;
            }
          } else {
            // ds 通過暴力搜索在當前葉子中獲取最佳匹配 - 簿記需要合併的匹配（距離 == 0）
            best_matches.clear();
#ifdef SRRG_MERGE_DESCRIPTORS
            Matchable* matchable_reference = nullptr;
            _matchExhaustive(matchable_query,
                             node_current->matchables,
                             maximum_distance_matching_,
                             best_matches,
                             matchable_reference);
#else
            _matchExhaustive(
              matchable_query, node_current->matchables, maximum_distance_matching_, best_matches);
#endif

            // ds 在輸出結構中註冊所有匹配
            for (const auto& best_match : best_matches) {
              matches_.at(best_match.first).push_back(best_match.second);
            }

#ifdef SRRG_MERGE_DESCRIPTORS
            // ds 若可以將查詢可匹配對象合併到參考中
            if (matchable_reference &&
                merged_reference_matchables.count(matchable_reference) == 0) {
              assert(matchable_query->objects.size() == 1);

              // ds 簿記可匹配對象以便合併
              _merged_matchables.emplace_back(MatchableMerge(
                matchable_query, std::move(matchable_query->_object), matchable_reference));
              merged_reference_matchables.insert(matchable_reference);
            } else {
#endif
              // ds 簿記可匹配對象以便添加
              _trainables[index_trainable].node      = node_current;
              _trainables[index_trainable].matchable = matchable_query;
              ++index_trainable;
#ifdef SRRG_MERGE_DESCRIPTORS
            }
#endif

            // ds 無論是否合併，葉子都需要更新
            ++node_current->_header.number_of_matchables_uncompressed;
            leafs_to_update.insert(node_current);
            break;
          }
        }
      }
      _trainables.resize(index_trainable);
#ifdef SRRG_MERGE_DESCRIPTORS

      // ds 合併可匹配對象
      for (MatchableMerge& mergable : _merged_matchables) {
        assert(mergable.reference != mergable.query);

        // ds 執行合併
        mergable.reference->mergeSingle(mergable.query);

        // ds 釋放查詢對象（！）注意樹擁有可匹配對象的所有權
        delete mergable.query;
      }
      _number_of_merged_matchables_last_training = _merged_matchables.size();
      _merged_matchables.clear();
#endif

      // ds 整合新可匹配對象：合併、添加並根據需要生成葉子
      MatchableVector new_matchables;
      new_matchables.reserve(_trainables.size());
      for (const Trainable& trainable : _trainables) {
        trainable.node->matchables.push_back(trainable.matchable);
        new_matchables.emplace_back(trainable.matchable);
      }
      for (Node* leaf : leafs_to_update) {
        leaf->spawnLeafs(train_mode_);
      }

      // ds 插入新可匹配對象和標識符
      _matchables.insert(_matchables.end(), new_matchables.begin(), new_matchables.end());
      _header.number_of_matchables_compressed += new_matchables.size();
      _added_identifiers_train.insert(identifier_image_query);
      ++_header.number_of_training_entries;
    }

#ifdef SRRG_HBST_HAS_OPENCV

    // ds 從 opencv 描述子創建可匹配對象向量（指針）- 僅在構建系統中存在 OpenCV 時可用
    static const MatchableVector getMatchables(const cv::Mat& descriptors_cv_,
                                               const std::vector<ObjectType>& objects_,
                                               const uint64_t& identifier_tree_ = 0) {
      MatchableVector matchables(descriptors_cv_.rows);

      // ds 拷貝原始數據
      for (int64_t index_descriptor = 0; index_descriptor < descriptors_cv_.rows;
           ++index_descriptor) {
        matchables[index_descriptor] = new Matchable(
          objects_[index_descriptor], descriptors_cv_.row(index_descriptor), identifier_tree_);
      }
      return matchables;
    }

#endif

    //! @brief 清除整個結構（對應於空構造）
    void clear(const bool& delete_matchables_ = true) {
      // ds 數據庫標識符不會被重置

      // ds 清理內部簿記
      _added_identifiers_train.clear();
      _trainables.clear();
      _header.number_of_matchables_uncompressed = 0;
      _header.number_of_matchables_compressed   = 0;
      _header.number_of_training_entries        = 0;
#ifdef SRRG_MERGE_DESCRIPTORS
      _number_of_merged_matchables_last_training = 0;
#endif

      // ds 遞歸刪除所有節點
      delete _root;
      _root = nullptr;

      // ds 根據所有權決定
      if (delete_matchables_) {
        deleteMatchables();
      }
      _matchables.clear();
      _matchables_to_train.clear();
    }

    //! @brief 釋放樹中包含的所有可匹配對象（析構函數）
    void deleteMatchables() {
      for (const Matchable* matchable : _matchables) {
        delete matchable;
      }
      for (const Matchable* matchable : _matchables_to_train) {
        delete matchable;
      }
    }

    //! ds 將完整數據庫保存到磁盤
    bool write(const std::string& file_path) const {
      // ds 打開文件（覆蓋已有文件）
      std::ofstream outfile(file_path, std::ios::binary | std::ios::out);
      if (!outfile.is_open()) {
        std::cerr << "BinaryTree::write|ERROR: unable to open file: " << file_path << std::endl;
        return false;
      }
      if (!outfile.good()) {
        std::cerr << "BinaryTree::write|ERROR: file is not valid: " << file_path << std::endl;
        return false;
      }

      // ds 遍歷完整樹以獲取葉子信息
      uint64_t number_of_leafs      = 0;
      uint64_t number_of_matchables = 0;
      std::vector<const Node*> leafs;
      _getLeafs(_root, number_of_leafs, number_of_matchables, leafs);
      assert(number_of_matchables == _matchables.size());

      // ds 設置字節序標誌位元組 - 讀取時將檢查其值是否為零
      const char endianness_check[] = {char(0)};
      GUARDED_IO(outfile, write, endianness_check, 1, "BinaryTree::write|ERROR: unable to write");

      // ds 在頭部填充葉子計數（目前僅用於序列化/反序列化）
      _header.number_of_leafs = number_of_leafs;
      GUARDED_IO(outfile,
                 write,
                 reinterpret_cast<const char*>(&_header),
                 sizeof(_header),
                 "BinaryTree::write|ERROR: unable to write database header");
      for (const uint64_t identifier : _added_identifiers_train) {
        GUARDED_IO(outfile,
                   write,
                   reinterpret_cast<const char*>(&identifier),
                   sizeof(identifier),
                   "BinaryTree::write|ERROR: unable to write identifiers train");
      }

      // ds 寫入葉子信息
      for (const Node* leaf : leafs) {
        assert(leaf->_header.depth > 0);
        assert(leaf->index_split_bit == -1);
        assert(leaf->has_leafs == false);
#ifndef SRRG_MERGE_DESCRIPTORS
        assert(leaf->_header.number_of_matchables_uncompressed ==
               leaf->_header.number_of_matchables_compressed);
#endif
        GUARDED_IO(outfile,
                   write,
                   reinterpret_cast<const char*>(&leaf->_header),
                   sizeof(leaf->_header),
                   "BinaryTree::write|ERROR: unable to write leaf header");

        // ds 存儲分裂位索引順序
        const Node* current = leaf;
        std::vector<int32_t> indices_split_bit;
        indices_split_bit.reserve(leaf->_header.depth);
        while (current) {
          indices_split_bit.emplace_back(current->index_split_bit);
          current = current->parent;
        }
        for (auto it = indices_split_bit.rbegin(); it != indices_split_bit.rend(); ++it) {
          GUARDED_IO(outfile,
                     write,
                     reinterpret_cast<const char*>(&*it),
                     sizeof(int32_t),
                     "BinaryTree::write|ERROR: unable to write bit index order");
        }

        // ds 序列化可匹配對象（描述子）數據
        assert(leaf->_header.number_of_matchables_uncompressed >= leaf->matchables.size());
        for (const Matchable* matchable : leaf->matchables) {
          GUARDED_IO(outfile,
                     write,
                     reinterpret_cast<const char*>(&matchable->descriptor),
                     Matchable::raw_descriptor_size_bytes,
                     "BinaryTree::write|ERROR: unable to write descriptor data");
          GUARDED_IO(outfile,
                     write,
                     reinterpret_cast<const char*>(&matchable->number_of_objects),
                     sizeof(uint64_t),
                     "BinaryTree::write|ERROR: unable to write number of objects");
          assert(matchable->number_of_objects == matchable->objects.size());
          for (const ObjectMapElement& element : matchable->objects) {
            GUARDED_IO(outfile,
                       write,
                       reinterpret_cast<const char*>(&element.first),
                       sizeof(uint64_t),
                       "BinaryTree::write|ERROR: unable to write object key");
            GUARDED_IO(outfile,
                       write,
                       reinterpret_cast<const char*>(&element.second),
                       sizeof(ObjectType),
                       "BinaryTree::write|ERROR: unable to write object data");
          }
        }
      }
      outfile.close();
      return true;
    }

    //! ds 從磁盤加載數據庫
    bool read(const std::string& file_path) {
      // ds 打開文件以進行讀取
      std::ifstream infile(file_path, std::ios::binary);
      if (!infile.is_open()) {
        std::cerr << "BinaryTree::read|ERROR: unable to open file: " << file_path << std::endl;
        return false;
      }
      if (!infile.good()) {
        std::cerr << "BinaryTree::read|ERROR: file is not valid: " << file_path << std::endl;
        return false;
      }

      // ds 檢查字節序一致性
      char endianness_check[] = {char(1)};
      GUARDED_IO(
        infile, read, endianness_check, 1, "BinaryTree::read|ERROR: unable to read endian byte");
      if (endianness_check[0] != char(0)) {
        std::cerr << "BinaryTree::read|ERROR: invalid endianness, database saved on different arch"
                  << std::endl;
        infile.close();
        return false;
      }

      // ds 讀取數據庫頭部
      GUARDED_IO(infile, read, reinterpret_cast<char*>(&_header), sizeof(_header), "");
      for (size_t i = 0; i < _header.number_of_training_entries; ++i) {
        uint64_t identifier = 0;
        GUARDED_IO(infile, read, reinterpret_cast<char*>(&identifier), sizeof(identifier), "");
        _added_identifiers_train.insert(identifier);
      }
      assert(_added_identifiers_train.size() == _header.number_of_training_entries);

      // ds 葉子緩衝區（靜態以允許輕鬆退出而無需釋放）
      // ds TODO 使用更緊湊的數據結構
      std::vector<typename Node::Header> leaf_headers;
      std::vector<std::vector<Descriptor>> descriptors_per_leaf;
      std::vector<std::vector<ObjectMap>> objects_per_descriptor_per_leaf;
      std::vector<std::vector<int32_t>> bit_indexes_per_leaf;
      leaf_headers.reserve(_header.number_of_leafs);
      descriptors_per_leaf.reserve(_header.number_of_leafs);
      bit_indexes_per_leaf.reserve(_header.number_of_leafs);

      // ds 讀取帶有可匹配對象數據的葉子
      size_t number_of_read_matchables = 0;
      for (size_t i = 0; i < _header.number_of_leafs; ++i) {
        typename Node::Header leaf_header;
        GUARDED_IO(infile,
                   read,
                   reinterpret_cast<char*>(&leaf_header),
                   sizeof(leaf_header),
                   "BinaryTree::read|ERROR: unable to read Node header");
        assert(leaf_header.depth > 0);
        assert(leaf_header.number_of_matchables_uncompressed > 0);
#ifdef SRRG_MERGE_DESCRIPTORS
        assert(leaf_header.number_of_matchables_compressed <=
               leaf_header.number_of_matchables_uncompressed);
#else
        assert(leaf_header.number_of_matchables_uncompressed ==
               leaf_header.number_of_matchables_compressed);
#endif

        // ds 讀取分裂位索引順序 - 注意還需要讀取葉子的 -1
        std::vector<int32_t> indices_split_bit(leaf_header.depth + 1);
        for (size_t j = 0; j < leaf_header.depth + 1; ++j) {
          GUARDED_IO(infile,
                     read,
                     reinterpret_cast<char*>(&indices_split_bit[j]),
                     sizeof(int32_t),
                     "BinaryTree::read|ERROR: unable to read bit index order");
        }
        assert(indices_split_bit.back() == -1);

        // ds 讀取此葉子的可匹配對象
        std::vector<Descriptor> descriptors;
        std::vector<ObjectMap> objects_per_descriptor;
        descriptors.reserve(leaf_header.number_of_matchables_compressed);
        objects_per_descriptor.reserve(leaf_header.number_of_matchables_compressed);
        for (size_t j = 0; j < leaf_header.number_of_matchables_compressed; ++j) {
          Descriptor descriptor;
          GUARDED_IO(infile,
                     read,
                     reinterpret_cast<char*>(&descriptor),
                     Matchable::raw_descriptor_size_bytes,
                     "BinaryTree::read|ERROR: unable to read Matchable data");
          descriptors.emplace_back(descriptor);
          uint64_t number_of_objects = 0;
          GUARDED_IO(infile,
                     read,
                     reinterpret_cast<char*>(&number_of_objects),
                     sizeof(uint64_t),
                     "BinaryTree::read|ERROR: unable to read number of objects");
          ObjectMap objects;
          for (size_t index_object = 0; index_object < number_of_objects; ++index_object) {
            uint64_t key = 0;
            ObjectType object;
            GUARDED_IO(infile,
                       read,
                       reinterpret_cast<char*>(&key),
                       sizeof(uint64_t),
                       "BinaryTree::read|ERROR: unable to read object key");
            GUARDED_IO(infile,
                       read,
                       reinterpret_cast<char*>(&object),
                       sizeof(ObjectType),
                       "BinaryTree::read|ERROR: unable to read object");
            objects.insert(std::make_pair(key, object));
          }
          objects_per_descriptor.emplace_back(objects);
        }
        number_of_read_matchables += descriptors.size();
        leaf_headers.emplace_back(leaf_header);
        descriptors_per_leaf.emplace_back(descriptors);
        objects_per_descriptor_per_leaf.emplace_back(objects_per_descriptor);
        bit_indexes_per_leaf.emplace_back(indices_split_bit);
      }
      infile.close();

      // ds 一致性檢查
      if (number_of_read_matchables != _header.number_of_matchables_compressed) {
        std::cerr << "BinaryTree::read|ERROR: number of loaded matchables inconsistent with header"
                  << std::endl;
        return false;
      }

      // ds 此後使用動態內存構建樹 - 不會拋出異常！
      // ds 通過評估所有葉子來組裝實際數據庫
      _root = new Node();
      assert(leaf_headers.size() == bit_indexes_per_leaf.size());
      for (size_t i = 0; i < leaf_headers.size(); ++i) {
        const typename Node::Header& leaf_header             = leaf_headers[i];
        const std::vector<Descriptor>& descriptors           = descriptors_per_leaf[i];
        const std::vector<int32_t>& bit_index_order          = bit_indexes_per_leaf[i];
        const std::vector<ObjectMap>& objects_per_descriptor = objects_per_descriptor_per_leaf[i];

        // ds 取任意一個描述子用於判定決策規則
        // ds 這樣做是安全的，因為所有描述子都位於同一葉子中且滿足該規則
        const Descriptor& descriptor_sample = descriptors.back();

        // ds 從根節點開始處理每個葉子
        Node* current = _root;
        while (current) {
          // ds 若到達葉子則終止
          if (bit_index_order[current->_header.depth] == -1) {
            assert(current->_header.depth == leaf_header.depth);
            assert(descriptors.size() == leaf_header.number_of_matchables_compressed);

            // ds 填充葉子的可匹配對象
            assert(current->matchables.empty());
            current->matchables.reserve(descriptors.size());
            for (size_t index_descriptor = 0; index_descriptor < descriptors.size();
                 ++index_descriptor) {
              current->matchables.emplace_back(new Matchable(
                objects_per_descriptor[index_descriptor], descriptors[index_descriptor]));
            }
            current->_header = std::move(leaf_header);
            _matchables.insert(
              _matchables.end(), current->matchables.begin(), current->matchables.end());
            break;
          } else {
            // ds 否則始終為中間節點
            current->has_leafs = true;
          }

          // ds 若此節點尚未初始化
          if (current->index_split_bit != bit_index_order[current->_header.depth]) {
            current->index_split_bit                    = bit_index_order[current->_header.depth];
            current->bit_mask[current->index_split_bit] = 0;
          }

          // ds 若必要則生成葉子（我們有一個完整的樹）
          if (!current->right) {
            current->right                = new Node();
            current->right->_header.depth = current->_header.depth + 1;
            current->right->parent        = current;
          }
          if (!current->left) {
            current->left                = new Node();
            current->left->_header.depth = current->_header.depth + 1;
            current->left->parent        = current;
          }

          // ds 遍歷樹
          if (descriptor_sample[current->index_split_bit]) {
            current = current->right;
          } else {
            current = current->left;
          }
        }
      }

      // ds 一致性檢查
      if (_matchables.size() != _header.number_of_matchables_compressed) {
        std::cerr << "BinaryTree::read|ERROR: unable to reconstruct tree with read matchables "
                     "(check HBST version used to generate database)"
                  << std::endl;
        return false;
      }
      return true;
    }

    // ds 輔助函數
  protected:
#ifdef SRRG_MERGE_DESCRIPTORS
    // 快速无异常、无堆内存分配的 _matchExhaustive 向量化重载版本
    void _matchExhaustive(const Matchable* matchable_query_,
                          const MatchableVector& matchables_reference_,
                          const uint32_t& maximum_distance_matching_,
                          MatchMap& best_matches_) const {
      ObjectType object_query = matchable_query_->objects.at(matchable_query_->_image_identifier);

      for (const Matchable* matchable_reference : matchables_reference_) {
        const uint32_t distance = matchable_query_->distance(matchable_reference);

        if (distance < maximum_distance_matching_) {
          for (const ObjectMapElement& object : matchable_reference->objects) {
            const uint64_t& identifer_tree_reference = object.first;

            // 线性搜索，由于叶子节点中对应的KeyFrame极少（通常<=5个），线性搜索性能远胜std::map且无内存分配
            auto it = best_matches_.end();
            for (auto i = best_matches_.begin(); i != best_matches_.end(); ++i) {
              if (i->first == identifer_tree_reference) {
                it = i;
                break;
              }
            }

            if (it != best_matches_.end()) {
              Match& best_match_so_far = it->second;
              const real_type& best_distance_so_far = best_match_so_far.distance;
              if (distance < best_distance_so_far) {
                best_match_so_far.matchable_references = std::vector<const Matchable*>(1, matchable_reference);
                best_match_so_far.object_references = std::vector<ObjectType>(1, object.second);
                best_match_so_far.distance = distance;
              } else if (distance == best_distance_so_far) {
                best_match_so_far.matchable_references.push_back(matchable_reference);
                best_match_so_far.object_references.push_back(object.second);
              }
            } else {
              best_matches_.push_back(std::make_pair(
                identifer_tree_reference,
                Match(matchable_query_, matchable_reference, object_query, object.second, distance)));
            }
          }
        }
      }
    }

    void _matchExhaustive(const Matchable* matchable_query_,
                          const MatchableVector& matchables_reference_,
                          const uint32_t& maximum_distance_matching_,
                          MatchMap& best_matches_,
                          Matchable*& matchable_reference_for_merge_) const {
      ObjectType object_query = matchable_query_->objects.at(matchable_query_->_image_identifier);

      for (const Matchable* matchable_reference : matchables_reference_) {
        const uint32_t distance = matchable_query_->distance(matchable_reference);

        if (distance < maximum_distance_matching_) {
          for (const ObjectMapElement& object : matchable_reference->objects) {
            const uint64_t& identifer_tree_reference = object.first;

            auto it = best_matches_.end();
            for (auto i = best_matches_.begin(); i != best_matches_.end(); ++i) {
              if (i->first == identifer_tree_reference) {
                it = i;
                break;
              }
            }

            if (it != best_matches_.end()) {
              Match& best_match_so_far = it->second;
              const real_type& best_distance_so_far = best_match_so_far.distance;
              if (distance < best_distance_so_far) {
                best_match_so_far.matchable_references = std::vector<const Matchable*>(1, matchable_reference);
                best_match_so_far.object_references = std::vector<ObjectType>(1, object.second);
                best_match_so_far.distance = distance;
              } else if (distance == best_distance_so_far) {
                best_match_so_far.matchable_references.push_back(matchable_reference);
                best_match_so_far.object_references.push_back(object.second);
              }
            } else {
              best_matches_.push_back(std::make_pair(
                identifer_tree_reference,
                Match(matchable_query_, matchable_reference, object_query, object.second, distance)));
            }
          }

          if (distance <= maximum_distance_for_merge) {
            matchable_reference_for_merge_ = const_cast<Matchable*>(matchable_reference);
          }
        }
      }
    }
#else
    // 快速无异常、无堆内存分配的 _matchExhaustive 向量化非Merge重载版本
    void _matchExhaustive(const Matchable* matchable_query_,
                          const MatchableVector& matchables_reference_,
                          const uint32_t& maximum_distance_matching_,
                          MatchMap& best_matches_) const {
      ObjectType object_query = matchable_query_->objects.at(matchable_query_->_image_identifier);

      for (const Matchable* matchable_reference : matchables_reference_) {
        const uint32_t distance = matchable_query_->distance(matchable_reference);

        if (distance < maximum_distance_matching_) {
          const uint64_t& identifer_tree_reference = matchable_reference->_image_identifier;
          ObjectType object_reference = matchable_reference->objects.at(identifer_tree_reference);

          auto it = best_matches_.end();
          for (auto i = best_matches_.begin(); i != best_matches_.end(); ++i) {
            if (i->first == identifer_tree_reference) {
              it = i;
              break;
            }
          }

          if (it != best_matches_.end()) {
            Match& best_match_so_far = it->second;
            const real_type& best_distance_so_far = best_match_so_far.distance;
            if (distance < best_distance_so_far) {
              best_match_so_far.matchable_references = std::vector<const Matchable*>(1, matchable_reference);
              best_match_so_far.object_references = std::vector<ObjectType>(1, object_reference);
              best_match_so_far.distance = distance;
            } else if (distance == best_distance_so_far) {
              best_match_so_far.matchable_references.push_back(matchable_reference);
              best_match_so_far.object_references.push_back(object_reference);
            }
          } else {
            best_matches_.push_back(std::make_pair(
              identifer_tree_reference,
              Match(matchable_query_, matchable_reference, object_query, object_reference, distance)));
          }
        }
      }
    }
#endif

    //! @brief 遞歸統計樹中存儲的所有葉子和描述子（開銷較大）
    //! @param[in] node_ 起始節點（僅評估子樹）
    //! @param[out] number_of_leafs_ 葉子數量
    //! @param[out] number_of_matchables_ 可匹配對象數量
    //! @param[out] leafs_ 葉子列表
    void _getLeafs(Node* node_,
                   uint64_t& number_of_leafs_,
                   uint64_t& number_of_matchables_,
                   std::vector<const Node*>& leafs_) const {
      // ds 處理 node_ == _root 為空樹的情況
      if (_root == nullptr) {
        number_of_leafs_      = 0;
        number_of_matchables_ = 0;
        leafs_.clear();
        return;
      }

      // ds 若處於有葉子的節點中
      if (node_->has_leafs) {
        assert(node_->right);
        assert(node_->left);

        // ds 深入查找左右子樹
        _getLeafs(node_->left, number_of_leafs_, number_of_matchables_, leafs_);
        _getLeafs(node_->right, number_of_leafs_, number_of_matchables_, leafs_);

      } else {
        // ds 到達葉子
        assert(!node_->right);
        assert(!node_->left);

        // ds 更新統計信息並終止遞歸
        ++number_of_leafs_;
        number_of_matchables_ += node_->matchables.size();
        leafs_.push_back(node_);
      }
    }

    // ds 公有屬性
  public:
#ifdef SRRG_MERGE_DESCRIPTORS
    //! @brief 合併兩個描述子所允許的最大描述子距離
    static uint32_t maximum_distance_for_merge;
#endif

    // ds 屬性
  protected:
    //! @brief 可序列化的頭部，攜帶核心屬性
    mutable Header _header;

    //! @brief 根節點（例如相似性搜索的起點）
    Node* _root = nullptr;

    //! @brief 簿記：樹中包含的所有可匹配對象
    MatchableVector _matchables;
    MatchableVector _matchables_to_train;

    //! @brief 簿記：已整合的可匹配對象訓練標識符（唯一）
    std::set<uint64_t> _added_identifiers_train;

    //! @brief 簿記：上次 matchAndAdd 調用後的可訓練可匹配對象
    std::vector<Trainable> _trainables;

#ifdef SRRG_MERGE_DESCRIPTORS
    //! @brief 簿記：上次 matchAndAdd 調用產生的合併可匹配對象對（查詢 -> 參考）
    //! 通過查詢可匹配對象可以訪問已合併（=已釋放）的可匹配對象，
    //! 並相應更新外部簿記
    MatchableMergeVector _merged_matchables;

    //! 統計信息
    size_t _number_of_merged_matchables_last_training = 0;
#endif
  };

// ds 默認配置
#ifdef SRRG_MERGE_DESCRIPTORS
  template <typename BinaryNodeType_>
  uint32_t BinaryTree<BinaryNodeType_>::maximum_distance_for_merge = 0;
#endif

  template <typename ObjectType_>
  using BinaryTree128 = BinaryTree<BinaryNode128<ObjectType_>>;
  template <typename ObjectType_>
  using BinaryTree256 = BinaryTree<BinaryNode256<ObjectType_>>;
  template <typename ObjectType_>
  using BinaryTree512 = BinaryTree<BinaryNode512<ObjectType_>>;

} // namespace srrg_hbst

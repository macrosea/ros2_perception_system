#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>

namespace perception_system {

template <typename MatchT, typename ItemA, typename ItemB>
class MatchQueue {
 public:
  using MatchPtr = std::shared_ptr<MatchT>;
  using ItemAPtr = std::shared_ptr<const ItemA>;
  using ItemBPtr = std::shared_ptr<const ItemB>;

  bool PushA(uint64_t seq, ItemAPtr item) {
    std::lock_guard<std::mutex> lock(mutex_);
    cache_a_[seq] = std::move(item);
    TryMatch(seq);
    Shrink(cache_a_);
    return has_new_;
  }

  bool PushB(uint64_t seq, ItemBPtr item) {
    std::lock_guard<std::mutex> lock(mutex_);
    cache_b_[seq] = std::move(item);
    TryMatch(seq);
    Shrink(cache_b_);
    return has_new_;
  }

  MatchPtr Pop() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_new_) return nullptr;
    has_new_ = false;
    if (latest_match_seq_ > 0) {
      CleanupCache(latest_match_seq_);
      latest_match_seq_ = 0;
    }
    return std::move(latest_match_);
  }

  bool HasNew() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return has_new_;
  }

  void Clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    cache_a_.clear();
    cache_b_.clear();
    latest_match_.reset();
    latest_match_seq_ = 0;
    has_new_ = false;
  }

  void SetMaxCacheSize(int size) {
    max_cache_size_ = size;
  }

 private:
  void TryMatch(uint64_t seq) {
    auto it_a = cache_a_.find(seq);
    auto it_b = cache_b_.find(seq);
    if (it_a != cache_a_.end() && it_b != cache_b_.end()) {
      latest_match_ = std::make_shared<MatchT>(it_a->second, it_b->second);
      latest_match_seq_ = seq;
      has_new_ = true;
    }
  }

  template <typename C>
  void Shrink(C& c) {
    while (static_cast<int>(c.size()) > max_cache_size_) {
      c.erase(c.begin());
    }
  }

  void CleanupCache(uint64_t up_to) {
    auto it_a = cache_a_.begin();
    while (it_a != cache_a_.end() && it_a->first <= up_to) {
      it_a = cache_a_.erase(it_a);
    }
    auto it_b = cache_b_.begin();
    while (it_b != cache_b_.end() && it_b->first <= up_to) {
      it_b = cache_b_.erase(it_b);
    }
  }

  std::map<uint64_t, ItemAPtr> cache_a_;
  std::map<uint64_t, ItemBPtr> cache_b_;
  MatchPtr latest_match_;
  uint64_t latest_match_seq_{0};
  bool has_new_{false};
  int max_cache_size_{100};
  mutable std::mutex mutex_;
};

}  // namespace perception_system

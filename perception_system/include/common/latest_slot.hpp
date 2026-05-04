#pragma once

#include <memory>
#include <mutex>

namespace perception_system {

template <typename T>
class LatestSlot {
 public:
  bool Store(std::shared_ptr<const T> msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool overwritten = has_new_;
    latest_ = std::const_pointer_cast<T>(std::move(msg));
    has_new_ = true;
    return overwritten;
  }

  std::shared_ptr<T> TakeLatest(bool& had_new) {
    std::lock_guard<std::mutex> lock(mutex_);
    had_new = has_new_;
    has_new_ = false;
    return latest_;
  }

  bool HasNew() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return has_new_;
  }

 private:
  mutable std::mutex mutex_;
  std::shared_ptr<T> latest_;
  bool has_new_{false};
};

}  // namespace perception_system

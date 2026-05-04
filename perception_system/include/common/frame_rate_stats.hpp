#pragma once

#include <chrono>
#include <cstdint>

namespace perception_system {

class FrameRateStats {
 public:
  using Clock = std::chrono::steady_clock;

  void reset() {
    frame_count_ = 0;
    window_start_ = Clock::now();
  }

  void count_frame() {
    if (!initialized_) {
      reset();
      initialized_ = true;
    }
    ++frame_count_;
  }

  bool has_samples() const {
    return initialized_ && frame_count_ > 0;
  }

  uint64_t frame_count() const {
    return frame_count_;
  }

  double fps() const {
    if (!has_samples()) {
      return 0.0;
    }

    const auto elapsed = std::chrono::duration<double>(Clock::now() - window_start_).count();
    if (elapsed <= 0.0) {
      return 0.0;
    }
    return static_cast<double>(frame_count_) / elapsed;
  }

 private:
  Clock::time_point window_start_{Clock::now()};
  uint64_t frame_count_{0};
  bool initialized_{false};
};

}  // namespace perception_system

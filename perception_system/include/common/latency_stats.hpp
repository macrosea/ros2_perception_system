#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "common/trace.hpp"
#include "common/xlogger.hpp"

namespace perception_system {

template <size_t N = 256>
class LatencyWindow {
 public:
  void add(uint64_t ns) {
    buf_[idx_++ % N] = ns;
    if (count_ < N) ++count_;
  }

  uint64_t percentile(double p) const {
    if (count_ == 0) return 0;
    std::array<uint64_t, N> tmp{};
    std::copy(buf_.begin(), buf_.begin() + count_, tmp.begin());
    std::sort(tmp.begin(), tmp.begin() + count_);
    const size_t idx = static_cast<size_t>(p * (count_ - 1));
    return tmp[idx];
  }

  uint64_t min() const {
    if (count_ == 0) return 0;
    return *std::min_element(buf_.begin(), buf_.begin() + count_);
  }

  uint64_t max() const {
    if (count_ == 0) return 0;
    return *std::max_element(buf_.begin(), buf_.begin() + count_);
  }

  size_t count() const {
    return count_;
  }

  void reset() {
    idx_ = 0;
    count_ = 0;
  }

 private:
  std::array<uint64_t, N> buf_{};
  size_t idx_{0};
  size_t count_{0};
};

class LatencyStats {
 public:
  void record(const TraceStamp& ts) {
    e2e_.add(ts.e2e_ns());
    proc_.add(ts.proc_lat_ns());
    det_.add(ts.det_lat_ns());
    transfer_.add(ts.transfer_ns());
    ++frame_out_;
  }

  void count_overwrite_drop() {
    ++overwrite_drop_;
  }

  void count_freshness_drop() {
    ++freshness_drop_;
  }

  void count_in() {
    ++frame_in_;
  }

  void log() const {
    if (e2e_.count() == 0) return;

    const uint64_t total_drop = overwrite_drop_ + freshness_drop_;
    const double drop_rate = frame_in_ > 0 ? static_cast<double>(total_drop) / frame_in_ : 0.0;

    LOG_INFO(
        "[LatencyStats] in=%lu out=%lu overwrite_drop=%lu freshness_drop=%lu drop=%.1f%%\n"
        "  E2E      p50=%5.2f  p95=%5.2f  p99=%5.2f  max=%5.2f ms\n"
        "  ImageProc p50=%5.2f  p95=%5.2f  p99=%5.2f ms\n"
        "  Detector  p50=%5.2f  p95=%5.2f  p99=%5.2f ms\n"
        "  Transfer  p50=%5.2f  p95=%5.2f ms",
        frame_in_,
        frame_out_,
        overwrite_drop_,
        freshness_drop_,
        drop_rate * 100.0,
        ms(e2e_.percentile(0.50)),
        ms(e2e_.percentile(0.95)),
        ms(e2e_.percentile(0.99)),
        ms(e2e_.max()),
        ms(proc_.percentile(0.50)),
        ms(proc_.percentile(0.95)),
        ms(proc_.percentile(0.99)),
        ms(det_.percentile(0.50)),
        ms(det_.percentile(0.95)),
        ms(det_.percentile(0.99)),
        ms(transfer_.percentile(0.50)),
        ms(transfer_.percentile(0.95)));
  }

  void reset() {
    e2e_.reset();
    proc_.reset();
    det_.reset();
    transfer_.reset();
    frame_in_ = 0;
    frame_out_ = 0;
    overwrite_drop_ = 0;
    freshness_drop_ = 0;
  }

  const LatencyWindow<>& e2e() const {
    return e2e_;
  }

  const LatencyWindow<>& proc() const {
    return proc_;
  }

  const LatencyWindow<>& det() const {
    return det_;
  }

  const LatencyWindow<>& transfer() const {
    return transfer_;
  }

  uint64_t frame_in() const {
    return frame_in_;
  }

  uint64_t frame_out() const {
    return frame_out_;
  }

  uint64_t overwrite_drop() const {
    return overwrite_drop_;
  }

  uint64_t freshness_drop() const {
    return freshness_drop_;
  }

 private:
  static double ms(uint64_t ns) {
    return static_cast<double>(ns) / 1e6;
  }

  LatencyWindow<> e2e_;
  LatencyWindow<> proc_;
  LatencyWindow<> det_;
  LatencyWindow<> transfer_;

  uint64_t frame_in_{0};
  uint64_t frame_out_{0};
  uint64_t overwrite_drop_{0};
  uint64_t freshness_drop_{0};
};

}  // namespace perception_system

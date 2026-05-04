#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <cstdint>
#include <ctime>

#include "perception_system/msg/trace_stamp.hpp"

namespace perception_system {

inline uint64_t now_ns() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ull + static_cast<uint64_t>(ts.tv_nsec);
}

inline uint64_t StampToNs(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<uint64_t>(stamp.sec) * 1'000'000'000ull + static_cast<uint64_t>(stamp.nanosec);
}

inline builtin_interfaces::msg::Time NsToStamp(uint64_t ns) {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(ns / 1'000'000'000ull);
  stamp.nanosec = static_cast<uint32_t>(ns % 1'000'000'000ull);
  return stamp;
}

struct TraceStamp {
  uint64_t t_cam_pub{0};

  uint64_t t_proc_in{0};
  uint64_t t_proc_out{0};

  uint64_t t_det_in{0};
  uint64_t t_det_out{0};

  msg::TraceStamp ToMsg() const {
    msg::TraceStamp out;
    out.t_cam_pub = t_cam_pub;
    out.t_proc_in = t_proc_in;
    out.t_proc_out = t_proc_out;
    out.t_det_in = t_det_in;
    out.t_det_out = t_det_out;
    return out;
  }

  static TraceStamp FromMsg(const msg::TraceStamp& in) {
    TraceStamp out;
    out.t_cam_pub = in.t_cam_pub;
    out.t_proc_in = in.t_proc_in;
    out.t_proc_out = in.t_proc_out;
    out.t_det_in = in.t_det_in;
    out.t_det_out = in.t_det_out;
    return out;
  }

  uint64_t e2e_ns() const {
    return t_det_out >= t_cam_pub ? t_det_out - t_cam_pub : 0;
  }

  uint64_t proc_lat_ns() const {
    return t_proc_out >= t_proc_in ? t_proc_out - t_proc_in : 0;
  }

  uint64_t det_lat_ns() const {
    return t_det_out >= t_det_in ? t_det_out - t_det_in : 0;
  }

  uint64_t transfer_ns() const {
    return t_det_in >= t_proc_out ? t_det_in - t_proc_out : 0;
  }
};

}  // namespace perception_system

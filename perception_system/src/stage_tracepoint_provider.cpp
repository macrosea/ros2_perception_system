#define TRACEPOINT_CREATE_PROBES
#define TRACEPOINT_DEFINE

#include "perception_system/stage_tracepoint_tp.hpp"

extern "C" {

void ros_trace_perception_system_image_proc_start(uint64_t frame_seq, uint64_t stamp_ns) {
  tracepoint(perception_system, image_proc_start, frame_seq, stamp_ns);
}

bool ros_trace_enabled_perception_system_image_proc_start(void) {
  return tracepoint_enabled(perception_system, image_proc_start);
}

void ros_trace_do_perception_system_image_proc_start(uint64_t frame_seq, uint64_t stamp_ns) {
  tracepoint(perception_system, image_proc_start, frame_seq, stamp_ns);
}

void ros_trace_perception_system_image_proc_end(uint64_t frame_seq, uint64_t stamp_ns) {
  tracepoint(perception_system, image_proc_end, frame_seq, stamp_ns);
}

bool ros_trace_enabled_perception_system_image_proc_end(void) {
  return tracepoint_enabled(perception_system, image_proc_end);
}

void ros_trace_do_perception_system_image_proc_end(uint64_t frame_seq, uint64_t stamp_ns) {
  tracepoint(perception_system, image_proc_end, frame_seq, stamp_ns);
}

void ros_trace_perception_system_detector_start(uint64_t frame_seq, uint64_t stamp_ns) {
  tracepoint(perception_system, detector_start, frame_seq, stamp_ns);
}

bool ros_trace_enabled_perception_system_detector_start(void) {
  return tracepoint_enabled(perception_system, detector_start);
}

void ros_trace_do_perception_system_detector_start(uint64_t frame_seq, uint64_t stamp_ns) {
  tracepoint(perception_system, detector_start, frame_seq, stamp_ns);
}

void ros_trace_perception_system_detector_end(uint64_t frame_seq, uint64_t stamp_ns) {
  tracepoint(perception_system, detector_end, frame_seq, stamp_ns);
}

bool ros_trace_enabled_perception_system_detector_end(void) {
  return tracepoint_enabled(perception_system, detector_end);
}

void ros_trace_do_perception_system_detector_end(uint64_t frame_seq, uint64_t stamp_ns) {
  tracepoint(perception_system, detector_end, frame_seq, stamp_ns);
}
}

#pragma once

#include <tracetools/tracetools.h>

#ifdef __cplusplus
extern "C" {
#endif

void ros_trace_perception_system_image_proc_start(uint64_t frame_seq, uint64_t stamp_ns);
bool ros_trace_enabled_perception_system_image_proc_start(void);
void ros_trace_do_perception_system_image_proc_start(uint64_t frame_seq, uint64_t stamp_ns);

void ros_trace_perception_system_image_proc_end(uint64_t frame_seq, uint64_t stamp_ns);
bool ros_trace_enabled_perception_system_image_proc_end(void);
void ros_trace_do_perception_system_image_proc_end(uint64_t frame_seq, uint64_t stamp_ns);

void ros_trace_perception_system_detector_start(uint64_t frame_seq, uint64_t stamp_ns);
bool ros_trace_enabled_perception_system_detector_start(void);
void ros_trace_do_perception_system_detector_start(uint64_t frame_seq, uint64_t stamp_ns);

void ros_trace_perception_system_detector_end(uint64_t frame_seq, uint64_t stamp_ns);
bool ros_trace_enabled_perception_system_detector_end(void);
void ros_trace_do_perception_system_detector_end(uint64_t frame_seq, uint64_t stamp_ns);

#ifdef __cplusplus
}
#endif

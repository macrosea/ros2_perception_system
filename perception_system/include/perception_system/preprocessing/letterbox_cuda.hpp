#pragma once

#include <cstdint>

namespace perception_system {

bool LaunchLetterboxKernel(
    const uint8_t* d_src, float* d_dst, int src_width, int src_height, int dst_size, float scale);

}  // namespace perception_system

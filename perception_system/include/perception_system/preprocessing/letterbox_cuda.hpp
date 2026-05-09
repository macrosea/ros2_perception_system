#pragma once

#include <cuda_runtime.h>

#include <cstdint>

namespace perception_system {

bool LaunchLetterboxKernel(const uint8_t* d_src,
                           float* d_dst,
                           int src_width,
                           int src_height,
                           int src_step,
                           int dst_size,
                           float scale,
                           cudaStream_t stream);

}  // namespace perception_system

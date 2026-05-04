#include "perception_system/preprocessing/letterbox_cuda.hpp"

#include <cuda_runtime.h>

#include <cmath>

namespace perception_system {

namespace {

constexpr float kPaddingValue = 114.0f / 255.0f;

/*
YOLO 风格的 letterbox 预处理：
  1. 按比例缩放输入图像到目标尺寸
  2. 保持宽高比
  3. 用灰色填充（114/255）
  4. BGR -> RGB 转换
  5. 归一化到 [0, 1]
  6. 输出 CHW layout（channel-first）
*/

__global__ void LetterboxKernel(const uint8_t *__restrict__ src,
                                float *__restrict__ dst, int src_width,
                                int src_height, int dst_size, int pad_left,
                                int pad_top, int resized_w, int resized_h,
                                float inv_scale) {
  const int dst_x = blockIdx.x * blockDim.x + threadIdx.x;
  const int dst_y = blockIdx.y * blockDim.y + threadIdx.y;
  if (dst_x >= dst_size || dst_y >= dst_size) {
    return;
  }

  const int out_idx = dst_y * dst_size + dst_x;

  if (dst_x < pad_left || dst_x >= pad_left + resized_w || dst_y < pad_top ||
      dst_y >= pad_top + resized_h) {
    dst[0 * dst_size * dst_size + out_idx] = kPaddingValue;
    dst[1 * dst_size * dst_size + out_idx] = kPaddingValue;
    dst[2 * dst_size * dst_size + out_idx] = kPaddingValue;
    return;
  }
  //  nearest neighbor 插值
  int src_x = static_cast<int>((dst_x - pad_left) * inv_scale);
  int src_y = static_cast<int>((dst_y - pad_top) * inv_scale);
  src_x = min(max(src_x, 0), src_width - 1);
  src_y = min(max(src_y, 0), src_height - 1);

  const int src_idx = (src_y * src_width + src_x) * 3;
  const uint8_t b = src[src_idx];
  const uint8_t g = src[src_idx + 1];
  const uint8_t r = src[src_idx + 2];

  constexpr float kNormFactor = 1.0f / 255.0f;
  dst[0 * dst_size * dst_size + out_idx] = r * kNormFactor;
  dst[1 * dst_size * dst_size + out_idx] = g * kNormFactor;
  dst[2 * dst_size * dst_size + out_idx] = b * kNormFactor;
}

} // namespace

bool LaunchLetterboxKernel(const uint8_t *d_src, float *d_dst, int src_width,
                           int src_height, int dst_size, float scale) {
  const int resized_w = static_cast<int>(std::round(src_width * scale));
  const int resized_h = static_cast<int>(std::round(src_height * scale));
  const int pad_left = (dst_size - resized_w) / 2;
  const int pad_top = (dst_size - resized_h) / 2;
  const float inv_scale = 1.0f / scale;

  const dim3 block(32, 32);
  const dim3 grid((dst_size + block.x - 1) / block.x,
                  (dst_size + block.y - 1) / block.y);

  LetterboxKernel<<<grid, block>>>(d_src, d_dst, src_width, src_height,
                                   dst_size, pad_left, pad_top, resized_w,
                                   resized_h, inv_scale);

  cudaError_t err = cudaGetLastError();
  if (err != cudaSuccess) {
    return false;
  }

  return cudaDeviceSynchronize() == cudaSuccess;
}

} // namespace perception_system

#include "perception_system/preprocessing/preprocessor.hpp"

#include <cuda_runtime.h>
#include <string.h>

#include <algorithm>
#include <cmath>

#include "common/xlogger.hpp"
#include "perception_system/preprocessing/letterbox_cuda.hpp"

namespace perception_system {

struct Preprocessor::Impl {
  Impl(int input_size) : input_size_(input_size) {
    // cudaSetDevice(cuda_device_id);
    cudaMalloc(&d_dst_, input_size_ * input_size_ * 3 * sizeof(float));
  }

  ~Impl() {
    cudaFree(d_src_);
    cudaFree(d_dst_);
  }

  bool Process(const sensor_msgs::msg::Image& msg,
               std::vector<float>& output,
               int& orig_width,
               int& orig_height,
               float& scale) {
    if (msg.encoding != "bgr8") {
      LOG_ERROR("unsupported encoding: %s", msg.encoding.c_str());
      return false;
    }

    orig_width = msg.width;
    orig_height = msg.height;
    scale = std::min(static_cast<float>(input_size_) / orig_width,
                     static_cast<float>(input_size_) / orig_height);

    int resized_w = static_cast<int>(std::round(orig_width * scale));
    int resized_h = static_cast<int>(std::round(orig_height * scale));
    (void)resized_w;
    (void)resized_h;

    const size_t src_bytes = msg.step * msg.height;
    if (msg.step < msg.width * 3 || msg.data.size() < src_bytes) {
      LOG_ERROR("invalid image layout width=%u height=%u step=%u data=%zu",
                msg.width,
                msg.height,
                msg.step,
                msg.data.size());
      return false;
    }

    if (src_bytes > src_capacity_) {
      cudaFree(d_src_);
      if (cudaMalloc(&d_src_, src_bytes) != cudaSuccess) {
        LOG_ERROR("cudaMalloc d_src failed bytes=%zu", src_bytes);
        d_src_ = nullptr;
        src_capacity_ = 0;
        return false;
      }
      src_capacity_ = src_bytes;
    }
    if (cudaMemcpy(d_src_, msg.data.data(), src_bytes, cudaMemcpyHostToDevice) != cudaSuccess) {
      LOG_ERROR("cudaMemcpy H2D failed bytes=%zu", src_bytes);
      return false;
    }

    if (!LaunchLetterboxKernel(d_src_, d_dst_, orig_width, orig_height, input_size_, scale)) {
      LOG_ERROR("LaunchLetterboxKernel failed");
      return false;
    }

    output.resize(input_size_ * input_size_ * 3);
    if (cudaMemcpy(output.data(),
                   d_dst_,
                   input_size_ * input_size_ * 3 * sizeof(float),
                   cudaMemcpyDeviceToHost)
        != cudaSuccess) {
      LOG_ERROR("cudaMemcpy D2H failed bytes=%zu", input_size_ * input_size_ * 3 * sizeof(float));
      return false;
    }
    return true;
  }

  int input_size_;
  uint8_t* d_src_{nullptr};
  float* d_dst_{nullptr};
  size_t src_capacity_{0};
};

Preprocessor::Preprocessor(int input_size) : impl_(std::make_unique<Impl>(input_size)) {
}

Preprocessor::~Preprocessor() = default;

bool Preprocessor::Process(const sensor_msgs::msg::Image& msg,
                           std::vector<float>& output,
                           int& orig_width,
                           int& orig_height,
                           float& scale) {
  return impl_->Process(msg, output, orig_width, orig_height, scale);
}

}  // namespace perception_system

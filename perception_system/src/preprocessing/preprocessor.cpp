#include "perception_system/preprocessing/preprocessor.hpp"

#include <cuda_runtime.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <exception>
#include <limits>
#include <utility>

#include "common/xlogger.hpp"
#include "perception_system/inference/buffers.hpp"
#include "perception_system/preprocessing/letterbox_cuda.hpp"

namespace perception_system {

struct Preprocessor::Impl {
  Impl(int input_size) : input_size_(input_size) {
    // cudaSetDevice(cuda_device_id);

    if (input_size_ <= 0) {
      LOG_ERROR("invalid input_size=%d", input_size_);
      return;
    }

    cudaError_t err = cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking);
    if (err != cudaSuccess) {
      LOG_ERROR("cudaStreamCreateWithFlags failed: %s", cudaGetErrorString(err));
      stream_ = nullptr;
      return;
    }

    const size_t dst_bytes = static_cast<size_t>(input_size_) * input_size_ * 3 * sizeof(float);

    try {
      d_dst_ = DeviceBuffer(dst_bytes);
      h_dst_ = PinnedHostBuffer(dst_bytes);
    } catch (const std::exception& e) {
      LOG_ERROR("failed to allocate preprocess output buffers bytes=%zu: %s", dst_bytes, e.what());
    }
  }

  ~Impl() {
    if (stream_ != nullptr) {
      cudaStreamSynchronize(stream_);
      cudaStreamDestroy(stream_);
    }
  }

  bool Process(const sensor_msgs::msg::Image& msg,
               std::vector<float>& output,
               int& orig_width,
               int& orig_height,
               float& scale) {
    if (stream_ == nullptr || !d_dst_ || !h_dst_) {
      LOG_ERROR("Preprocessor is not initialized");
      return false;
    }

    if (msg.encoding != "bgr8") {
      LOG_ERROR("unsupported encoding: %s", msg.encoding.c_str());
      return false;
    }

    if (msg.width == 0 || msg.height == 0) {
      LOG_ERROR("invalid image size width=%u height=%u", msg.width, msg.height);
      return false;
    }

    orig_width = msg.width;
    orig_height = msg.height;
    scale = std::min(static_cast<float>(input_size_) / orig_width,
                     static_cast<float>(input_size_) / orig_height);

    const size_t min_step = static_cast<size_t>(msg.width) * 3;
    const size_t src_bytes = static_cast<size_t>(msg.step) * msg.height;
    if (msg.step < min_step || msg.data.size() < src_bytes
        || msg.step > static_cast<size_t>(std::numeric_limits<int>::max())) {
      LOG_ERROR("invalid image layout width=%u height=%u step=%u data=%zu",
                msg.width,
                msg.height,
                msg.step,
                msg.data.size());
      return false;
    }

    if (src_bytes > src_capacity_) {
      try {
        DeviceBuffer new_d_src(src_bytes);
        PinnedHostBuffer new_h_src(src_bytes);
        d_src_ = std::move(new_d_src);
        h_src_ = std::move(new_h_src);
      } catch (const std::exception& e) {
        LOG_ERROR("failed to allocate preprocess input buffers bytes=%zu: %s", src_bytes, e.what());
        src_capacity_ = 0;
        return false;
      }
      src_capacity_ = src_bytes;
    }

    std::memcpy(h_src_.Data(), msg.data.data(), src_bytes);
    if (cudaMemcpyAsync(d_src_.Data(), h_src_.Data(), src_bytes, cudaMemcpyHostToDevice, stream_)
        != cudaSuccess) {
      LOG_ERROR("cudaMemcpyAsync H2D failed bytes=%zu", src_bytes);
      return false;
    }

    if (!LaunchLetterboxKernel(static_cast<const uint8_t*>(d_src_.Data()),
                               static_cast<float*>(d_dst_.Data()),
                               orig_width,
                               orig_height,
                               static_cast<int>(msg.step),
                               input_size_,
                               scale,
                               stream_)) {
      LOG_ERROR("LaunchLetterboxKernel failed");
      cudaStreamSynchronize(stream_);
      return false;
    }

    const size_t dst_elements = static_cast<size_t>(input_size_) * input_size_ * 3;
    const size_t dst_bytes = dst_elements * sizeof(float);

    if (cudaMemcpyAsync(h_dst_.Data(), d_dst_.Data(), dst_bytes, cudaMemcpyDeviceToHost, stream_)
        != cudaSuccess) {
      LOG_ERROR("cudaMemcpyAsync D2H failed bytes=%zu", dst_bytes);
      cudaStreamSynchronize(stream_);
      return false;
    }

    const cudaError_t sync_err = cudaStreamSynchronize(stream_);
    if (sync_err != cudaSuccess) {
      LOG_ERROR("cudaStreamSynchronize failed: %s", cudaGetErrorString(sync_err));
      return false;
    }
    output.resize(dst_elements);
    std::memcpy(output.data(), h_dst_.Data(), dst_bytes);
    return true;
  }

  int input_size_;

  DeviceBuffer d_src_;
  DeviceBuffer d_dst_;
  PinnedHostBuffer h_src_;
  PinnedHostBuffer h_dst_;
  size_t src_capacity_{0};
  cudaStream_t stream_{nullptr};
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

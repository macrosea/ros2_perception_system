#pragma once

#include <cuda_runtime.h>

#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>

namespace perception_system {

struct DeviceMemoryPolicy {
  static cudaError_t Allocate(void** ptr, size_t bytes) {
    return cudaMalloc(ptr, bytes);
  }

  static cudaError_t Free(void* ptr) {
    return cudaFree(ptr);
  }

  static const char* Label() {
    return "device memory";
  }
};

struct PinnedHostMemoryPolicy {
  static cudaError_t Allocate(void** ptr, size_t bytes) {
    return cudaMallocHost(ptr, bytes);
  }

  static cudaError_t Free(void* ptr) {
    return cudaFreeHost(ptr);
  }

  static const char* Label() {
    return "pinned host memory";
  }
};

template <typename Policy>
class CudaBuffer {
 public:
  CudaBuffer() = default;

  explicit CudaBuffer(size_t bytes) : bytes_(bytes) {
    if (bytes_ == 0) {
      throw std::runtime_error("CudaBuffer size must be positive");
    }
    if (Policy::Allocate(&ptr_, bytes_) != cudaSuccess) {
      throw std::runtime_error(std::string("Failed to allocate ") + Policy::Label());
    }
  }

  ~CudaBuffer() noexcept {
    Release();
  }

  CudaBuffer(const CudaBuffer&) = delete;
  CudaBuffer& operator=(const CudaBuffer&) = delete;

  CudaBuffer(CudaBuffer&& other) noexcept : ptr_(other.ptr_), bytes_(other.bytes_) {
    other.ptr_ = nullptr;
    other.bytes_ = 0;
  }

  CudaBuffer& operator=(CudaBuffer&& other) noexcept {
    if (this != &other) {
      Release();
      ptr_ = other.ptr_;
      bytes_ = other.bytes_;
      other.ptr_ = nullptr;
      other.bytes_ = 0;
    }
    return *this;
  }

  void* Data() const {
    return ptr_;
  }

  size_t Bytes() const {
    return bytes_;
  }

  explicit operator bool() const {
    return ptr_ != nullptr;
  }

 private:
  void Release() noexcept {
    if (ptr_) {
      Policy::Free(ptr_);
      ptr_ = nullptr;
      bytes_ = 0;
    }
  }

  void* ptr_ = nullptr;
  size_t bytes_ = 0;
};

using DeviceBuffer = CudaBuffer<DeviceMemoryPolicy>;
using PinnedHostBuffer = CudaBuffer<PinnedHostMemoryPolicy>;

}  // namespace perception_system

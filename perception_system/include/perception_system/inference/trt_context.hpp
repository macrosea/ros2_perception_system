#pragma once

#include <NvInfer.h>
#include <cuda_runtime.h>

#include <string>
#include <unordered_map>
#include <vector>

#include "trt_engine.hpp"

namespace perception_system {

class Context {
 public:
  explicit Context(const Engine& engine);
  ~Context();

  Context(const Context&) = delete;
  Context& operator=(const Context&) = delete;

  std::unordered_map<std::string, std::vector<float>> infer(const void* input_data,
                                                            const nvinfer1::Dims& input_shape);

 private:
  struct DeviceBuffer {
    void* ptr = nullptr;
    size_t bytes = 0;
  };

  struct TrtDeleter {
    template <typename T>
    void operator()(T* obj) const {
      delete obj;
    }
  };

  const Engine& engine_;
  std::unique_ptr<nvinfer1::IExecutionContext, TrtDeleter> ctx_;
  cudaStream_t stream_ = nullptr;

  std::unordered_map<std::string, DeviceBuffer> device_bufs_;

  void allocate_buffers(const nvinfer1::Dims& input_shape);
  void free_buffers();
  size_t dims_volume(const nvinfer1::Dims& dims) const;
  size_t dtype_bytes(nvinfer1::DataType dtype) const;
};

}  // namespace perception_system

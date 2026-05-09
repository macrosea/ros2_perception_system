#pragma once

#include <NvInfer.h>
#include <cuda_runtime.h>

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "perception_system/inference/buffers.hpp"
#include "trt_engine.hpp"

namespace perception_system {

class Context {
 public:
  explicit Context(const Engine& engine);
  ~Context();

  Context(const Context&) = delete;
  Context& operator=(const Context&) = delete;
  Context(Context&&) = delete;
  Context& operator=(Context&&) = delete;

  std::unordered_map<std::string, std::vector<float>> Infer(const void* input_data,
                                                            const nvinfer1::Dims& input_shape);

 private:
  struct TensorBuffers {
    DeviceBuffer device;
    PinnedHostBuffer host;
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
  std::unordered_map<std::string, TensorBuffers> tensor_buffers_;
  nvinfer1::Dims cached_input_shape_{};

  void AllocateBuffers(const nvinfer1::Dims& input_shape);
  void FreeBuffers();
  bool SameDims(const nvinfer1::Dims& lhs, const nvinfer1::Dims& rhs) const;
  size_t DimsVolume(const nvinfer1::Dims& dims) const;
  size_t DtypeBytes(nvinfer1::DataType dtype) const;
};

}  // namespace perception_system

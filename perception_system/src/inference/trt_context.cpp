#include "perception_system/inference/trt_context.hpp"

#include <cuda_fp16.h>
#include <cuda_runtime.h>

#include <cstring>
#include <stdexcept>

namespace perception_system {

Context::Context(const Engine& engine) : engine_(engine) {
  ctx_.reset(engine_.raw()->createExecutionContext());
  if (!ctx_) {
    throw std::runtime_error("Failed to create execution context");
  }
  if (cudaStreamCreate(&stream_) != cudaSuccess) {
    throw std::runtime_error("Failed to create CUDA stream");
  }
}

Context::~Context() {
  free_buffers();
  if (stream_) {
    cudaStreamDestroy(stream_);
  }
}

size_t Context::dims_volume(const nvinfer1::Dims& dims) const {
  size_t vol = 1;
  for (int i = 0; i < dims.nbDims; ++i) {
    if (dims.d[i] < 0) {
      throw std::runtime_error("Invalid dimension: negative value");
    }
    vol *= dims.d[i];
  }
  return vol;
}

size_t Context::dtype_bytes(nvinfer1::DataType dtype) const {
  switch (dtype) {
    case nvinfer1::DataType::kFLOAT:
      return 4;
    case nvinfer1::DataType::kHALF:
      return 2;
    case nvinfer1::DataType::kINT8:
      return 1;
    case nvinfer1::DataType::kINT32:
      return 4;
    default:
      throw std::runtime_error("Unsupported data type");
  }
}

void Context::allocate_buffers(const nvinfer1::Dims& input_shape) {
  free_buffers();

  for (const auto& info : engine_.tensors()) {
    nvinfer1::Dims dims = info.dims;
    if (info.is_input) {
      dims = input_shape;
      if (!ctx_->setInputShape(info.name.c_str(), dims)) {
        throw std::runtime_error("Failed to set input shape for " + info.name);
      }
    } else {
      dims = ctx_->getTensorShape(info.name.c_str());
    }

    const size_t vol = dims_volume(dims);
    const size_t bytes = vol * dtype_bytes(info.dtype);

    void* ptr = nullptr;
    if (cudaMalloc(&ptr, bytes) != cudaSuccess) {
      throw std::runtime_error("Failed to allocate device memory for " + info.name);
    }
    device_bufs_[info.name] = {ptr, bytes};
    ctx_->setTensorAddress(info.name.c_str(), ptr);
  }
}

void Context::free_buffers() {
  for (auto& kv : device_bufs_) {
    if (kv.second.ptr) {
      cudaFree(kv.second.ptr);
    }
  }
  device_bufs_.clear();
}

std::unordered_map<std::string, std::vector<float>> Context::infer(
    const void* input_data, const nvinfer1::Dims& input_shape) {
  allocate_buffers(input_shape);

  // 找到输入 tensor
  const TensorInfo* input_info = nullptr;
  for (const auto& t : engine_.tensors()) {
    if (t.is_input) {
      input_info = &t;
      break;
    }
  }
  if (!input_info) {
    throw std::runtime_error("No input tensor found");
  }

  // H2D copy
  const size_t input_bytes = device_bufs_[input_info->name].bytes;
  if (cudaMemcpyAsync(device_bufs_[input_info->name].ptr,
                      input_data,
                      input_bytes,
                      cudaMemcpyHostToDevice,
                      stream_)
      != cudaSuccess) {
    throw std::runtime_error("Failed to copy input to device");
  }

  // 推理
  if (!ctx_->enqueueV3(stream_)) {
    throw std::runtime_error("Inference failed");
  }

  // D2H copy
  std::unordered_map<std::string, std::vector<float>> outputs;
  for (const auto& t : engine_.tensors()) {
    if (!t.is_input) {
      const size_t bytes = device_bufs_[t.name].bytes;
      const size_t count = bytes / dtype_bytes(t.dtype);
      std::vector<float> host_buf(count);

      if (t.dtype == nvinfer1::DataType::kFLOAT) {
        if (cudaMemcpyAsync(host_buf.data(),
                            device_bufs_[t.name].ptr,
                            bytes,
                            cudaMemcpyDeviceToHost,
                            stream_)
            != cudaSuccess) {
          throw std::runtime_error("Failed to copy output from device");
        }
      } else if (t.dtype == nvinfer1::DataType::kHALF) {
        std::vector<uint16_t> half_buf(count);
        if (cudaMemcpyAsync(half_buf.data(),
                            device_bufs_[t.name].ptr,
                            bytes,
                            cudaMemcpyDeviceToHost,
                            stream_)
            != cudaSuccess) {
          throw std::runtime_error("Failed to copy output from device");
        }
        cudaStreamSynchronize(stream_);
        for (size_t i = 0; i < count; ++i) {
          host_buf[i] = __half2float(*reinterpret_cast<__half*>(&half_buf[i]));
        }
      } else {
        throw std::runtime_error("Unsupported output dtype for " + t.name);
      }
      outputs[t.name] = std::move(host_buf);
    }
  }

  cudaStreamSynchronize(stream_);
  return outputs;
}

}  // namespace perception_system

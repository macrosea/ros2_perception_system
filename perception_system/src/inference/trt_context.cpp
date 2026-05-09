#include "perception_system/inference/trt_context.hpp"

#include <cuda_fp16.h>
#include <cuda_runtime.h>

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace perception_system {

namespace {

bool IsValidDims(const nvinfer1::Dims& dims) {
  if (dims.nbDims <= 0) {
    return false;
  }
  for (int i = 0; i < dims.nbDims; ++i) {
    if (dims.d[i] <= 0) {
      return false;
    }
  }
  return true;
}

std::vector<float> ConvertHostOutput(const void* host_data,
                                     nvinfer1::DataType dtype,
                                     size_t count) {
  std::vector<float> host_buf(count);

  if (dtype == nvinfer1::DataType::kFLOAT) {
    const auto* src = static_cast<const float*>(host_data);
    std::copy(src, src + count, host_buf.begin());
  } else if (dtype == nvinfer1::DataType::kHALF) {
    const auto* src = static_cast<const __half*>(host_data);
    float* dst = host_buf.data();
    for (size_t i = 0; i < count; ++i) {
      dst[i] = __half2float(src[i]);
    }
  } else {
    throw std::runtime_error("Unsupported output dtype");
  }

  return host_buf;
}

}  // namespace

Context::Context(const Engine& engine) : engine_(engine) {
  ctx_.reset(engine_.Raw()->createExecutionContext());
  if (!ctx_) {
    throw std::runtime_error("Failed to create execution context");
  }
  if (cudaStreamCreate(&stream_) != cudaSuccess) {
    throw std::runtime_error("Failed to create CUDA stream");
  }
}

Context::~Context() {
  FreeBuffers();
  if (stream_) {
    cudaStreamDestroy(stream_);
  }
}

size_t Context::DimsVolume(const nvinfer1::Dims& dims) const {
  size_t vol = 1;
  for (int i = 0; i < dims.nbDims; ++i) {
    if (dims.d[i] <= 0) {
      throw std::runtime_error("Invalid dimension: non-positive value");
    }

    vol *= static_cast<size_t>(dims.d[i]);
  }
  return vol;
}

bool Context::SameDims(const nvinfer1::Dims& lhs, const nvinfer1::Dims& rhs) const {
  if (lhs.nbDims != rhs.nbDims) {
    return false;
  }
  for (int i = 0; i < lhs.nbDims; ++i) {
    if (lhs.d[i] != rhs.d[i]) {
      return false;
    }
  }
  return true;
}

size_t Context::DtypeBytes(nvinfer1::DataType dtype) const {
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

void Context::AllocateBuffers(const nvinfer1::Dims& input_shape) {
  if (!IsValidDims(input_shape)) {
    throw std::runtime_error("Invalid input shape");
  }
  if (!tensor_buffers_.empty() && SameDims(cached_input_shape_, input_shape)) {
    return;
  }

  FreeBuffers();

  for (const auto& info : engine_.Tensors()) {
    nvinfer1::Dims dims = info.dims;
    if (info.is_input) {
      dims = input_shape;
      if (!ctx_->setInputShape(info.name.c_str(), dims)) {
        throw std::runtime_error("Failed to set input shape for " + info.name);
      }
    } else {
      dims = ctx_->getTensorShape(info.name.c_str());
    }

    if (!IsValidDims(dims)) {
      throw std::runtime_error("Invalid tensor shape for " + info.name);
    }

    const size_t vol = DimsVolume(dims);
    const size_t bytes = vol * DtypeBytes(info.dtype);

    TensorBuffers buffers;
    buffers.device = DeviceBuffer(bytes);
    if (!info.is_input) {
      buffers.host = PinnedHostBuffer(bytes);
    }

    if (!ctx_->setTensorAddress(info.name.c_str(), buffers.device.Data())) {
      throw std::runtime_error("Failed to set tensor address for " + info.name);
    }
    tensor_buffers_.emplace(info.name, std::move(buffers));
  }

  cached_input_shape_ = input_shape;
}

void Context::FreeBuffers() {
  tensor_buffers_.clear();
  cached_input_shape_ = nvinfer1::Dims{};
}

std::unordered_map<std::string, std::vector<float>> Context::Infer(
    const void* input_data, const nvinfer1::Dims& input_shape) {
  AllocateBuffers(input_shape);

  const TensorInfo* input_info = nullptr;
  for (const auto& t : engine_.Tensors()) {
    if (t.is_input) {
      input_info = &t;
      break;
    }
  }
  if (!input_info) {
    throw std::runtime_error("No input tensor found");
  }
  const auto& input_buffers = tensor_buffers_.at(input_info->name);
  if (cudaMemcpyAsync(input_buffers.device.Data(),
                      input_data,
                      input_buffers.device.Bytes(),
                      cudaMemcpyHostToDevice,
                      stream_)
      != cudaSuccess) {
    throw std::runtime_error("Failed to copy input to device");
  }

  if (!ctx_->enqueueV3(stream_)) {
    throw std::runtime_error("Inference failed");
  }

  struct OutputMeta {
    nvinfer1::DataType dtype;
    size_t count = 0;
  };

  std::unordered_map<std::string, OutputMeta> output_meta;
  for (const auto& t : engine_.Tensors()) {
    if (t.is_input) {
      continue;
    }

    const auto& buffers = tensor_buffers_.at(t.name);
    const size_t count = buffers.device.Bytes() / DtypeBytes(t.dtype);
    if (t.dtype != nvinfer1::DataType::kFLOAT && t.dtype != nvinfer1::DataType::kHALF) {
      throw std::runtime_error("Unsupported output dtype for " + t.name);
    }

    if (cudaMemcpyAsync(buffers.host.Data(),
                        buffers.device.Data(),
                        buffers.device.Bytes(),
                        cudaMemcpyDeviceToHost,
                        stream_)
        != cudaSuccess) {
      throw std::runtime_error("Failed to copy output from device");
    }

    output_meta.emplace(t.name, OutputMeta{t.dtype, count});
  }

  if (cudaStreamSynchronize(stream_) != cudaSuccess) {
    throw std::runtime_error("Failed to synchronize CUDA stream");
  }

  std::unordered_map<std::string, std::vector<float>> outputs;
  for (const auto& kv : output_meta) {
    const auto& name = kv.first;
    const auto& meta = kv.second;
    const auto& buffers = tensor_buffers_.at(name);
    outputs.emplace(name, ConvertHostOutput(buffers.host.Data(), meta.dtype, meta.count));
  }

  return outputs;
}

}  // namespace perception_system

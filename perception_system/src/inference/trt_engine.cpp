#include "perception_system/inference/trt_engine.hpp"

#include <NvInfer.h>

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace perception_system {

void Engine::Logger::log(Severity severity, const char* msg) noexcept {
  if (severity <= Severity::kWARNING) {
    std::cerr << "[TRT] " << msg << "\n";
  }
}

Engine::Engine(const std::string& engine_path) {
  std::ifstream file(engine_path, std::ios::binary);
  if (!file) {
    throw std::runtime_error("Cannot open engine file: " + engine_path);
  }
  file.seekg(0, std::ios::end);
  const size_t size = file.tellg();
  file.seekg(0, std::ios::beg);
  std::vector<char> data(size);
  file.read(data.data(), size);

  runtime_.reset(nvinfer1::createInferRuntime(logger_));
  if (!runtime_) {
    throw std::runtime_error("Failed to create TensorRT runtime");
  }

  engine_.reset(runtime_->deserializeCudaEngine(data.data(), data.size()));
  if (!engine_) {
    throw std::runtime_error("Failed to deserialize engine: " + engine_path);
  }

  parse_tensors();
}

void Engine::parse_tensors() {
  const int n = engine_->getNbIOTensors();
  tensors_.reserve(n);
  for (int i = 0; i < n; ++i) {
    const char* name = engine_->getIOTensorName(i);
    TensorInfo info;
    info.name = name;
    info.dims = engine_->getTensorShape(name);
    info.dtype = engine_->getTensorDataType(name);
    info.is_input = (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT);
    tensors_.push_back(std::move(info));
  }
}

const TensorInfo* Engine::find_tensor(const std::string& name) const {
  for (const auto& t : tensors_) {
    if (t.name == name) return &t;
  }
  return nullptr;
}

}  // namespace perception_system

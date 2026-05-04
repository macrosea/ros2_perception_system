#pragma once

#include <NvInfer.h>

#include <memory>
#include <string>
#include <vector>

namespace perception_system {
struct TensorInfo {
  std::string name;
  nvinfer1::Dims dims;
  nvinfer1::DataType dtype;
  bool is_input;
};

class Engine {
 public:
  explicit Engine(const std::string& engine_path);
  ~Engine() = default;

  Engine(const Engine&) = delete;
  Engine& operator=(const Engine&) = delete;
  Engine(Engine&&) = default;
  Engine& operator=(Engine&&) = default;

  const std::vector<TensorInfo>& tensors() const {
    return tensors_;
  }

  const TensorInfo* find_tensor(const std::string& name) const;

  nvinfer1::ICudaEngine* raw() const {
    return engine_.get();
  }

 private:
  class Logger : public nvinfer1::ILogger {
   public:
    void log(Severity severity, const char* msg) noexcept override;
  };

  struct TrtDeleter {
    template <typename T>
    void operator()(T* obj) const {
      delete obj;
    }
  };

  Logger logger_;
  std::unique_ptr<nvinfer1::IRuntime, TrtDeleter> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine, TrtDeleter> engine_;
  std::vector<TensorInfo> tensors_;

  void parse_tensors();
};
}  // namespace perception_system

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace perception_system {

class InferenceAdapter {
 public:
  struct Output {
    std::vector<float> data;
    std::vector<int32_t> shape;
  };

  explicit InferenceAdapter(const std::string& model_path);
  ~InferenceAdapter();

  InferenceAdapter(const InferenceAdapter&) = delete;
  InferenceAdapter& operator=(const InferenceAdapter&) = delete;
  InferenceAdapter(InferenceAdapter&&) noexcept = default;
  InferenceAdapter& operator=(InferenceAdapter&&) noexcept = default;

  bool IsLoaded() const;
  Output Infer(const float* input_data, int input_size);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace perception_system

#include "perception_system/inference/inference_adapter.hpp"

#include <NvInfer.h>

#include <unordered_map>
#include <vector>

#include "perception_system/inference/trt_context.hpp"
#include "perception_system/inference/trt_engine.hpp"

namespace perception_system {

struct InferenceAdapter::Impl {
 public:
  explicit Impl(const std::string& model_path)

      : engine_(std::make_unique<Engine>(model_path)),
        context_(std::make_unique<Context>(*engine_)) {
  }

  std::unordered_map<std::string, std::vector<float>> Infer(const float* input_data,
                                                            int input_size) const {
    nvinfer1::Dims input_shape;
    input_shape.nbDims = 4;
    input_shape.d[0] = 1;
    input_shape.d[1] = 3;
    input_shape.d[2] = input_size;
    input_shape.d[3] = input_size;
    return context_->Infer(input_data, input_shape);
  }

  InferenceAdapter::Output GetInferResult(
      std::unordered_map<std::string, std::vector<float>>&& outputs, int input_size) const {
    InferenceAdapter::Output output;
    if (outputs.empty()) {
      return output;
    }
    // 约束：这里只考虑yolov8s 单头输出
    auto& first_output = *outputs.begin();
    output.data = std::move(first_output.second);
    const TensorInfo* tensor_info = engine_->FindTensor(first_output.first);
    if (tensor_info == nullptr) {
      return output;
    }

    nvinfer1::Dims output_dims = tensor_info->dims;
    if (output_dims.nbDims > 0 && output_dims.d[0] < 0) {
      output_dims.nbDims = 4;
      output_dims.d[0] = 1;
      output_dims.d[1] = 3;
      output_dims.d[2] = input_size;
      output_dims.d[3] = input_size;
    }
    for (int i = 0; i < output_dims.nbDims; ++i) {
      output.shape.push_back(static_cast<int32_t>(output_dims.d[i]));
    }

    return output;
  }

 private:
  std::unique_ptr<Engine> engine_;
  std::unique_ptr<Context> context_;
};

InferenceAdapter::InferenceAdapter(const std::string& model_path) {
  impl_ = std::make_unique<Impl>(model_path);
}

InferenceAdapter::~InferenceAdapter() = default;

bool InferenceAdapter::IsLoaded() const {
  return impl_ != nullptr;
}

InferenceAdapter::Output InferenceAdapter::Infer(const float* input_data, int input_size) {
  if (!impl_) {
    return {};
  }

  auto outputs = impl_->Infer(input_data, input_size);
  return impl_->GetInferResult(std::move(outputs), input_size);
}

}  // namespace perception_system

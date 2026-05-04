#include "perception_system/inference/inference_adapter.hpp"

#include <NvInfer.h>

#include "perception_system/inference/trt_context.hpp"
#include "perception_system/inference/trt_engine.hpp"

namespace perception_system {

struct InferenceAdapter::Impl {
  std::unique_ptr<Engine> engine;
  std::unique_ptr<Context> context;

  explicit Impl(const std::string& model_path) {
    engine = std::make_unique<Engine>(model_path);
    context = std::make_unique<Context>(*engine);
  }
};

InferenceAdapter::InferenceAdapter(const std::string& model_path) {
  impl_ = std::make_unique<Impl>(model_path);
}

InferenceAdapter::~InferenceAdapter() = default;

bool InferenceAdapter::IsLoaded() const {
  return impl_ != nullptr;
}

InferenceAdapter::Output InferenceAdapter::Infer(const float* input_data, int input_size) {
  Output output;
  if (!impl_) {
    return output;
  }

  nvinfer1::Dims input_shape;
  input_shape.nbDims = 4;
  input_shape.d[0] = 1;
  input_shape.d[1] = 3;
  input_shape.d[2] = input_size;
  input_shape.d[3] = input_size;

  auto outputs = impl_->context->infer(input_data, input_shape);
  if (outputs.empty()) {
    return output;
  }

  auto& first_output = *outputs.begin();
  output.data = std::move(first_output.second);

  const TensorInfo* tensor_info = impl_->engine->find_tensor(first_output.first);
  if (tensor_info != nullptr) {
    nvinfer1::Dims output_dims = tensor_info->dims;
    if (output_dims.nbDims > 0 && output_dims.d[0] < 0) {
      output_dims = input_shape;
    }
    for (int i = 0; i < output_dims.nbDims; ++i) {
      output.shape.push_back(static_cast<int32_t>(output_dims.d[i]));
    }
  }

  return output;
}

}  // namespace perception_system

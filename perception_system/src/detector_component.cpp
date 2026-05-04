#include "perception_system/detector_component.hpp"

#include <chrono>

#include "common/xlogger.hpp"
#include "perception_system/stage_tracepoint.hpp"

namespace perception_system {

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

DetectorComponent::DetectorComponent(const rclcpp::NodeOptions& options)
    : WorkerNode("detector_node", options) {
  if (!has_parameter("model_path")) {
    declare_parameter("model_path", std::string(""));
  }
  worker_cpu_ = 3;
  worker_priority_ = 80;
}

CallbackReturn DetectorComponent::on_configure(const rclcpp_lifecycle::State&) {
  model_path_ = get_parameter("model_path").as_string();

  if (model_input_size_ <= 0) {
    LOG_ERROR("invalid model_input_size=%d", model_input_size_);
    return CallbackReturn::FAILURE;
  }

  adapter_.reset();
  if (model_path_.empty()) {
    LOG_WARN("no model path; detector runs empty");
  } else {
    try {
      adapter_ = std::make_unique<InferenceAdapter>(model_path_);
    } catch (const std::exception& e) {
      LOG_ERROR("engine load failed: %s", e.what());
      return CallbackReturn::FAILURE;
    }
  }

  auto qos = rclcpp::SensorDataQoS();
  qos.keep_last(1);
  pub_ = create_publisher<msg::InferenceResult>("detector/inference_result", qos);

  LOG_INFO("configured model=%s input_size=%d cpu=%d prio=%d",
           model_path_.empty() ? "(none)" : model_path_.c_str(),
           model_input_size_,
           worker_cpu_,
           worker_priority_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn DetectorComponent::on_cleanup(const rclcpp_lifecycle::State&) {
  pub_.reset();
  adapter_.reset();
  return CallbackReturn::SUCCESS;
}

void DetectorComponent::OnActivated(const rclcpp_lifecycle::State&) {
  auto qos = rclcpp::SensorDataQoS();
  qos.keep_last(1);
  sub_ = create_subscription<msg::ProcessedImage>(
      "camera/image_proc",
      qos,
      [this](msg::ProcessedImage::ConstSharedPtr msg) { OnMessageInternal(msg); });
}

void DetectorComponent::OnDeactivating(const rclcpp_lifecycle::State&) {
  sub_.reset();
}

void DetectorComponent::OnErroring(const rclcpp_lifecycle::State&) {
  sub_.reset();
}

void DetectorComponent::OnErrored(const rclcpp_lifecycle::State&) {
  pub_.reset();
  adapter_.reset();
  LOG_ERROR("error: resources released");
}

void DetectorComponent::OnMessagePostStore(bool overwritten) {
  WorkerNode::OnMessagePostStore(overwritten);
  if (overwritten) {
    LOG_WARN_THROTTLE(*get_clock(), 1000, "latest-only overwrite=%lu", stats_.OverwriteDrop());
  }
}

void DetectorComponent::Process(std::shared_ptr<msg::ProcessedImage> msg) {
  LOG_DEBUG("sub: camera/image_proc frame_seq=%lu", msg->frame_seq);

  TraceStamp trace = TraceStamp::FromMsg(msg->trace);
  trace.t_det_in = static_cast<uint64_t>(now().nanoseconds());
  TRACETOOLS_TRACEPOINT(perception_system_detector_start, msg->frame_seq, trace.t_det_in);

  auto result = infer(*msg);

  trace.t_det_out = static_cast<uint64_t>(now().nanoseconds());
  TRACETOOLS_TRACEPOINT(perception_system_detector_end, msg->frame_seq, trace.t_det_out);

  if (result) {
    result->trace = trace.ToMsg();
    LOG_DEBUG("pub: detector/inference_result frame_seq=%lu", result->frame_seq);
    pub_->publish(std::move(result));
  }
}

std::unique_ptr<msg::InferenceResult> DetectorComponent::infer(msg::ProcessedImage& msg) {
  auto out = std::make_unique<msg::InferenceResult>();
  out->frame_seq = msg.frame_seq;
  out->source_stamp = msg.source_stamp;
  out->trace = msg.trace;
  out->image = std::move(msg.image);

  if (!adapter_ || !adapter_->IsLoaded() || msg.preprocessed_tensor.empty()) {
    return out;
  }

  auto result = adapter_->Infer(msg.preprocessed_tensor.data(), model_input_size_);
  out->output_data = std::move(result.data);
  out->output_shape = std::move(result.shape);

  return out;
}

}  // namespace perception_system

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(perception_system::DetectorComponent)

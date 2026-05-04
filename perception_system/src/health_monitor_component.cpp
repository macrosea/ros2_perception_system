#include "perception_system/health_monitor_component.hpp"

#include "common/xlogger.hpp"

namespace perception_system {

using namespace std::chrono_literals;

HealthMonitorComponent::HealthMonitorComponent(const rclcpp::NodeOptions& options)
    : rclcpp::Node("health_monitor", options) {
  declare_parameter("idle_cycle_limit", 3);
  declare_parameter("reset_cooldown_sec", 30);
  declare_parameter("input_fresh_sec", 10);

  idle_cycle_limit_ = get_parameter("idle_cycle_limit").as_int();
  reset_cooldown_sec_ = get_parameter("reset_cooldown_sec").as_int();
  input_fresh_sec_ = get_parameter("input_fresh_sec").as_int();

  auto qos = rclcpp::SensorDataQoS();
  qos.keep_last(1);

  processed_image_sub_ = create_subscription<msg::ProcessedImage>(
      "camera/image_proc",
      qos,
      [this](msg::ProcessedImage::ConstSharedPtr msg) { on_processed_image(std::move(msg)); });

  inference_result_sub_ = create_subscription<msg::InferenceResult>(
      "detector/inference_result",
      qos,
      [this](msg::InferenceResult::ConstSharedPtr msg) { on_inference_result(std::move(msg)); });

  detection_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      "detector/detections",
      qos,
      [this](vision_msgs::msg::Detection2DArray::ConstSharedPtr msg) {
        on_detection(std::move(msg));
      });

  reset_client_ = create_client<srv::ResetNode>("/lifecycle_manager/reset_node");

  health_check_timer_ = create_wall_timer(5s, [this]() { health_check(); });

  LOG_INFO("health_monitor started cycle_limit=%d cooldown=%ds fresh=%ds",
           idle_cycle_limit_,
           reset_cooldown_sec_,
           input_fresh_sec_);
}

void HealthMonitorComponent::on_processed_image(msg::ProcessedImage::ConstSharedPtr) {
  detector_watch_.last_input_ns.store(now().nanoseconds(), std::memory_order_relaxed);
}

void HealthMonitorComponent::on_inference_result(msg::InferenceResult::ConstSharedPtr) {
  const int64_t ns = now().nanoseconds();
  detector_watch_.last_output_ns.store(ns, std::memory_order_relaxed);
  detector_watch_.idle_cycles.store(0, std::memory_order_relaxed);
  visualize_watch_.last_input_ns.store(ns, std::memory_order_relaxed);
}

void HealthMonitorComponent::on_detection(vision_msgs::msg::Detection2DArray::ConstSharedPtr) {
  visualize_watch_.last_output_ns.store(now().nanoseconds(), std::memory_order_relaxed);
  visualize_watch_.idle_cycles.store(0, std::memory_order_relaxed);
}

void HealthMonitorComponent::health_check() {
  check_watch(detector_watch_, "detector_node");
  check_watch(visualize_watch_, "visualize_node");
}

void HealthMonitorComponent::check_watch(NodeWatch& watch, const std::string& node_name) {
  const int64_t now_ns = now().nanoseconds();
  const int64_t last_input = watch.last_input_ns.load(std::memory_order_relaxed);
  const int64_t last_output = watch.last_output_ns.load(std::memory_order_relaxed);
  const int64_t last_reset = watch.last_reset_ns.load(std::memory_order_relaxed);

  if (last_input == 0) return;
  if ((now_ns - last_input) > static_cast<int64_t>(input_fresh_sec_) * 1'000'000'000ll) return;
  if (last_output != 0 && last_output >= last_input) return;
  if (last_reset != 0
      && (now_ns - last_reset) < static_cast<int64_t>(reset_cooldown_sec_) * 1'000'000'000ll) {
    return;
  }

  const int cycles = watch.idle_cycles.fetch_add(1, std::memory_order_relaxed) + 1;
  if (cycles < idle_cycle_limit_) return;

  watch.idle_cycles.store(0, std::memory_order_relaxed);
  watch.last_reset_ns.store(now_ns, std::memory_order_relaxed);
  request_reset(node_name);
}

void HealthMonitorComponent::request_reset(const std::string& node_name) {
  if (!reset_client_->service_is_ready()) {
    LOG_WARN("[%s] lifecycle_manager/reset_node not ready", node_name.c_str());
    return;
  }
  auto req = std::make_shared<srv::ResetNode::Request>();
  req->node_name = node_name;
  reset_client_->async_send_request(
      req,
      [node_name](rclcpp::Client<srv::ResetNode>::SharedFuture future) {
        if (!future.get()->accepted) {
          LOG_WARN("[%s] reset request rejected (shutting down?)", node_name.c_str());
        }
      });
}

}  // namespace perception_system

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(perception_system::HealthMonitorComponent)

#pragma once

#include <atomic>
#include <string>

#include "perception_system/msg/inference_result.hpp"
#include "perception_system/msg/processed_image.hpp"
#include "perception_system/srv/reset_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace perception_system {

struct NodeWatch {
  std::atomic<int> idle_cycles{0};
  std::atomic<int64_t> last_input_ns{0};
  std::atomic<int64_t> last_output_ns{0};
  std::atomic<int64_t> last_reset_ns{0};
};

class HealthMonitorComponent : public rclcpp::Node {
 public:
  explicit HealthMonitorComponent(const rclcpp::NodeOptions& options);

 private:
  void on_processed_image(msg::ProcessedImage::ConstSharedPtr msg);
  void on_inference_result(msg::InferenceResult::ConstSharedPtr msg);
  void on_detection(vision_msgs::msg::Detection2DArray::ConstSharedPtr msg);

  void health_check();
  void check_watch(NodeWatch& watch, const std::string& node_name);
  void request_reset(const std::string& node_name);

  rclcpp::Subscription<msg::ProcessedImage>::SharedPtr processed_image_sub_;
  rclcpp::Subscription<msg::InferenceResult>::SharedPtr inference_result_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
  rclcpp::TimerBase::SharedPtr health_check_timer_;
  rclcpp::Client<srv::ResetNode>::SharedPtr reset_client_;

  NodeWatch detector_watch_;
  NodeWatch visualize_watch_;

  int idle_cycle_limit_;
  int reset_cooldown_sec_;
  int input_fresh_sec_;
};

}  // namespace perception_system

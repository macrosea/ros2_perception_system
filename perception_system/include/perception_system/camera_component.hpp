#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace perception_system::capture {
class CaptureSource;
}

namespace perception_system {

class CameraComponent : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit CameraComponent(const rclcpp::NodeOptions& options);
  ~CameraComponent();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(
      const rclcpp_lifecycle::State& state) override;

 private:
  void publish_frame();

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<capture::CaptureSource> capture_source_;
  std::string input_;
  std::string frame_id_{"camera"};
  uint32_t frame_width_{1920};
  uint32_t frame_height_{1080};
  double publish_rate_hz_{30.0};

  uint64_t frame_pub_cnt_{0};
};

}  // namespace perception_system

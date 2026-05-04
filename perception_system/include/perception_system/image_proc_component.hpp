#pragma once

#include <atomic>
#include <cstdint>
#include <memory>

#include "perception_system/msg/processed_image.hpp"
#include "perception_system/preprocessing/preprocessor.hpp"
#include "perception_system/worker_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace perception_system {

class ImageProcComponent : public WorkerNode<sensor_msgs::msg::Image> {
 public:
  explicit ImageProcComponent(const rclcpp::NodeOptions& options);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& state) override;

 protected:
  CallbackReturn OnActivating(const rclcpp_lifecycle::State& state) override;
  void OnActivated(const rclcpp_lifecycle::State& state) override;
  void OnDeactivating(const rclcpp_lifecycle::State& state) override;
  void OnDeactivated(const rclcpp_lifecycle::State& state) override;
  void OnErroring(const rclcpp_lifecycle::State& state) override;
  void OnErrored(const rclcpp_lifecycle::State& state) override;

  void Process(std::shared_ptr<sensor_msgs::msg::Image> msg) override;
  void OnMessagePostStore(bool overwritten) override;

 private:
  std::unique_ptr<msg::ProcessedImage> process(sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp_lifecycle::LifecyclePublisher<msg::ProcessedImage>::SharedPtr pub_;

  std::atomic<uint64_t> freshness_drop_cnt_{0};
  int64_t input_max_age_ns_{0};

  uint64_t next_frame_seq_{0};

  int model_input_size_{640};

  std::unique_ptr<Preprocessor> preprocessor_;
};

}  // namespace perception_system

#pragma once

#include <atomic>
#include <memory>
#include <string>

#include "common/trace.hpp"
#include "perception_system/inference/inference_adapter.hpp"
#include "perception_system/msg/inference_result.hpp"
#include "perception_system/msg/processed_image.hpp"
#include "perception_system/worker_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace perception_system {

class DetectorComponent : public WorkerNode<msg::ProcessedImage> {
 public:
  explicit DetectorComponent(const rclcpp::NodeOptions& options);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& state) override;

 protected:
  void OnActivated(const rclcpp_lifecycle::State& state) override;
  void OnDeactivating(const rclcpp_lifecycle::State& state) override;
  void OnErroring(const rclcpp_lifecycle::State& state) override;
  void OnErrored(const rclcpp_lifecycle::State& state) override;

  void Process(std::shared_ptr<msg::ProcessedImage> msg) override;
  void OnMessagePostStore(bool overwritten) override;

 private:
  std::unique_ptr<msg::InferenceResult> infer(msg::ProcessedImage& msg);

  rclcpp::Subscription<msg::ProcessedImage>::SharedPtr sub_;
  rclcpp_lifecycle::LifecyclePublisher<msg::InferenceResult>::SharedPtr pub_;
  std::string model_path_;
  int model_input_size_{640};

  std::unique_ptr<InferenceAdapter> adapter_;
};

}  // namespace perception_system

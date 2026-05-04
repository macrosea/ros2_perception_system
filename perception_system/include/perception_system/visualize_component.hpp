#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "common/frame_rate_stats.hpp"
#include "common/latency_stats.hpp"
#include "perception_system/msg/inference_result.hpp"
#include "perception_system/worker_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace cv {
class Mat;
}  // namespace cv

namespace perception_system {

class VisualizeComponent : public WorkerNode<msg::InferenceResult> {
 public:
  explicit VisualizeComponent(const rclcpp::NodeOptions& options);

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

  void Process(std::shared_ptr<msg::InferenceResult> msg) override;
  void OnMessagePostStore(bool overwritten) override;
  void OnWorkerStarted() override;

 private:
  void ResetStats();
  void LogStats(bool before_stop, bool reset);
  bool ProcessResult(const msg::InferenceResult& result, uint64_t frame_count);
  void SaveFrame(const cv::Mat& image, uint64_t frame_count) const;

  rclcpp::Subscription<msg::InferenceResult>::SharedPtr sub_result_;
  rclcpp_lifecycle::LifecyclePublisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_det_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  std::mutex stats_mutex_;
  FrameRateStats frame_rate_stats_;
  LatencyStats stats_;

  std::string output_dir_{"/tmp/perception_output"};
  int num_classes_{80};
  int num_anchors_{8400};
  float score_threshold_{0.25f};
  float iou_threshold_{0.45f};
  int model_input_size_{640};

  uint64_t frame_count_{0};
  uint64_t skip_count_{0};
};

}  // namespace perception_system

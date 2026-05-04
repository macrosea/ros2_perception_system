#include "perception_system/visualize_component.hpp"

#include <chrono>
#include <filesystem>
#include <string>

#include "common/affinity.hpp"
#include "common/trace.hpp"
#include "common/xlogger.hpp"
#include "perception_system/visualize/visualizer.hpp"

namespace perception_system {

using namespace std::chrono_literals;

namespace {
constexpr int kStatsIntervalSec = 10;

rclcpp::SensorDataQoS CreateSensorQos() {
  auto qos = rclcpp::SensorDataQoS();
  qos.keep_last(1);
  return qos;
}
}  // namespace

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

VisualizeComponent::VisualizeComponent(const rclcpp::NodeOptions& options)
    : WorkerNode("visualize_node", options) {
  declare_parameter("output_dir", output_dir_);
  declare_parameter("num_classes", num_classes_);
  declare_parameter("num_anchors", num_anchors_);
  declare_parameter("score_threshold", score_threshold_);
  declare_parameter("iou_threshold", iou_threshold_);
  worker_cpu_ = 4;
  worker_priority_ = 0;
}

CallbackReturn VisualizeComponent::on_configure(const rclcpp_lifecycle::State&) {
  output_dir_ = get_parameter("output_dir").as_string();
  num_classes_ = get_parameter("num_classes").as_int();
  num_anchors_ = get_parameter("num_anchors").as_int();
  score_threshold_ = static_cast<float>(get_parameter("score_threshold").as_double());
  iou_threshold_ = static_cast<float>(get_parameter("iou_threshold").as_double());

  if (output_dir_.empty()) {
    LOG_ERROR("output_dir must not be empty");
    return CallbackReturn::FAILURE;
  }
  if (model_input_size_ <= 0) {
    LOG_ERROR("invalid model_input_size=%d", model_input_size_);
    return CallbackReturn::FAILURE;
  }

  pub_det_ = create_publisher<vision_msgs::msg::Detection2DArray>("detector/detections",
                                                                  CreateSensorQos());

  LOG_INFO("configured output=%s classes=%d anchors=%d thr=%.2f/%.2f size=%d cpu=%d prio=%d",
           output_dir_.c_str(),
           num_classes_,
           num_anchors_,
           score_threshold_,
           iou_threshold_,
           model_input_size_,
           worker_cpu_,
           worker_priority_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn VisualizeComponent::OnActivating(const rclcpp_lifecycle::State&) {
  ResetStats();
  frame_count_ = 0;
  skip_count_ = 0;
  return CallbackReturn::SUCCESS;
}

void VisualizeComponent::OnActivated(const rclcpp_lifecycle::State&) {
  sub_result_ = create_subscription<msg::InferenceResult>(
      "detector/inference_result",
      CreateSensorQos(),
      [this](msg::InferenceResult::ConstSharedPtr msg) { OnMessageInternal(std::move(msg)); });

  stats_timer_ = create_wall_timer(std::chrono::seconds(kStatsIntervalSec),
                                   [this]() { LogStats(false, true); });
}

void VisualizeComponent::OnDeactivating(const rclcpp_lifecycle::State&) {
  sub_result_.reset();
  stats_timer_.reset();
}

void VisualizeComponent::OnDeactivated(const rclcpp_lifecycle::State&) {
  LogStats(true, false);
  LOG_INFO("worker done: frames=%lu skipped=%lu", frame_count_, skip_count_);
}

void VisualizeComponent::OnErroring(const rclcpp_lifecycle::State&) {
  sub_result_.reset();
  stats_timer_.reset();
}

void VisualizeComponent::OnErrored(const rclcpp_lifecycle::State&) {
  pub_det_.reset();
  ResetStats();
  LOG_ERROR("error: resources released");
}

void VisualizeComponent::OnMessagePostStore(bool overwritten) {
  WorkerNode::OnMessagePostStore(overwritten);
  std::lock_guard<std::mutex> lock(stats_mutex_);
  if (overwritten) {
    stats_.count_overwrite_drop();
  }
}

void VisualizeComponent::OnWorkerStarted() {
  std::error_code ec;
  std::filesystem::create_directories(output_dir_, ec);
  if (ec) {
    LOG_ERROR("failed to create output dir %s: %s", output_dir_.c_str(), ec.message().c_str());
  }
}

void VisualizeComponent::Process(std::shared_ptr<msg::InferenceResult> msg) {
  LOG_DEBUG("sub: detector/inference_result frame_seq=%lu", msg->frame_seq);

  if (!ProcessResult(*msg, frame_count_)) {
    ++skip_count_;
    return;
  }

  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    frame_rate_stats_.count_frame();
    stats_.record(TraceStamp::FromMsg(msg->trace));
  }

  ++frame_count_;
}

CallbackReturn VisualizeComponent::on_cleanup(const rclcpp_lifecycle::State&) {
  pub_det_.reset();
  ResetStats();
  return CallbackReturn::SUCCESS;
}

void VisualizeComponent::ResetStats() {
  std::lock_guard<std::mutex> lock(stats_mutex_);
  frame_rate_stats_.reset();
  stats_.reset();
}

void VisualizeComponent::LogStats(const bool before_stop, const bool reset) {
  std::lock_guard<std::mutex> lock(stats_mutex_);
  const uint64_t total_drop = stats_.overwrite_drop() + stats_.freshness_drop();
  const bool has_fps_samples = frame_rate_stats_.has_samples();
  const uint64_t output_frames = frame_rate_stats_.frame_count();
  const double output_fps = frame_rate_stats_.fps();

  if (has_fps_samples) {
    LOG_INFO("[VisualizeFps] out=%lu fps=%.2f", output_frames, output_fps);
  } else if (before_stop) {
    LOG_INFO("[VisualizeFps] no output samples before stop");
  } else {
    LOG_INFO("[VisualizeFps] no output samples");
  }

  if (stats_.e2e().count() == 0) {
    if (before_stop) {
      LOG_INFO("[LatencyStats] no samples before stop in=%lu drop=%lu",
               stats_.frame_in(),
               total_drop);
    } else {
      LOG_INFO("[LatencyStats] no samples in=%lu drop=%lu", stats_.frame_in(), total_drop);
    }
  } else {
    const double drop_rate
        = stats_.frame_in() > 0 ? static_cast<double>(total_drop) / stats_.frame_in() : 0.0;

    stats_.log();

    if (!before_stop && total_drop > 0) {
      LOG_WARN("[VisualizeHealth] drop=%.1f%% overwrite=%lu stale=%lu",
               drop_rate * 100.0,
               stats_.overwrite_drop(),
               stats_.freshness_drop());
    }
  }

  if (reset) {
    frame_rate_stats_.reset();
    stats_.reset();
  }
}

bool VisualizeComponent::ProcessResult(const msg::InferenceResult& result,
                                       const uint64_t frame_count) {
  if (result.output_data.empty()) {
    LOG_WARN("skip: no output_data frame_seq=%lu", result.frame_seq);
    return false;
  }

  cv::Mat image = Visualizer::DecodeImage(result.image);
  if (image.empty()) {
    LOG_WARN("skip: decode failed frame_seq=%lu", result.frame_seq);
    return false;
  }

  const float* output_data = result.output_data.data();
  cv::Size orig_size(image.cols, image.rows);

  auto detections = yolov8::postprocess(output_data,
                                        num_classes_,
                                        num_anchors_,
                                        score_threshold_,
                                        iou_threshold_,
                                        orig_size,
                                        model_input_size_);

  LOG_DEBUG("postprocess: %zu detections frame_seq=%lu", detections.size(), result.frame_seq);
  if (!detections.empty()) {
    LOG_DEBUG("pub: detector/detections frame_seq=%lu count=%zu",
              result.frame_seq,
              detections.size());
    pub_det_->publish(Visualizer::ToMsg(detections, result.source_stamp));

    Visualizer::DrawDetections(image, detections);
    SaveFrame(image, frame_count);
  }

  return true;
}

void VisualizeComponent::SaveFrame(const cv::Mat& image, const uint64_t frame_count) const {
  char filename[256];
  snprintf(filename, sizeof(filename), "%s/frame_%06lu.jpg", output_dir_.c_str(), frame_count);
  if (!cv::imwrite(filename, image)) {
    LOG_WARN("failed to save %s", filename);
  }
}

}  // namespace perception_system

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(perception_system::VisualizeComponent)

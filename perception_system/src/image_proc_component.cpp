#include "perception_system/image_proc_component.hpp"

#include <thread>

#include "common/trace.hpp"
#include "common/xlogger.hpp"
#include "perception_system/stage_tracepoint.hpp"

namespace perception_system {

namespace {
constexpr int kStatsThrottleMs = 1000;
constexpr int64_t kMinReasonableAgeNs = 10'000'000;  // 10ms
}  // namespace

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

ImageProcComponent::ImageProcComponent(const rclcpp::NodeOptions& options)
    : WorkerNode("image_proc_node", options) {
  worker_cpu_ = 2;
  worker_priority_ = 70;
}

CallbackReturn ImageProcComponent::on_configure(const rclcpp_lifecycle::State&) {
  const int num_cpus = static_cast<int>(std::thread::hardware_concurrency());
  if (worker_cpu_ < -1 || worker_cpu_ >= num_cpus) {
    LOG_ERROR("invalid worker_cpu=%d (valid: -1 or 0-%d)", worker_cpu_, num_cpus - 1);
    return CallbackReturn::FAILURE;
  }

  if (worker_priority_ < 0 || worker_priority_ > 99) {
    LOG_ERROR("invalid worker_priority=%d (valid: 0-99)", worker_priority_);
    return CallbackReturn::FAILURE;
  }

  if (model_input_size_ <= 0) {
    LOG_ERROR("invalid model_input_size=%d", model_input_size_);
    return CallbackReturn::FAILURE;
  }

  if (input_max_age_ns_ > 0 && input_max_age_ns_ < kMinReasonableAgeNs) {
    LOG_WARN("input_max_age_ns=%ld is very small (< 10ms), may drop all frames", input_max_age_ns_);
  }

  try {
    preprocessor_ = std::make_unique<Preprocessor>(model_input_size_);
  } catch (const std::exception& e) {
    LOG_ERROR("preprocessor init failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }

  auto qos = rclcpp::SensorDataQoS();
  qos.keep_last(1);
  pub_ = create_publisher<msg::ProcessedImage>("camera/image_proc", qos);

  LOG_INFO("configured cpu=%d prio=%d max_age_ns=%ld",
           worker_cpu_,
           worker_priority_,
           input_max_age_ns_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn ImageProcComponent::OnActivating(const rclcpp_lifecycle::State&) {
  freshness_drop_cnt_.store(0, std::memory_order_relaxed);
  next_frame_seq_ = 0;
  return CallbackReturn::SUCCESS;
}

void ImageProcComponent::OnActivated(const rclcpp_lifecycle::State&) {
  auto qos = rclcpp::SensorDataQoS();
  qos.keep_last(1);
  sub_ = create_subscription<sensor_msgs::msg::Image>(
      "camera/image_raw",
      qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { OnMessageInternal(msg); });
}

void ImageProcComponent::OnDeactivating(const rclcpp_lifecycle::State&) {
  sub_.reset();
}

void ImageProcComponent::OnDeactivated(const rclcpp_lifecycle::State&) {
  LOG_INFO("stopped");
}

void ImageProcComponent::OnErroring(const rclcpp_lifecycle::State&) {
  sub_.reset();
}

void ImageProcComponent::OnErrored(const rclcpp_lifecycle::State&) {
  pub_.reset();
  preprocessor_.reset();
  LOG_ERROR("error: resources released");
}

void ImageProcComponent::OnMessagePostStore(bool overwritten) {
  WorkerNode::OnMessagePostStore(overwritten);
  if (overwritten) {
    LOG_WARN_THROTTLE(*get_clock(),
                      kStatsThrottleMs,
                      "latest-only overwrite=%lu",
                      stats_.OverwriteDrop());
  }
}

void ImageProcComponent::Process(std::shared_ptr<sensor_msgs::msg::Image> msg) {
  LOG_DEBUG("sub: camera/image_raw h=%u w=%u step=%u", msg->height, msg->width, msg->step);

  const rclcpp::Time source_time(msg->header.stamp);
  const rclcpp::Time now_time = now();
  const rclcpp::Duration age = now_time - source_time;

  if (age.nanoseconds() < 0) {
    LOG_WARN_THROTTLE(*get_clock(),
                      kStatsThrottleMs,
                      "clock jump detected: age=%.1fms (negative), processing anyway",
                      age.seconds() * 1000.0);
  } else if (input_max_age_ns_ > 0 && age.nanoseconds() > input_max_age_ns_) {
    freshness_drop_cnt_.fetch_add(1, std::memory_order_relaxed);
    LOG_WARN_THROTTLE(*get_clock(),
                      kStatsThrottleMs,
                      "stale frame dropped: age=%.1fms > max=%.1fms, drop_count=%lu",
                      age.seconds() * 1000.0,
                      input_max_age_ns_ / (1000 * 1000.0),
                      freshness_drop_cnt_.load(std::memory_order_relaxed));
    return;
  }

  TraceStamp trace{};
  trace.t_cam_pub = StampToNs(msg->header.stamp);
  trace.t_proc_in = static_cast<uint64_t>(now_time.nanoseconds());
  TRACETOOLS_TRACEPOINT(perception_system_image_proc_start, next_frame_seq_, trace.t_proc_in);

  auto out = process(msg);

  trace.t_proc_out = static_cast<uint64_t>(now().nanoseconds());

  if (out) {
    out->frame_seq = next_frame_seq_++;
    out->source_stamp = msg->header.stamp;
    out->trace = trace.ToMsg();
    TRACETOOLS_TRACEPOINT(perception_system_image_proc_end, out->frame_seq, trace.t_proc_out);
    LOG_DEBUG("pub: camera/image_proc frame_seq=%lu", out->frame_seq);
    pub_->publish(std::move(out));
  } else {
    LOG_ERROR_THROTTLE(*get_clock(), kStatsThrottleMs, "preprocess failed");
  }
}

CallbackReturn ImageProcComponent::on_cleanup(const rclcpp_lifecycle::State&) {
  pub_.reset();
  preprocessor_.reset();
  return CallbackReturn::SUCCESS;
}

std::unique_ptr<msg::ProcessedImage> ImageProcComponent::process(
    sensor_msgs::msg::Image::SharedPtr msg) {
  auto out = std::make_unique<msg::ProcessedImage>();

  int orig_width, orig_height;
  float scale;
  std::vector<float> tensor_buffer;
  if (!preprocessor_->Process(*msg, tensor_buffer, orig_width, orig_height, scale)) {
    return nullptr;
  }

  out->image.header = msg->header;
  out->image.header.stamp = now();
  out->image.width = msg->width;
  out->image.height = msg->height;
  out->image.encoding = msg->encoding;
  out->image.step = msg->step;
  out->image.data = std::move(msg->data);
  out->preprocessed_tensor = std::move(tensor_buffer);
  return out;
}

}  // namespace perception_system

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(perception_system::ImageProcComponent)

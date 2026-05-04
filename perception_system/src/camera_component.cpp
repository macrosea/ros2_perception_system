#include "perception_system/camera_component.hpp"

#include <chrono>
#include <utility>

#include "common/xlogger.hpp"
#include "perception_system/capture/capture_source.hpp"

namespace perception_system {

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CameraComponent::CameraComponent(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("camera_node", options) {
  if (!has_parameter("input")) {
    declare_parameter("input", input_);
  }
  if (!has_parameter("frame_width")) {
    declare_parameter("frame_width", static_cast<int>(frame_width_));
  }
  if (!has_parameter("frame_height")) {
    declare_parameter("frame_height", static_cast<int>(frame_height_));
  }
  if (!has_parameter("publish_rate_hz")) {
    declare_parameter("publish_rate_hz", publish_rate_hz_);
  }
  if (!has_parameter("frame_id")) {
    declare_parameter("frame_id", frame_id_);
  }
}

CameraComponent::~CameraComponent() = default;

CallbackReturn CameraComponent::on_configure(const rclcpp_lifecycle::State&) {
  input_ = get_parameter("input").as_string();
  frame_width_ = static_cast<uint32_t>(get_parameter("frame_width").as_int());
  frame_height_ = static_cast<uint32_t>(get_parameter("frame_height").as_int());
  publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
  frame_id_ = get_parameter("frame_id").as_string();

  capture_source_ = std::make_unique<capture::CaptureSource>();
  try {
    capture_source_->Initialize({.input = input_});
  } catch (const std::exception& e) {
    LOG_ERROR("failed to open capture source: %s\n input: %s", e.what(), input_.c_str());
    capture_source_.reset();
    return CallbackReturn::FAILURE;
  }

  const auto& info = capture_source_->GetInfo();
  if (info.width > 0 && info.height > 0) {
    frame_width_ = static_cast<uint32_t>(info.width);
    frame_height_ = static_cast<uint32_t>(info.height);
  }

  auto qos = rclcpp::SensorDataQoS();
  qos.keep_last(1);
  pub_ = create_publisher<sensor_msgs::msg::Image>("camera/image_raw", qos);

  LOG_INFO("configured %ux%u @ %.1f Hz, input=%s",
           frame_width_,
           frame_height_,
           publish_rate_hz_,
           input_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn CameraComponent::on_activate(const rclcpp_lifecycle::State& state) {
  LifecycleNode::on_activate(state);

  capture_source_->StartCapture();

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));

  frame_pub_cnt_ = 0;
  timer_ = create_wall_timer(period, [this]() { publish_frame(); });

  return CallbackReturn::SUCCESS;
}

CallbackReturn CameraComponent::on_deactivate(const rclcpp_lifecycle::State& state) {
  timer_.reset();
  capture_source_->StopCapture();
  LifecycleNode::on_deactivate(state);
  LOG_INFO("stopped pub=%lu", frame_pub_cnt_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn CameraComponent::on_cleanup(const rclcpp_lifecycle::State&) {
  if (capture_source_) {
    capture_source_->Shutdown();
  }
  pub_.reset();
  capture_source_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn CameraComponent::on_error(const rclcpp_lifecycle::State&) {
  timer_.reset();
  if (capture_source_) {
    capture_source_->Shutdown();
  }
  pub_.reset();
  capture_source_.reset();
  LOG_ERROR("error: resources released");
  return CallbackReturn::SUCCESS;
}

void CameraComponent::publish_frame() {
  if (!capture_source_) {
    return;
  }
  static cv::Mat reusable_buffer;

  capture::Frame frame;
  if (!capture_source_->GrabFrameInto(reusable_buffer, frame)) {
    static bool eof_logged = false;
    if (!eof_logged) {
      LOG_WARN("EOF reached, no more frames");
      eof_logged = true;
    }
    return;
  }

  auto loaned_msg = pub_->borrow_loaned_message();
  if (loaned_msg.is_valid()) {
    auto& msg = loaned_msg.get();
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.height = static_cast<uint32_t>(frame.height);
    msg.width = static_cast<uint32_t>(frame.width);
    msg.encoding = "bgr8";
    msg.step = static_cast<uint32_t>(frame.image_bgr.step[0]);

    const size_t data_size = msg.step * frame.height;
    msg.data.resize(data_size);
    std::memcpy(msg.data.data(), reusable_buffer.data, data_size);

    LOG_DEBUG("pub: camera/image_raw cnt=%lu %ux%u (loaned)",
              frame_pub_cnt_,
              msg.height,
              msg.width);
    pub_->publish(std::move(loaned_msg));
  } else {
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->header.stamp = now();
    msg->header.frame_id = frame_id_;
    msg->height = static_cast<uint32_t>(frame.height);
    msg->width = static_cast<uint32_t>(frame.width);
    msg->encoding = "bgr8";
    msg->step = static_cast<uint32_t>(frame.image_bgr.step[0]);

    const size_t data_size = msg->step * frame.height;
    msg->data.assign(reusable_buffer.data, reusable_buffer.data + data_size);

    LOG_DEBUG("pub: camera/image_raw cnt=%lu %ux%u (standard)",
              frame_pub_cnt_,
              msg->height,
              msg->width);
    pub_->publish(std::move(msg));
  }

  ++frame_pub_cnt_;
}

}  // namespace perception_system

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(perception_system::CameraComponent)

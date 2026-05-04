#include "perception_system/visualize/visualizer.hpp"

#include <string>

#include "common/xlogger.hpp"

namespace perception_system {

cv::Mat Visualizer::DecodeImage(const sensor_msgs::msg::Image& img_msg) {
  if (img_msg.encoding == "bgr8" && img_msg.step == static_cast<uint32_t>(img_msg.width * 3)) {
    cv::Mat mat(img_msg.height,
                img_msg.width,
                CV_8UC3,
                const_cast<uint8_t*>(img_msg.data.data()),
                img_msg.step);
    return mat.clone();
  }
  std::vector<uint8_t> buf(img_msg.data.begin(), img_msg.data.end());
  cv::Mat decoded = cv::imdecode(buf, cv::IMREAD_COLOR);
  if (decoded.empty()) {
    LOG_WARN("failed to decode image, encoding=%s", img_msg.encoding.c_str());
  }
  return decoded;
}

std::unique_ptr<vision_msgs::msg::Detection2DArray> Visualizer::ToMsg(
    const std::vector<yolov8::Detection>& dets, const builtin_interfaces::msg::Time& stamp) {
  auto msg = std::make_unique<vision_msgs::msg::Detection2DArray>();
  msg->header.stamp = stamp;
  msg->header.frame_id = "visualize";
  for (const auto& det : dets) {
    vision_msgs::msg::Detection2D det2d;
    det2d.bbox.center.position.x = static_cast<double>(det.x);
    det2d.bbox.center.position.y = static_cast<double>(det.y);
    det2d.bbox.size_x = static_cast<double>(det.w);
    det2d.bbox.size_y = static_cast<double>(det.h);
    vision_msgs::msg::ObjectHypothesisWithPose hyp;
    hyp.hypothesis.class_id = std::to_string(det.class_id);
    hyp.hypothesis.score = static_cast<double>(det.score);
    det2d.results.push_back(hyp);
    msg->detections.push_back(det2d);
  }
  return msg;
}

void Visualizer::DrawDetections(cv::Mat& image, const std::vector<yolov8::Detection>& dets) {
  for (const auto& det : dets) {
    const int x1 = static_cast<int>(det.x - det.w / 2.f);
    const int y1 = static_cast<int>(det.y - det.h / 2.f);
    const int x2 = static_cast<int>(det.x + det.w / 2.f);
    const int y2 = static_cast<int>(det.y + det.h / 2.f);
    cv::rectangle(image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
    const std::string label = "cls" + std::to_string(det.class_id) + " "
                              + std::to_string(static_cast<int>(det.score * 100)) + "%";
    cv::putText(image,
                label,
                cv::Point(x1, y1 - 5),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(0, 255, 0),
                1);
  }
}

}  // namespace perception_system

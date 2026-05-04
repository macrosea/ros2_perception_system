#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "perception_system/visualize/yolov8_postprocess.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace perception_system {

class Visualizer {
 public:
  static cv::Mat DecodeImage(const sensor_msgs::msg::Image& img_msg);

  static std::unique_ptr<vision_msgs::msg::Detection2DArray> ToMsg(
      const std::vector<yolov8::Detection>& dets, const builtin_interfaces::msg::Time& stamp);

  static void DrawDetections(cv::Mat& image, const std::vector<yolov8::Detection>& dets);
};

}  // namespace perception_system

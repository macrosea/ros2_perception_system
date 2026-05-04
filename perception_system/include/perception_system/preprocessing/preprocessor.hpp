#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "sensor_msgs/msg/image.hpp"

namespace perception_system {

class Preprocessor {
 public:
  explicit Preprocessor(int input_size);
  ~Preprocessor();

  bool Process(const sensor_msgs::msg::Image& msg,
               std::vector<float>& output,
               int& orig_width,
               int& orig_height,
               float& scale);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace perception_system

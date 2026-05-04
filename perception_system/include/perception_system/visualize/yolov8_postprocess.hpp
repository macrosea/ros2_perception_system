#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

namespace yolov8 {

struct Detection {
  float x, y, w, h;  // 原图坐标系下的 box (center_x, center_y, width, height)
  float score;
  int class_id;
};

// YOLOv8 后处理
// output: [1, 84, 8400] 格式的原始输出 (4 box coords + 80 class scores)
// num_classes: 类别数（COCO 为 80）
// num_anchors: anchor 数量（YOLOv8s 为 8400）
// score_threshold: 置信度阈值
// iou_threshold: NMS IoU 阈值
// orig_size: 原图尺寸（用于坐标还原）
// input_size: 模型输入尺寸（默认 640）
std::vector<Detection> postprocess(const float* output,
                                   int num_classes,
                                   int num_anchors,
                                   float score_threshold,
                                   float iou_threshold,
                                   const cv::Size& orig_size,
                                   int input_size = 640);

}  // namespace yolov8

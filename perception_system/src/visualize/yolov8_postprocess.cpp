#include "perception_system/visualize/yolov8_postprocess.hpp"

#include <algorithm>
#include <cmath>

namespace yolov8 {

// IoU of two boxes in (cx, cy, w, h) format
static float iou(const Detection& a, const Detection& b) {
  const float ax1 = a.x - a.w / 2.f, ay1 = a.y - a.h / 2.f;
  const float ax2 = a.x + a.w / 2.f, ay2 = a.y + a.h / 2.f;
  const float bx1 = b.x - b.w / 2.f, by1 = b.y - b.h / 2.f;
  const float bx2 = b.x + b.w / 2.f, by2 = b.y + b.h / 2.f;

  const float ix1 = std::max(ax1, bx1), iy1 = std::max(ay1, by1);
  const float ix2 = std::min(ax2, bx2), iy2 = std::min(ay2, by2);
  const float inter = std::max(0.f, ix2 - ix1) * std::max(0.f, iy2 - iy1);
  const float ua = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - inter;
  return ua > 0.f ? inter / ua : 0.f;
}

static std::vector<Detection> nms(std::vector<Detection> dets, float iou_threshold) {
  std::sort(dets.begin(), dets.end(), [](const Detection& a, const Detection& b) {
    return a.score > b.score;
  });

  std::vector<bool> suppressed(dets.size(), false);
  std::vector<Detection> result;

  for (size_t i = 0; i < dets.size(); ++i) {
    if (suppressed[i]) continue;
    result.push_back(dets[i]);
    for (size_t j = i + 1; j < dets.size(); ++j) {
      if (!suppressed[j] && dets[i].class_id == dets[j].class_id
          && iou(dets[i], dets[j]) > iou_threshold) {
        suppressed[j] = true;
      }
    }
  }
  return result;
}

std::vector<Detection> postprocess(const float* output,
                                   int num_classes,
                                   int num_anchors,
                                   float score_threshold,
                                   float iou_threshold,
                                   const cv::Size& orig_size,
                                   int input_size) {
  // YOLOv8 输出 layout: [1, num_classes+4, num_anchors]
  // 内存排列: output[row * num_anchors + col]
  // row 0-3: cx, cy, w, h（letterbox 空间，像素）
  // row 4...: class score（已是 sigmoid 后的值）

  // 计算 letterbox 参数（与预处理对应）
  const float scale = std::min(static_cast<float>(input_size) / orig_size.width,
                               static_cast<float>(input_size) / orig_size.height);
  const int pad_w = input_size - static_cast<int>(std::round(orig_size.width * scale));
  const int pad_h = input_size - static_cast<int>(std::round(orig_size.height * scale));
  const float pad_left = pad_w / 2.f;
  const float pad_top = pad_h / 2.f;

  std::vector<Detection> candidates;
  candidates.reserve(512);

  for (int a = 0; a < num_anchors; ++a) {
    // 找最大 class score
    float max_score = 0.f;
    int best_cls = 0;
    for (int c = 0; c < num_classes; ++c) {
      const float s = output[(4 + c) * num_anchors + a];
      if (s > max_score) {
        max_score = s;
        best_cls = c;
      }
    }
    if (max_score < score_threshold) continue;

    const float cx = output[0 * num_anchors + a];
    const float cy = output[1 * num_anchors + a];
    const float bw = output[2 * num_anchors + a];
    const float bh = output[3 * num_anchors + a];

    // letterbox → 原图坐标
    const float orig_cx = (cx - pad_left) / scale;
    const float orig_cy = (cy - pad_top) / scale;
    const float orig_bw = bw / scale;
    const float orig_bh = bh / scale;

    Detection det;
    det.x = orig_cx;
    det.y = orig_cy;
    det.w = orig_bw;
    det.h = orig_bh;
    det.score = max_score;
    det.class_id = best_cls;
    candidates.push_back(det);
  }

  return nms(std::move(candidates), iou_threshold);
}

}  // namespace yolov8

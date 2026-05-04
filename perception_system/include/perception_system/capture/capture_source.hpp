#pragma once

#include <cstdint>
#include <opencv2/core.hpp>
#include <optional>
#include <string>

extern "C" {
typedef struct AVFormatContext AVFormatContext;
typedef struct AVCodecContext AVCodecContext;
typedef struct AVFrame AVFrame;
typedef struct AVPacket AVPacket;
typedef struct SwsContext SwsContext;
}

namespace perception_system::capture {

enum class PixelFormat { kBgr24 };

enum class CaptureSourceType { kFile };

struct CaptureConfig {
  std::string input;
};

struct CaptureInfo {
  CaptureSourceType source_type{CaptureSourceType::kFile};
  std::string source_name;
  int width{0};
  int height{0};
  double fps{0.0};
  PixelFormat pixel_format{PixelFormat::kBgr24};
  bool is_live{false};
  std::optional<double> duration_seconds;
  std::optional<std::int64_t> frame_count;
  bool frame_count_is_estimated{false};
};

struct Frame {
  cv::Mat image_bgr;
  int width{0};
  int height{0};
  std::int64_t timestamp_ns{0};
  std::uint64_t frame_index{0};
  std::string source_name;
};

class CaptureSource {
 public:
  CaptureSource() = default;
  ~CaptureSource();

  CaptureSource(const CaptureSource&) = delete;
  CaptureSource& operator=(const CaptureSource&) = delete;

  void Initialize(const CaptureConfig& config);
  void StartCapture();
  void StopCapture();
  void Shutdown();
  bool GrabFrameInto(cv::Mat& output_buffer, Frame& frame);
  const CaptureInfo& GetInfo() const;

 private:
  bool ReceiveFrame(cv::Mat& frame_bgr);

  AVFormatContext* format_context_{nullptr};
  AVCodecContext* codec_context_{nullptr};
  AVFrame* frame_{nullptr};
  AVPacket* packet_{nullptr};
  SwsContext* sws_context_{nullptr};
  int video_stream_index_{-1};
  bool eof_sent_{false};
  std::uint64_t next_frame_index_{0};
  CaptureInfo info_;
};

}  // namespace perception_system::capture

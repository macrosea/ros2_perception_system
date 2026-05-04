#include "perception_system/capture/capture_source.hpp"

#include <chrono>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <string>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/error.h>
#include <libswscale/swscale.h>
}

namespace perception_system::capture {

namespace {

using SteadyClock = std::chrono::steady_clock;

std::string MakeErrorString(int error_code) {
  char buffer[AV_ERROR_MAX_STRING_SIZE] = {};
  av_strerror(error_code, buffer, sizeof(buffer));
  return buffer;
}

void CheckFfmpeg(int error_code, const std::string& message) {
  if (error_code < 0) {
    throw std::runtime_error(message + ": " + MakeErrorString(error_code));
  }
}

double RationalToDouble(const AVRational value) {
  if (value.den == 0) {
    return 0.0;
  }
  return av_q2d(value);
}

std::int64_t NowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(SteadyClock::now().time_since_epoch())
      .count();
}

}  // namespace

CaptureSource::~CaptureSource() {
  Shutdown();
}

void CaptureSource::Initialize(const CaptureConfig& config) {
  if (config.input.empty()) {
    throw std::runtime_error("input video path is required");
  }

  Shutdown();

  CheckFfmpeg(avformat_open_input(&format_context_, config.input.c_str(), nullptr, nullptr),
              "failed to open video");
  CheckFfmpeg(avformat_find_stream_info(format_context_, nullptr), "failed to read stream info");

  video_stream_index_
      = av_find_best_stream(format_context_, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
  if (video_stream_index_ < 0) {
    throw std::runtime_error("failed to find video stream");
  }

  AVStream* stream = format_context_->streams[video_stream_index_];
  const AVCodecParameters* codec_parameters = stream->codecpar;

  info_ = {};
  info_.source_type = CaptureSourceType::kFile;
  info_.source_name = config.input;
  info_.width = codec_parameters->width;
  info_.height = codec_parameters->height;
  info_.fps = RationalToDouble(stream->avg_frame_rate);
  if (info_.fps <= 0.0) {
    info_.fps = RationalToDouble(stream->r_frame_rate);
  }
  info_.pixel_format = PixelFormat::kBgr24;
  info_.is_live = false;

  if (stream->duration > 0 && stream->time_base.den != 0) {
    info_.duration_seconds = stream->duration * av_q2d(stream->time_base);
  } else if (format_context_->duration > 0) {
    info_.duration_seconds = static_cast<double>(format_context_->duration) / AV_TIME_BASE;
  }

  if (stream->nb_frames > 0) {
    info_.frame_count = stream->nb_frames;
    info_.frame_count_is_estimated = false;
  } else if (info_.fps > 0.0 && info_.duration_seconds.has_value()) {
    info_.frame_count = static_cast<std::int64_t>(info_.duration_seconds.value() * info_.fps + 0.5);
    info_.frame_count_is_estimated = true;
  }

  const AVCodec* codec = avcodec_find_decoder(codec_parameters->codec_id);
  if (codec == nullptr) {
    throw std::runtime_error("failed to find video decoder");
  }

  codec_context_ = avcodec_alloc_context3(codec);
  if (codec_context_ == nullptr) {
    throw std::runtime_error("failed to allocate codec context");
  }

  CheckFfmpeg(avcodec_parameters_to_context(codec_context_, codec_parameters),
              "failed to copy codec parameters");
  CheckFfmpeg(avcodec_open2(codec_context_, codec, nullptr), "failed to open decoder");

  frame_ = av_frame_alloc();
  packet_ = av_packet_alloc();
  if (frame_ == nullptr || packet_ == nullptr) {
    throw std::runtime_error("failed to allocate ffmpeg frame or packet");
  }

  sws_context_ = sws_getContext(codec_context_->width,
                                codec_context_->height,
                                codec_context_->pix_fmt,
                                codec_context_->width,
                                codec_context_->height,
                                AV_PIX_FMT_BGR24,
                                SWS_BILINEAR,
                                nullptr,
                                nullptr,
                                nullptr);
  if (sws_context_ == nullptr) {
    throw std::runtime_error("failed to create sws context");
  }

  eof_sent_ = false;
  next_frame_index_ = 0;
}

void CaptureSource::StartCapture() {
}

void CaptureSource::StopCapture() {
}

void CaptureSource::Shutdown() {
  sws_freeContext(sws_context_);
  sws_context_ = nullptr;
  av_packet_free(&packet_);
  av_frame_free(&frame_);
  avcodec_free_context(&codec_context_);
  avformat_close_input(&format_context_);
  video_stream_index_ = -1;
  eof_sent_ = false;
  next_frame_index_ = 0;
  info_ = {};
}

bool CaptureSource::GrabFrameInto(cv::Mat& output_buffer, Frame& frame) {
  // 确保 output_buffer 已预分配
  if (output_buffer.empty() || output_buffer.cols != codec_context_->width
      || output_buffer.rows != codec_context_->height) {
    output_buffer.create(codec_context_->height, codec_context_->width, CV_8UC3);
  }
  while (true) {
    if (ReceiveFrame(output_buffer)) {  // 直接写入调用者的 buffer
      frame.image_bgr = output_buffer;  // 浅拷贝（共享指针）
      frame.frame_index = next_frame_index_++;
      frame.timestamp_ns = NowNs();
      frame.width = output_buffer.cols;
      frame.height = output_buffer.rows;
      frame.source_name = info_.source_name;
      return true;
    }

    if (eof_sent_) {
      return false;
    }

    const int read_result = av_read_frame(format_context_, packet_);
    if (read_result == AVERROR_EOF) {
      CheckFfmpeg(avcodec_send_packet(codec_context_, nullptr), "failed to flush decoder");
      eof_sent_ = true;
      continue;
    }

    CheckFfmpeg(read_result, "failed to read frame packet");

    if (packet_->stream_index != video_stream_index_) {
      av_packet_unref(packet_);
      continue;
    }

    const int send_result = avcodec_send_packet(codec_context_, packet_);
    av_packet_unref(packet_);

    if (send_result == AVERROR(EAGAIN)) {
      continue;
    }
    CheckFfmpeg(send_result, "failed to send packet to decoder");
  }
}

const CaptureInfo& CaptureSource::GetInfo() const {
  return info_;
}

bool CaptureSource::ReceiveFrame(cv::Mat& frame_bgr) {
  const int result = avcodec_receive_frame(codec_context_, frame_);
  if (result == AVERROR(EAGAIN) || result == AVERROR_EOF) {
    return false;
  }

  CheckFfmpeg(result, "failed to decode frame");

  if (frame_bgr.empty() || frame_bgr.cols != codec_context_->width
      || frame_bgr.rows != codec_context_->height || frame_bgr.type() != CV_8UC3) {
    throw std::runtime_error("ReceiveFrame: frame_bgr not properly pre-allocated");
  }

  std::uint8_t* destination_data[4] = {frame_bgr.data, nullptr, nullptr, nullptr};
  int destination_linesize[4] = {static_cast<int>(frame_bgr.step[0]), 0, 0, 0};

  const int scaled_height = sws_scale(sws_context_,
                                      frame_->data,
                                      frame_->linesize,
                                      0,
                                      codec_context_->height,
                                      destination_data,
                                      destination_linesize);
  av_frame_unref(frame_);

  if (scaled_height != codec_context_->height) {
    throw std::runtime_error("failed to convert frame to BGR");
  }
  return true;
}

}  // namespace perception_system::capture

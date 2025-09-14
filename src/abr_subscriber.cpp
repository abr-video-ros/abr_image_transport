/*********************************************************************
* Copyright 2025 José Miguel Guerrero Hernández
*
* This file is part of abr-video-ros (Adaptative Bitrate Video for ROS 2).
*
* abr-video-ros is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* abr-video-ros is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with program.  If not, see <https://www.gnu.org/licenses/>.
*********************************************************************/

/// \file
/// \brief Implementation of the AbrSubscriber class for adaptive bitrate image transport in ROS 2.

#include "abr_image_transport/abr_subscriber.hpp"

#include <rclcpp/logging.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
#include <libavutil/hwcontext.h>
#include <libavformat/avformat.h>
}

namespace abr_image_transport
{

// ============================================================================
// AdaptiveQualityMonitor Implementation
// ============================================================================

AdaptiveQualityMonitor::AdaptiveQualityMonitor(rclcpp::Node::SharedPtr node)
: node_(std::move(node))
{
  loadDefaultYaml();
  // Create service client for updating encoder parameters
  client_ = node_->create_client<abr_image_transport_interfaces::srv::AbrParams>("abr_params");
}

void AdaptiveQualityMonitor::loadDefaultYaml()
{
  // Load configuration file from package share
  const std::string yaml_file =
    ament_index_cpp::get_package_share_directory("abr_image_transport") + "/params/abr_config.yaml";

  try {
    YAML::Node cfg = YAML::LoadFile(yaml_file);
    if (cfg["abr_subscriber"]) {
      auto y = cfg["abr_subscriber"];
      min_bitrate_safe_ = y["min_bitrate_safe"] ? y["min_bitrate_safe"].as<int>() : 100000;
      max_bitrate_safe_ = y["max_bitrate_safe"] ? y["max_bitrate_safe"].as<int>() : 2000000;
      fps_variance_threshold_ =
        y["fps_variance_threshold"] ? y["fps_variance_threshold"].as<double>() : 2.0;
      increase_factor_ = y["increase_factor"] ? y["increase_factor"].as<double>() : 1.5;
      decrease_factor_ = y["decrease_factor"] ? y["decrease_factor"].as<double>() : 0.75;
      stable_window_ = y["stable_window"] ? y["stable_window"].as<int>() : 10;
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(node_->get_logger(), "Could not load YAML config: %s", e.what());
  }

  RCLCPP_INFO(node_->get_logger(), "Configuration loaded from: %s", yaml_file.c_str());
  RCLCPP_INFO(
    node_->get_logger(),
    "min_bitrate_safe=%d max_bitrate_safe=%d fps_variance_threshold=%.2f "
    "increase_factor=%.2f decrease_factor=%.2f stable_window=%d",
    min_bitrate_safe_, max_bitrate_safe_, fps_variance_threshold_,
    increase_factor_, decrease_factor_, (int)stable_window_);
}

void AdaptiveQualityMonitor::updateAndAdapt(size_t packet_size)
{
  using namespace std::chrono;
  auto now = steady_clock::now();

  // Store history of timestamps for stability analysis
  timestamps_.push_back(now);
  if (timestamps_.size() > static_cast<size_t>(stable_window_)) {
    timestamps_.pop_front();
  }

  // Calculate FPS values based on inter-frame time differences
  std::vector<double> fps_window;
  for (size_t i = 1; i < timestamps_.size(); ++i) {
    double dt = duration_cast<milliseconds>(timestamps_[i] - timestamps_[i - 1]).count() / 1000.0;
    if (dt > 0.0) {
      fps_window.push_back(1.0 / dt);
    }
  }

  // Compute min, max, and average FPS
  double fps_avg = 0.0, fps_min = 0.0, fps_max = 0.0;
  if (!fps_window.empty()) {
    fps_min = fps_max = fps_window.front();
    for (double f : fps_window) {
      fps_avg += f;
      fps_min = std::min(fps_min, f);
      fps_max = std::max(fps_max, f);
    }
    fps_avg /= fps_window.size();
  }
  
  // Standard deviation
  double variance = 0.0;
  for (double f : fps_window) {
    variance += (f - fps_avg) * (f - fps_avg);
  }
  variance /= fps_window.size();
  double stddev = std::sqrt(variance);

  // Relative FPS variance (normalized instability measure)
  // double fps_var_rel = (fps_max - fps_min) / (fps_avg + 1e-6);

  // Relative variance (instability metric)
  double fps_var_rel = stddev / (fps_avg + 1e-6);

  // Decide on bitrate adjustment
  double desired_bitrate = current_bitrate_;
  if (fps_var_rel > fps_variance_threshold_) {
    // Stream unstable, decrease bitrate
    desired_bitrate = std::max(desired_bitrate * decrease_factor_,
                               static_cast<double>(min_bitrate_safe_));
    stable_count_ = 0;
  } else {
    // Stream stable, gradually increase bitrate
    stable_count_++;
    if (stable_count_ >= stable_window_) {
      desired_bitrate = std::min(desired_bitrate * increase_factor_,
                                 static_cast<double>(max_bitrate_safe_));
      stable_count_ = 0;
    }
  }

  RCLCPP_INFO(
        node_->get_logger(),
        "ABR monitor: pkt_size=%zu fps_avg=%.1f fps_var=%.2f bitrate=%d → %d",
        packet_size, fps_avg, fps_var_rel,
        static_cast<int>(current_bitrate_), static_cast<int>(desired_bitrate));
        
  // Apply changes if bitrate changed
  if (desired_bitrate != current_bitrate_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "ABR monitor: pkt_size=%zu fps_avg=%.1f fps_var=%.2f bitrate=%d → %d",
      packet_size, fps_avg, fps_var_rel,
      static_cast<int>(current_bitrate_), static_cast<int>(desired_bitrate));

    current_bitrate_ = desired_bitrate;
    sendRequest();
  }
}

void AdaptiveQualityMonitor::sendRequest()
{
  // Wait for service to be ready
  if (!client_->service_is_ready()) {
    return;
  }

  // Send request
  auto req = std::make_shared<abr_image_transport_interfaces::srv::AbrParams::Request>();
  req->bitrate = static_cast<int>(current_bitrate_);
  client_->async_send_request(req);

  RCLCPP_INFO(node_->get_logger(), "ABR service request sent: bitrate=%d",
              static_cast<int>(current_bitrate_));
}


// ============================================================================
// AbrSubscriber Implementation
// ============================================================================

AbrSubscriber::AbrSubscriber()
: logger_(rclcpp::get_logger("AbrSubscriber"))
{
  loadDefaultYaml();
  //logAvailableDecoders(); // Log available decoders for debugging
}

AbrSubscriber::~AbrSubscriber()
{
  closeDecoder();
}

void AbrSubscriber::loadDefaultYaml()
{
  // Load ABR subscriber configuration from YAML
  const std::string yaml_file =
    ament_index_cpp::get_package_share_directory("abr_image_transport") + "/params/abr_config.yaml";

  try {
    YAML::Node cfg = YAML::LoadFile(yaml_file);
    if (cfg["abr_subscriber"]) {
      auto y = cfg["abr_subscriber"];
      use_buffer_ = y["use_buffer"] ? y["use_buffer"].as<bool>() : false;
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_, "Could not load YAML: %s", e.what());
  }
}

void AbrSubscriber::logAvailableDecoders()
{
  // Enumerate and log available FFmpeg decoders
  void * it = nullptr;
  const AVCodec * codec = nullptr;
  RCLCPP_INFO(logger_, "=== Available FFmpeg decoders ===");
  while ((codec = av_codec_iterate(&it))) {
    if (av_codec_is_decoder(codec)) {
      RCLCPP_INFO(logger_, "Decoder available: %s (id=%d)", codec->name, codec->id);
    }
  }
}

void AbrSubscriber::resetSwsContext()
{
  // Free scaling context
  if (sws_ctx_) {
    sws_freeContext(sws_ctx_);
    sws_ctx_ = nullptr;
  }
  sws_src_pix_fmt_ = -1;
}

bool AbrSubscriber::initDecoder(
  const std::string & encoding,
  unsigned int width,
  unsigned int height)
{
  // Close existing decoder if any
  closeDecoder();

  // Try to find requested decoder, fallback to H.264 if unavailable
  const AVCodec * codec = avcodec_find_decoder_by_name(encoding.c_str());
  if (!codec) {
    codec = avcodec_find_decoder_by_name("h264");
    if (!codec) {
      RCLCPP_ERROR(logger_, "Decoder '%s' not found, and fallback 'h264' unavailable",
                   encoding.c_str());
      return false;
    } else {
      RCLCPP_WARN(logger_, "Decoder '%s' not found. Using fallback 'h264'", encoding.c_str());
    }
  }

  // Allocate codec context
  codec_ctx_raw_ = avcodec_alloc_context3(codec);
  if (!codec_ctx_raw_) {
    RCLCPP_ERROR(logger_, "Failed to allocate AVCodecContext");
    return false;
  }

  // Configure codec
  codec_ctx_raw_->width = width;
  codec_ctx_raw_->height = height;
  codec_ctx_raw_->thread_count = 0;  // auto-detect
  codec_ctx_raw_->flags2 |= AV_CODEC_FLAG2_FAST;

  // Open codec
  if (avcodec_open2(codec_ctx_raw_, codec, nullptr) < 0) {
    RCLCPP_ERROR(logger_, "Failed to open codec: %s", encoding.c_str());
    return false;
  }

  // Create parser
  parser_raw_ = av_parser_init(codec->id);
  if (!parser_raw_) {
    RCLCPP_WARN(logger_, "Parser could not be created (decoding may still work)");
  }

  // Create scaling context
  sws_ctx_ = nullptr;
  sws_src_pix_fmt_ = -1;

  RCLCPP_INFO(logger_, "Decoder initialized: %s (%dx%d)",
              codec->name, width, height);
  return true;
}

void AbrSubscriber::closeDecoder()
{
  // Close existing decoder
  if (parser_raw_) {
    av_parser_close(parser_raw_);
    parser_raw_ = nullptr;
  }

  // Free codec context
  if (codec_ctx_raw_) {
    avcodec_free_context(&codec_ctx_raw_);
    codec_ctx_raw_ = nullptr;
  }

  // Free scaling context
  resetSwsContext();
}

void AbrSubscriber::subscribeImpl(
  rclcpp::Node * node,
  const std::string & base_topic,
  const Callback & callback,
  rmw_qos_profile_t custom_qos,
  rclcpp::SubscriptionOptions options)
{
  logger_ = node->get_logger();
  node_ptr_ = node->shared_from_this();

  using Base = image_transport::SimpleSubscriberPlugin<abr_image_transport_interfaces::msg::AbrImage>;
  Base::subscribeImpl(node, base_topic, callback, custom_qos, options);
}

// Smart Pointer Deleters for unique_ptr (used for automatic resource management)
namespace
{
struct AVFrameDeleter  { void operator()(AVFrame * f)  const {av_frame_free(&f);} };
struct AVPacketDeleter { void operator()(AVPacket * p) const {av_packet_free(&p);} };
struct SwsDeleter      { void operator()(SwsContext * s) const {sws_freeContext(s);} };

using FramePtr = std::unique_ptr<AVFrame, AVFrameDeleter>;
using PacketPtr = std::unique_ptr<AVPacket, AVPacketDeleter>;
using SwsPtr = std::unique_ptr<SwsContext, SwsDeleter>;
} // namespace

void AbrSubscriber::publishFrame(
  AVCodecContext * ctx,
  AVFrame * frame,
  const std_msgs::msg::Header & header,
  const Callback & callback,
  SwsContext *& sws_ctx,
  int & last_pix_fmt)
{
  // Create output image
  cv::Mat img(cv::Size(ctx->width, ctx->height), CV_8UC3);

  // Re-create conversion context if pixel format changed
  if (!sws_ctx || last_pix_fmt != frame->format) {
    if (sws_ctx) {
      sws_freeContext(sws_ctx);
    }
    sws_ctx = sws_getContext(
      ctx->width, ctx->height,
      static_cast<AVPixelFormat>(frame->format),
      ctx->width, ctx->height,
      AV_PIX_FMT_BGR24,
      SWS_BILINEAR, nullptr, nullptr, nullptr);
    last_pix_fmt = frame->format;
  }

  if (!sws_ctx) {
    RCLCPP_ERROR(logger_, "Failed to create SwsContext for format %d", frame->format);
    return;
  }

  // Convert to BGR format
  uint8_t * dest[4] = {img.data, nullptr, nullptr, nullptr};
  int dest_linesize[4] = {static_cast<int>(img.step), 0, 0, 0};
  sws_scale(sws_ctx, frame->data, frame->linesize, 0, ctx->height, dest, dest_linesize);

  auto ros_img = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();

  // Call user callback with decoded image
  callback(ros_img);
}

void AbrSubscriber::internalCallback(
  const abr_image_transport_interfaces::msg::AbrImage::ConstSharedPtr & message,
  const Callback & callback)
{
  // ABR monitor
  if (!monitor_) {
    monitor_ = std::make_unique<AdaptiveQualityMonitor>(node_ptr_);
  }

  // Update monitor with new packet size
  monitor_->updateAndAdapt(message->compressed_data.size());

  // Decoder context
  AVCodecContext * ctx = nullptr;
  AVCodecParserContext * parser = nullptr;
  SwsContext * local_sws_ctx = nullptr;   // used only in non-buffer mode
  int local_pix_fmt = -1;
  bool free_ctx = false;

  if (use_buffer_) {
    // Persistent decoder
    if (!codec_ctx_raw_) {
      if (!initDecoder(message->encoding, message->original_width, message->original_height)) {
        RCLCPP_ERROR(logger_, "Failed to initialize decoder for %s", message->encoding.c_str());
        return;
      }
    }
    ctx = codec_ctx_raw_;
    parser = parser_raw_;

  } else {
    // Create temporary decoder for each message
    const AVCodec * codec = avcodec_find_decoder_by_name(message->encoding.c_str());
    if (!codec) {
      RCLCPP_ERROR(logger_, "Codec not found: %s", message->encoding.c_str());
      return;
    }
    ctx = avcodec_alloc_context3(codec);
    if (!ctx) {return;}
    ctx->width = message->original_width;
    ctx->height = message->original_height;
    if (avcodec_open2(ctx, codec, nullptr) < 0) {
      RCLCPP_ERROR(logger_, "Could not open codec");
      avcodec_free_context(&ctx);
      return;
    }
    free_ctx = true;
  }

  // Input data buffer
  uint8_t * data = const_cast<uint8_t *>(message->compressed_data.data());
  int size = static_cast<int>(message->compressed_data.size());

  // Decode loop
  while (size > 0) {
    uint8_t * parsed_data = nullptr;
    int parsed_size = 0;

    // Parse input data if parser is available
    int len = parser ?
      av_parser_parse2(parser, ctx, &parsed_data, &parsed_size,
                         data, size, AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0) :
      size;

    if (len < 0) {
      RCLCPP_ERROR(logger_, "Error while parsing compressed data");
      break;
    }
    if (!parser) {parsed_data = data; parsed_size = size;}

    data += len;
    size -= len;
    if (parsed_size == 0) {continue;}

    // Send parsed packet to decoder
    PacketPtr pkt(av_packet_alloc());
    av_new_packet(pkt.get(), parsed_size);
    memcpy(pkt->data, parsed_data, parsed_size);

    if (avcodec_send_packet(ctx, pkt.get()) < 0) {
      RCLCPP_ERROR(logger_, "Error sending packet to decoder");
      continue;
    }

    // Receive all available frames
    FramePtr frame(av_frame_alloc());
    while (true) {
      int ret = avcodec_receive_frame(ctx, frame.get());
      if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {break;}
      if (ret < 0) {
        RCLCPP_ERROR(logger_, "Error during decoding");
        break;
      }

      // Convert and publish
      if (use_buffer_) {
        publishFrame(ctx, frame.get(), message->header, callback, sws_ctx_, sws_src_pix_fmt_);
      } else {
        publishFrame(ctx, frame.get(), message->header, callback, local_sws_ctx, local_pix_fmt);
      }
    }
  }

  // Cleanup
  if (!use_buffer_) {
    if (local_sws_ctx) {sws_freeContext(local_sws_ctx);}
    if (free_ctx) {avcodec_free_context(&ctx);}
  }
}

}  // namespace abr_image_transport

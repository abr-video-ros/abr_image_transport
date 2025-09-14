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
/// \brief Implementation of the AbrPublisher class for adaptive bitrate image transport in ROS 2.

#include "abr_image_transport/abr_publisher.hpp"

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

extern "C" {
#include <libswscale/swscale.h>
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
}

namespace abr_image_transport
{

AbrPublisher::AbrPublisher()
: logger_(rclcpp::get_logger("AbrPublisher")) {}

AbrPublisher::~AbrPublisher()
{
  closeEncoder();
}

void AbrPublisher::advertiseImpl(
  rclcpp::Node * node,
  const std::string & base_topic,
  rmw_qos_profile_t custom_qos,
  rclcpp::PublisherOptions options)
{
  loadDefaultYaml();
  logger_ = node->get_logger();

  using Base =
    image_transport::SimplePublisherPlugin<abr_image_transport_interfaces::msg::AbrImage>;
  Base::advertiseImpl(node, base_topic, custom_qos, options);

  // Service for updating ABR encoding parameters dynamically
  srv_ = node->create_service<abr_image_transport_interfaces::srv::AbrParams>(
    "abr_params",
    std::bind(&AbrPublisher::setAbrParams, this, std::placeholders::_1, std::placeholders::_2));
}

void AbrPublisher::loadDefaultYaml()
{
  std::string yaml_file =
    ament_index_cpp::get_package_share_directory("abr_image_transport") +
    "/params/abr_config.yaml";

  try {
    YAML::Node cfg = YAML::LoadFile(yaml_file);
    if (cfg["abr_publisher"]) {
      auto y = cfg["abr_publisher"];
      codec_ = y["codec"] ? y["codec"].as<std::string>() : "libx264";
      bitrate_ = y["bitrate"] ? y["bitrate"].as<int>() : 400000;
      fps_ = y["fps"] ? y["fps"].as<int>() : 25;
      preset_ = y["preset"] ? y["preset"].as<std::string>() : "ultrafast";
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_, "Could not load YAML: %s", e.what());
  }

  RCLCPP_INFO(
    logger_,
    "Configuration loaded from: %s", yaml_file.c_str());
  RCLCPP_INFO(
    logger_,
    "Using codec=%s bitrate=%d fps=%d preset=%s",
    codec_.c_str(), bitrate_, fps_, preset_.c_str());
}

void AbrPublisher::setAbrParams(
  const std::shared_ptr<abr_image_transport_interfaces::srv::AbrParams::Request> req,
  std::shared_ptr<abr_image_transport_interfaces::srv::AbrParams::Response> res)
{
  if (bitrate_ == req->bitrate) {
    res->success = true;
    res->message = "Bitrate unchanged";
    return;
  }

  // Update bitrate
  bitrate_ = req->bitrate;

  // Flush and reset encoder if already initialized
  if (codec_ctx_) {
    avcodec_send_frame(codec_ctx_, nullptr);
    while (avcodec_receive_packet(codec_ctx_, pkt_) == 0) {
      av_packet_unref(pkt_);
    }
    closeEncoder();
  }

  RCLCPP_INFO(
    logger_,
    "Updated parameters: codec=%s bitrate=%d fps=%d preset=%s",
    codec_.c_str(), bitrate_, fps_, preset_.c_str());

  res->success = true;
  res->message = "Parameters updated";
}

void AbrPublisher::initEncoder(int width, int height)
{
  // Find encoder
  const AVCodec * codec = avcodec_find_encoder_by_name(codec_.c_str());
  if (!codec) {
    throw std::runtime_error("Codec not found: " + codec_);
  }

  // Allocate codec context
  codec_ctx_ = avcodec_alloc_context3(codec);
  if (!codec_ctx_) {
    throw std::runtime_error("Failed to allocate AVCodecContext");
  }

  // Configure encoder context
  codec_ctx_->width = width;
  codec_ctx_->height = height;
  codec_ctx_->time_base = {1, fps_};
  codec_ctx_->framerate = {fps_, 1};
  codec_ctx_->gop_size = 1;       // Intra-frame only
  codec_ctx_->max_b_frames = 0;   // No B-frames (low latency)
  codec_ctx_->pix_fmt = AV_PIX_FMT_YUV420P;

  // Encoder options
  AVDictionary * opts = nullptr;
  av_dict_set(&opts, "preset", preset_.c_str(), 0);
  av_dict_set(&opts, "tune", "zerolatency", 0);
  av_dict_set(&opts, "repeat_headers", "1", 0);   // Adds SPS/PPS to every keyframe
  if (codec_ == "libx264" || codec_ == "h264_nvenc") {
    av_dict_set(&opts, "profile", "baseline", 0);
  }

  // Open codec
  if (avcodec_open2(codec_ctx_, codec, &opts) < 0) {
    av_dict_free(&opts);
    throw std::runtime_error("Could not open codec");
  }
  av_dict_free(&opts);

  // Allocate frame buffer
  frame_ = av_frame_alloc();
  frame_->format = codec_ctx_->pix_fmt;
  frame_->width = codec_ctx_->width;
  frame_->height = codec_ctx_->height;
  av_frame_get_buffer(frame_, 32);

  // Allocate packet buffer
  pkt_ = av_packet_alloc();

  // Setup scaling/conversion context (BGR to YUV420P)
  sws_ctx_ = sws_getContext(
    width, height, AV_PIX_FMT_BGR24,
    codec_ctx_->width, codec_ctx_->height, codec_ctx_->pix_fmt,
    SWS_BILINEAR, nullptr, nullptr, nullptr);

  // Reset frame index
  frame_index_ = 0;
}

void AbrPublisher::closeEncoder()
{
  // Flush encoder
  if (codec_ctx_) {
    avcodec_send_frame(codec_ctx_, nullptr);
    while (avcodec_receive_packet(codec_ctx_, pkt_) == 0) {
      av_packet_unref(pkt_);
    }
  }

  // Free resources
  if (pkt_) {
    av_packet_free(&pkt_);
    pkt_ = nullptr;
  }

  // Free frame
  if (frame_) {
    av_frame_free(&frame_);
    frame_ = nullptr;
  }

  // Free scaling context
  if (sws_ctx_) {
    sws_freeContext(sws_ctx_);
    sws_ctx_ = nullptr;
  }

  // Free codec context
  if (codec_ctx_) {
    avcodec_free_context(&codec_ctx_);
    codec_ctx_ = nullptr;
  }
}

void AbrPublisher::publish(
  const sensor_msgs::msg::Image & message,
  const PublishFn & publish_fn) const
{
  // Convert ROS Image to OpenCV
  cv_bridge::CvImagePtr image_ptr;
  try {
    image_ptr = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(logger_, "cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat cv_image = image_ptr->image;

  // Initialize encoder on first frame
  if (!codec_ctx_) {
    const_cast<AbrPublisher *>(this)->initEncoder(cv_image.cols, cv_image.rows);
  }

  // Convert BGR to YUV420P
  uint8_t * src_data[4] = {cv_image.data, nullptr, nullptr, nullptr};
  int src_linesize[4] = {static_cast<int>(cv_image.step), 0, 0, 0};
  sws_scale(
    sws_ctx_, src_data, src_linesize,
    0, cv_image.rows, frame_->data, frame_->linesize);

  // Set PTS
  frame_->pts = frame_index_++;

  // Encode frame
  if (avcodec_send_frame(codec_ctx_, frame_) < 0) {
    RCLCPP_ERROR(logger_, "Error sending frame to encoder");
    return;
  }

  // Retrieve encoded frame
  int ret = avcodec_receive_packet(codec_ctx_, pkt_);
  if (ret == 0) {
    abr_image_transport_interfaces::msg::AbrImage abr_msg;
    abr_msg.header = message.header;
    abr_msg.original_width = cv_image.cols;
    abr_msg.original_height = cv_image.rows;

    // Map codec to encoding string
    if (codec_ == "libx264") {
      abr_msg.encoding = "h264";
    } else if (codec_ == "hevc") {
      abr_msg.encoding = "hevc";
    } else {
      abr_msg.encoding = codec_;
    }

    // Copy compressed data
    abr_msg.compressed_data.assign(pkt_->data, pkt_->data + pkt_->size);

    // Publish encoded message
    publish_fn(abr_msg);

    // Free packet
    av_packet_unref(pkt_);

  } else if (ret != AVERROR(EAGAIN) && ret != AVERROR_EOF) {
    RCLCPP_ERROR(logger_, "Error receiving packet from encoder");
  }
}

}  // namespace abr_image_transport

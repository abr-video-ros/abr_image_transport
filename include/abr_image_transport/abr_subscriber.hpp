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
/// \brief Subscriber plugin for adaptive bitrate (ABR) image transport.

#pragma once

#include <memory>
#include <string>
#include <deque>
#include <mutex>
#include <chrono>

#include <image_transport/simple_subscriber_plugin.hpp>
#include <abr_image_transport_interfaces/msg/abr_image.hpp>
#include <abr_image_transport_interfaces/srv/abr_params.hpp>

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

// FFmpeg forward declarations (C headers, so wrap in extern "C")
extern "C" {
struct AVCodecContext;
struct AVFrame;
struct AVPacket;
struct SwsContext;
struct AVCodecParserContext;
struct AVBufferRef;
}

namespace abr_image_transport
{

/**
 * @class AdaptiveQualityMonitor
 * @brief Monitors FPS and packet sizes to dynamically adjust encoding bitrate.
 *
 * Tracks packet sizes and timing, applies heuristics to increase or decrease
 * bitrate safely, and sends parameter updates via ROS 2 service calls.
 */
class AdaptiveQualityMonitor
{
public:
  /**
   * @brief Constructor.
   * @param node Owning ROS 2 node.
   */
  explicit AdaptiveQualityMonitor(rclcpp::Node::SharedPtr node);

  /**
   * @brief Update monitor state with new packet size and adapt bitrate if needed.
   * @param packet_size Size of the received packet in bytes.
   */
  void updateAndAdapt(size_t packet_size);

private:
  /// @brief Load default decoding parameters from a YAML file.
  void loadDefaultYaml();

  /// @brief Send updated bitrate parameters via ROS service.
  void sendRequest();

  // ---------------------------------------------------------------------------
  // ROS handles.
  // ---------------------------------------------------------------------------
  rclcpp::Node::SharedPtr node_;  ///< Node for logging and service calls.
  rclcpp::Client<abr_image_transport_interfaces::srv::AbrParams>::SharedPtr client_;  ///< Service client for updating encoder parameters.

  // ---------------------------------------------------------------------------
  // Monitoring state.
  // ---------------------------------------------------------------------------
  std::deque<double> size_history_;  ///< History of recent packet sizes (bytes).
  std::deque<std::chrono::steady_clock::time_point> timestamps_;  ///< Timestamps of recent packets for FPS estimation.

  // ---------------------------------------------------------------------------
  // Bitrate adaptation parameters.
  // ---------------------------------------------------------------------------
  int current_bitrate_ = 400000;        ///< Current bitrate (bps).
  int min_bitrate_safe_ = 100000;       ///< Minimum safe bitrate (bps).
  int max_bitrate_safe_ = 2000000;      ///< Maximum safe bitrate (bps).

  size_t stable_count_ = 0;             ///< Consecutive stable frames counter.
  size_t stable_window_ = 10;           ///< Frames needed before increasing bitrate.

  double fps_variance_threshold_ = 2.0; ///< FPS variance threshold to detect instability.
  double increase_factor_ = 1.5;        ///< Multiplicative factor for increasing bitrate.
  double decrease_factor_ = 0.7;        ///< Multiplicative factor for decreasing bitrate.
};

/**
 * @class AbrSubscriber
 * @brief Subscriber plugin for adaptive bitrate (ABR) image transport.
 *
 * This class receives abr_image_transport_interfaces::msg::AbrImage messages,
 * decodes them using FFmpeg, and publishes the resulting sensor_msgs::msg::Image.
 */
class AbrSubscriber
  : public image_transport::SimpleSubscriberPlugin<
    abr_image_transport_interfaces::msg::AbrImage>
{
public:
  /// @brief Constructor.
  AbrSubscriber();

  /// @brief Destructor (ensures decoder cleanup).
  ~AbrSubscriber() override;

  /**
   * @brief Return the transport name identifier (used by image_transport).
   *
   * @return "abr"
   */
  std::string getTransportName() const override {return "abr";}

protected:
  /**
   * @brief Subscribe implementation.
   * @param node Owning ROS 2 node.
   * @param base_topic Base topic name.
   * @param callback Callback to deliver decoded images.
   * @param custom_qos QoS profile for the subscription.
   * @param options Subscription options.
   */
  void subscribeImpl(
    rclcpp::Node * node,
    const std::string & base_topic,
    const Callback & callback,
    rmw_qos_profile_t custom_qos,
    rclcpp::SubscriptionOptions options) override;

  /**
   * @brief Internal callback to handle received AbrImage messages.
   * @param message Encoded ABR image message.
   * @param callback Callback to deliver decoded sensor_msgs::msg::Image.
   */
  void internalCallback(
    const abr_image_transport_interfaces::msg::AbrImage::ConstSharedPtr & message,
    const Callback & callback) override;

private:
  /**
   * @brief Initialize the decoder for a given encoding and resolution.
   * @param encoding Image encoding (e.g., "h264").
   * @param width Frame width.
   * @param height Frame height.
   * @return True if initialization succeeds, false otherwise.
   */
  bool initDecoder(const std::string & encoding, unsigned int width, unsigned int height);

  /// @brief Close and release decoder resources.
  void closeDecoder();

  /// @brief Log all available FFmpeg decoders for debugging.
  void logAvailableDecoders();

  /// @brief Reset the SwsContext for pixel format conversion.
  void resetSwsContext();

  /// @brief Load default subscriber parameters from YAML file.
  void loadDefaultYaml();

  /**
   * @brief Decode and publish a single frame.
   * @param ctx Decoder context.
   * @param frame Decoded frame.
   * @param header Original message header for timestamping.
   * @param callback User callback to deliver decoded image.
   * @param sws_ctx SwsContext for pixel format conversion (may be updated).
   * @param last_pix_fmt Last pixel format used (to detect changes).
   */
  void publishFrame(
    AVCodecContext * ctx,
    AVFrame * frame,
    const std_msgs::msg::Header & header,
    const Callback & callback,
    SwsContext *& sws_ctx, int & last_pix_fmt);

  // ---------------------------------------------------------------------------
  // Decoder contexts (managed by FFmpeg).
  // ---------------------------------------------------------------------------
  AVCodecContext * codec_ctx_raw_ = nullptr;       ///< Raw codec context.
  AVCodecParserContext * parser_raw_ = nullptr;    ///< Codec parser context.
  SwsContext * sws_ctx_ = nullptr;                 ///< Scaling/conversion context.
  int sws_src_pix_fmt_ = -1;                       ///< Source pixel format.
  bool use_buffer_ = false;                        ///< If true, buffer messages for decoding.

  // ---------------------------------------------------------------------------
  // ROS handles.
  // ---------------------------------------------------------------------------
  rclcpp::Logger logger_;              ///< Logger for diagnostics.
  rclcpp::Node::SharedPtr node_ptr_;   ///< Node pointer for subscriptions.

  // ---------------------------------------------------------------------------
  // Adaptive bitrate monitor.
  // ---------------------------------------------------------------------------
  std::unique_ptr<AdaptiveQualityMonitor> monitor_; ///< Monitors quality and adapts bitrate.
};

}  // namespace abr_image_transport

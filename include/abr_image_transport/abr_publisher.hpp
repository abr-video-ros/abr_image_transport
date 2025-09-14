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
/// \brief Publisher plugin for adaptive bitrate (ABR) image transport.

#pragma once

#include <memory>
#include <string>

#include <sensor_msgs/msg/image.hpp>
#include <image_transport/simple_publisher_plugin.hpp>
#include <abr_image_transport_interfaces/msg/abr_image.hpp>
#include <abr_image_transport_interfaces/srv/abr_params.hpp>

// FFmpeg forward declarations (C headers, so wrap in extern "C")
extern "C" {
struct AVCodecContext;
struct AVFrame;
struct AVPacket;
struct SwsContext;
struct AVBufferRef;
}

namespace abr_image_transport
{

/**
 * @class AbrPublisher
 * @brief Publisher plugin for adaptive bitrate (ABR) image transport.
 *
 * This class encodes images into an ABR-compressed format using FFmpeg,
 * and publishes them as abr_image_transport_interfaces::msg::AbrImage messages.
 */
class AbrPublisher
  : public image_transport::SimplePublisherPlugin<
    abr_image_transport_interfaces::msg::AbrImage>
{
public:
  /// @brief Constructor.
  AbrPublisher();

  /// @brief Destructor (ensures encoder resources are freed).
  ~AbrPublisher() override;

  /**
   * @brief Return the transport name identifier (used by image_transport).
   *
   * @return "abr"
   */
  std::string getTransportName() const override {return "abr";}

protected:
  /**
   * @brief Advertise the publisher with given QoS and options.
   * @param node Owning ROS 2 node.
   * @param base_topic Base topic name to publish under.
   * @param custom_qos QoS profile for the publisher.
   * @param options Publisher options.
   */
  void advertiseImpl(
    rclcpp::Node * node,
    const std::string & base_topic,
    rmw_qos_profile_t custom_qos,
    rclcpp::PublisherOptions options) override;

  /**
   * @brief Publish a sensor_msgs::msg::Image as an encoded AbrImage.
   * @param message Incoming raw image.
   * @param publish_fn Callback to publish encoded image.
   */
  void publish(
    const sensor_msgs::msg::Image & message,
    const PublishFn & publish_fn) const override;

  /**
   * @brief Service callback to update ABR encoding parameters dynamically.
   * @param req Service request containing new ABR parameters.
   * @param res Service response to confirm update.
   */
  void setAbrParams(
    const std::shared_ptr<abr_image_transport_interfaces::srv::AbrParams::Request> req,
    std::shared_ptr<abr_image_transport_interfaces::srv::AbrParams::Response> res);

  /// @brief Load default encoder parameters from a YAML file.
  void loadDefaultYaml();

private:
  /**
   * @brief Initialize FFmpeg encoder for given image size.
   * @param width Image width in pixels.
   * @param height Image height in pixels.
   */
  void initEncoder(int width, int height);

  /// @brief Clean up and release encoder resources.
  void closeEncoder();

  // ---------------------------------------------------------------------------
  // Encoder state (managed via FFmpeg).
  // ---------------------------------------------------------------------------
  AVCodecContext * codec_ctx_ = nullptr;  ///< Codec context (encoder).
  SwsContext * sws_ctx_ = nullptr;        ///< Color space conversion context.
  AVFrame * frame_ = nullptr;             ///< Frame buffer for encoding.
  AVPacket * pkt_ = nullptr;              ///< Packet buffer for encoded data.
  mutable int64_t frame_index_ = 0;       ///< Frame index (PTS counter).

  // ---------------------------------------------------------------------------
  // ROS handles.
  // ---------------------------------------------------------------------------
  rclcpp::Logger logger_;  ///< Node logger for diagnostics.
  rclcpp::Service<abr_image_transport_interfaces::srv::AbrParams>::SharedPtr srv_; ///< Service to update encoder parameters at runtime.

  // ---------------------------------------------------------------------------
  // Encoding parameters (configurable via YAML).
  // ---------------------------------------------------------------------------
  std::string codec_ = "libx264";    ///< Codec name (default: libx264).
  int bitrate_ = 400000;             ///< Target bitrate in bps (default: 400 kbps).
  int fps_ = 25;                     ///< Target frames per second (default: 25).
  std::string preset_ = "ultrafast"; ///< FFmpeg preset (default: ultrafast).
};

}  // namespace abr_image_transport

# abr_image_transport

`abr_image_transport` provides a plugin for ROS 2 [`image_transport`](https://docs.ros.org/en/rolling/p/image_transport/) that enables **adaptive bitrate (ABR) video streaming**.  
It allows compressed image topics to be transmitted efficiently over networks with varying bandwidth, dynamically adjusting video quality and bitrate to optimize **latency**, **throughput**, and **reliability**.

The plugin integrates seamlessly with the ROS 2 `image_transport` framework, supporting real-time video applications such as:

- Robotics
- Teleoperation
- Remote monitoring
- Multi-robot communication
- Cloud offloading for perception or AI

---

## ✨ Features

- Adaptive bitrate control based on network conditions  
- Codec support (e.g., H.264, H.265/HEVC)  
- Compatible with the ROS 2 `image_transport` plugin system  
- Configurable parameters for quality, target bitrate, and latency  
- Efficient handling of compressed frames (using AVPacket-like structures)  
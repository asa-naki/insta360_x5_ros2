// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
#ifndef INSTA360_X5_ROS2__INSTA360_X5_NODE_HPP_
#define INSTA360_X5_ROS2__INSTA360_X5_NODE_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/camera_publisher.hpp>
#include <image_transport/publisher.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include "insta360_x5_ros2/srv/set_camera_mode.hpp"
#include "insta360_x5_ros2/srv/get_camera_mode.hpp"
#include "insta360_x5_ros2/camera_mode.hpp"
#include "insta360_x5_ros2/gst_camera.hpp"

namespace insta360_x5_ros2
{

class Insta360X5Node : public rclcpp::Node
{
public:
  explicit Insta360X5Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~Insta360X5Node() override;

  /// Initialize publishers/services/diagnostics and start the camera.
  /// Must be called after construction (uses shared_from_this()).
  void init();

private:
  void declare_parameters();
  GstCameraConfig build_gst_config() const;
  bool start_camera(const std::string & mode_name, std::string & error_msg);
  void publish_mode_topics();

  void on_full_frame(const FrameView & fv);
  void on_preview_frame(const FrameView & fv);
  void on_jpeg_frame(const FrameView & fv);
  void on_camera_state(const std::string & state);

  void handle_set_mode(
    const std::shared_ptr<srv::SetCameraMode::Request> req,
    std::shared_ptr<srv::SetCameraMode::Response> res);
  void handle_get_mode(
    const std::shared_ptr<srv::GetCameraMode::Request> req,
    std::shared_ptr<srv::GetCameraMode::Response> res);

  // Diagnostics
  void diag_status(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void on_diag_timer();

  rclcpp::Time stamp_from_pts(const FrameView & fv) const;

  // Configuration
  std::string frame_id_;
  bool use_gst_timestamp_{true};
  double frame_timeout_sec_{5.0};
  std::string camera_info_url_;

  // Mode state (protected by camera_mutex_)
  mutable std::mutex camera_mutex_;
  CameraModeRegistry registry_;
  CameraMode current_mode_;
  GstCamera camera_;
  std::string pipeline_state_{"NULL"};

  // Publishers
  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraPublisher full_pub_;
  image_transport::Publisher preview_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr projection_pub_;

  // Services
  rclcpp::Service<srv::SetCameraMode>::SharedPtr set_mode_srv_;
  rclcpp::Service<srv::GetCameraMode>::SharedPtr get_mode_srv_;

  // Camera info
  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_mgr_;

  // Diagnostics
  std::unique_ptr<diagnostic_updater::Updater> diag_updater_;
  rclcpp::TimerBase::SharedPtr diag_timer_;

  std::atomic<uint64_t> frame_count_total_{0};
  std::atomic<uint64_t> frames_in_window_{0};
  rclcpp::Time last_frame_stamp_;
  rclcpp::Time fps_window_start_;
  double measured_fps_{0.0};
  std::mutex stats_mutex_;

  // Watchdog: avoids back-to-back restarts before the new pipeline produces frames.
  rclcpp::Time watchdog_last_restart_{0, 0, RCL_ROS_TIME};
};

}  // namespace insta360_x5_ros2

#endif  // INSTA360_X5_ROS2__INSTA360_X5_NODE_HPP_

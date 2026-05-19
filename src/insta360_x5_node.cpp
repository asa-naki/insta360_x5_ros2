// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
#include "insta360_x5_ros2/insta360_x5_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <utility>

namespace insta360_x5_ros2
{

using namespace std::chrono_literals;

Insta360X5Node::Insta360X5Node(const rclcpp::NodeOptions & options)
: rclcpp::Node("insta360_x5_node", options),
  last_frame_stamp_(0, 0, RCL_ROS_TIME),
  fps_window_start_(0, 0, RCL_ROS_TIME)
{
  declare_parameters();
}

Insta360X5Node::~Insta360X5Node()
{
  camera_.stop();
}

void Insta360X5Node::declare_parameters()
{
  declare_parameter<std::string>("mode", "equirectangular");
  declare_parameter<std::string>("device", "/dev/video0");
  declare_parameter<int>("io_mode", 4);
  declare_parameter<std::string>("frame_id", "insta360_x5_camera");

  declare_parameter<bool>("publish_preview", true);
  declare_parameter<int>("preview_width", 1440);
  declare_parameter<int>("preview_height", 720);

  declare_parameter<bool>("publish_compressed", true);
  declare_parameter<bool>("drop_old_frames", true);
  declare_parameter<bool>("use_gst_timestamp", true);

  declare_parameter<std::string>("camera_info_url", "");
  declare_parameter<double>("reconnect_interval_sec", 2.0);
  declare_parameter<double>("frame_timeout_sec", 5.0);
}

GstCameraConfig Insta360X5Node::build_gst_config() const
{
  GstCameraConfig cfg;
  cfg.device = get_parameter("device").as_string();
  cfg.io_mode = static_cast<int>(get_parameter("io_mode").as_int());
  cfg.publish_preview = get_parameter("publish_preview").as_bool();
  cfg.preview_width = static_cast<uint32_t>(get_parameter("preview_width").as_int());
  cfg.preview_height = static_cast<uint32_t>(get_parameter("preview_height").as_int());
  cfg.publish_compressed = get_parameter("publish_compressed").as_bool();
  cfg.reconnect_interval_sec = get_parameter("reconnect_interval_sec").as_double();
  return cfg;
}

void Insta360X5Node::init()
{
  frame_id_ = get_parameter("frame_id").as_string();
  use_gst_timestamp_ = get_parameter("use_gst_timestamp").as_bool();
  frame_timeout_sec_ = get_parameter("frame_timeout_sec").as_double();
  camera_info_url_ = get_parameter("camera_info_url").as_string();

  // --- Publishers ----------------------------------------------------
  rclcpp::QoS latched_qos(rclcpp::KeepLast(1));
  latched_qos.reliable().transient_local();

  mode_pub_ = create_publisher<std_msgs::msg::String>("mode", latched_qos);
  projection_pub_ = create_publisher<std_msgs::msg::String>("projection", latched_qos);

  it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
  full_pub_ = it_->advertiseCamera("image_raw", 1);

  if (get_parameter("publish_preview").as_bool()) {
    preview_pub_ = it_->advertise("image_preview", 1);
  }
  if (get_parameter("publish_compressed").as_bool()) {
    // Manual MJPEG pass-through publisher; uses standard topic
    // `image_raw/compressed`. Note: image_transport's compressed plugin
    // would publish on the same topic by re-encoding the decoded frame
    // which is wasteful. We disable that by not using it_'s compressed
    // path (we never call publish() with the decoded image only -- but
    // since we DO publish image_raw via image_transport, the plugin will
    // publish compressed too by re-encoding). Workaround: republish on a
    // dedicated topic name when the plugin's topic is present.
    // For first cut, publish to the standard topic; if a conflict shows
    // up at runtime, switch to `image_raw/jpeg`.
    compressed_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "image_raw/compressed", rclcpp::SensorDataQoS());
  }

  // --- camera_info_manager ------------------------------------------
  cinfo_mgr_ = std::make_shared<camera_info_manager::CameraInfoManager>(
    this, "insta360_x5", camera_info_url_);

  // --- Services -----------------------------------------------------
  set_mode_srv_ = create_service<srv::SetCameraMode>(
    "set_mode",
    std::bind(&Insta360X5Node::handle_set_mode, this,
      std::placeholders::_1, std::placeholders::_2));
  get_mode_srv_ = create_service<srv::GetCameraMode>(
    "get_mode",
    std::bind(&Insta360X5Node::handle_get_mode, this,
      std::placeholders::_1, std::placeholders::_2));

  // --- Camera callbacks ---------------------------------------------
  camera_.set_full_callback(
    [this](const FrameView & fv) {this->on_full_frame(fv);});
  camera_.set_preview_callback(
    [this](const FrameView & fv) {this->on_preview_frame(fv);});
  camera_.set_jpeg_callback(
    [this](const FrameView & fv) {this->on_jpeg_frame(fv);});
  camera_.set_state_callback(
    [this](const std::string & s) {this->on_camera_state(s);});

  // --- Diagnostics --------------------------------------------------
  diag_updater_ = std::make_unique<diagnostic_updater::Updater>(this);
  diag_updater_->setHardwareID("insta360_x5");
  diag_updater_->add("insta360_x5",
    std::bind(&Insta360X5Node::diag_status, this, std::placeholders::_1));
  diag_timer_ = create_wall_timer(
    1s, std::bind(&Insta360X5Node::on_diag_timer, this));

  // --- Start camera -------------------------------------------------
  const std::string initial_mode = get_parameter("mode").as_string();
  std::string err;
  if (!start_camera(initial_mode, err)) {
    RCLCPP_ERROR(get_logger(), "Initial camera start failed: %s", err.c_str());
  }
}

bool Insta360X5Node::start_camera(const std::string & mode_name, std::string & error_msg)
{
  std::lock_guard<std::mutex> lock(camera_mutex_);
  auto mode_opt = registry_.get(mode_name);
  if (!mode_opt) {
    error_msg = "Unknown mode: " + mode_name;
    return false;
  }
  GstCameraConfig cfg = build_gst_config();
  if (!camera_.restart(*mode_opt, cfg)) {
    error_msg = "GStreamer pipeline failed to start";
    return false;
  }
  current_mode_ = *mode_opt;

  // Reset CameraInfo to mode-appropriate width/height.
  sensor_msgs::msg::CameraInfo ci = cinfo_mgr_->getCameraInfo();
  if (camera_info_url_.empty() || ci.width == 0 || ci.height == 0) {
    ci = sensor_msgs::msg::CameraInfo();
    ci.width = current_mode_.width;
    ci.height = current_mode_.height;
    cinfo_mgr_->setCameraInfo(ci);
  }

  publish_mode_topics();

  // Reset stats. Set last_frame_stamp_ to now() so the watchdog gives the new
  // pipeline a grace period (frame_timeout_sec) before considering it stalled.
  {
    std::lock_guard<std::mutex> sl(stats_mutex_);
    frames_in_window_.store(0);
    fps_window_start_ = now();
    measured_fps_ = 0.0;
    last_frame_stamp_ = fps_window_start_;
  }
  watchdog_last_restart_ = now();
  RCLCPP_INFO(get_logger(), "Camera started in mode '%s' (%ux%u@%u, %s)",
    current_mode_.name.c_str(), current_mode_.width, current_mode_.height,
    current_mode_.fps, current_mode_.projection.c_str());
  return true;
}

void Insta360X5Node::publish_mode_topics()
{
  std_msgs::msg::String m;
  m.data = current_mode_.name;
  mode_pub_->publish(m);
  std_msgs::msg::String p;
  p.data = current_mode_.projection;
  projection_pub_->publish(p);
}

rclcpp::Time Insta360X5Node::stamp_from_pts(const FrameView & fv) const
{
  if (use_gst_timestamp_ && fv.pts_valid) {
    // GstBuffer::pts is in nanoseconds (GstClockTime).
    // With do-timestamp=true on a live source it derives from the pipeline
    // clock (monotonic). For ROS we just attach it as RCL_STEADY_TIME to
    // keep relative frame timing, but most downstream code expects ROS time.
    // To remain compatible with sensor_data QoS and rosbags, return ROS time
    // anchored to "now - (latest pts - this pts)" would be ideal; for the
    // first cut, fall back to ROS now() when pts is monotonic-anchored.
    // Here: just use now() — pts based handling can be refined later.
    (void)fv;
  }
  return const_cast<Insta360X5Node *>(this)->now();
}

void Insta360X5Node::on_full_frame(const FrameView & fv)
{
  if (fv.width == 0 || fv.height == 0 || fv.data == nullptr) {return;}

  auto img = std::make_unique<sensor_msgs::msg::Image>();
  img->header.stamp = stamp_from_pts(fv);
  img->header.frame_id = frame_id_;
  img->width = fv.width;
  img->height = fv.height;
  img->encoding = "bgr8";
  img->is_bigendian = 0;
  img->step = fv.width * 3;
  const size_t expected = static_cast<size_t>(img->step) * img->height;
  const size_t copy_size = std::min(expected, fv.size);
  img->data.resize(copy_size);
  std::memcpy(img->data.data(), fv.data, copy_size);

  sensor_msgs::msg::CameraInfo ci = cinfo_mgr_->getCameraInfo();
  if (ci.width == 0 || ci.height == 0) {
    ci.width = fv.width;
    ci.height = fv.height;
  }
  ci.header = img->header;

  full_pub_.publish(*img, ci);

  // Stats
  {
    std::lock_guard<std::mutex> sl(stats_mutex_);
    last_frame_stamp_ = img->header.stamp;
    frame_count_total_++;
    frames_in_window_++;
  }
}

void Insta360X5Node::on_preview_frame(const FrameView & fv)
{
  if (!preview_pub_ || fv.width == 0 || fv.height == 0) {return;}
  sensor_msgs::msg::Image img;
  img.header.stamp = stamp_from_pts(fv);
  img.header.frame_id = frame_id_;
  img.width = fv.width;
  img.height = fv.height;
  img.encoding = "bgr8";
  img.is_bigendian = 0;
  img.step = fv.width * 3;
  const size_t expected = static_cast<size_t>(img.step) * img.height;
  const size_t copy_size = std::min(expected, fv.size);
  img.data.resize(copy_size);
  std::memcpy(img.data.data(), fv.data, copy_size);
  preview_pub_.publish(img);
}

void Insta360X5Node::on_jpeg_frame(const FrameView & fv)
{
  if (!compressed_pub_) {return;}
  sensor_msgs::msg::CompressedImage msg;
  msg.header.stamp = stamp_from_pts(fv);
  msg.header.frame_id = frame_id_;
  msg.format = "jpeg";
  msg.data.assign(fv.data, fv.data + fv.size);
  compressed_pub_->publish(msg);
}

void Insta360X5Node::on_camera_state(const std::string & state)
{
  std::lock_guard<std::mutex> lock(camera_mutex_);
  pipeline_state_ = state;
  RCLCPP_INFO(get_logger(), "Camera state: %s", state.c_str());
}

void Insta360X5Node::handle_set_mode(
  const std::shared_ptr<srv::SetCameraMode::Request> req,
  std::shared_ptr<srv::SetCameraMode::Response> res)
{
  RCLCPP_INFO(get_logger(), "set_mode request: '%s'", req->mode.c_str());
  if (!registry_.has(req->mode)) {
    res->success = false;
    res->message = "Unknown mode: " + req->mode;
    res->current_mode = current_mode_.name;
    return;
  }
  std::string err;
  if (!start_camera(req->mode, err)) {
    res->success = false;
    res->message = err;
    res->current_mode = current_mode_.name;
    return;
  }
  res->success = true;
  res->message = "OK";
  res->current_mode = current_mode_.name;
}

void Insta360X5Node::handle_get_mode(
  const std::shared_ptr<srv::GetCameraMode::Request>/*req*/,
  std::shared_ptr<srv::GetCameraMode::Response> res)
{
  std::lock_guard<std::mutex> lock(camera_mutex_);
  res->current_mode = current_mode_.name;
  res->available_modes = registry_.available_modes();
}

void Insta360X5Node::on_diag_timer()
{
  // Update measured fps over the last 1s window.
  rclcpp::Time last_stamp;
  {
    std::lock_guard<std::mutex> sl(stats_mutex_);
    auto t = now();
    double dt = (t - fps_window_start_).seconds();
    if (dt > 0.0) {
      measured_fps_ = static_cast<double>(frames_in_window_.exchange(0)) / dt;
      fps_window_start_ = t;
    }
    last_stamp = last_frame_stamp_;
  }
  diag_updater_->force_update();

  // Watchdog: detect silent freeze (PLAYING but no frames for >= timeout) and
  // force a pipeline rebuild. GStreamer dropped frames in queues already, so
  // the camera_ side cannot deadlock on slow subscribers; this catches v4l2
  // anomalies (USB hiccup, kernel UVC stall) where no ERROR/EOS reaches the bus.
  std::string pipe_state;
  std::string mode_name;
  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    pipe_state = pipeline_state_;
    mode_name = current_mode_.name;
  }
  if (pipe_state != "PLAYING" || last_stamp.nanoseconds() == 0) {return;}
  const double age = (now() - last_stamp).seconds();
  if (age < frame_timeout_sec_) {return;}
  // Throttle: don't retrigger more often than frame_timeout_sec.
  if (watchdog_last_restart_.nanoseconds() > 0 &&
    (now() - watchdog_last_restart_).seconds() < frame_timeout_sec_)
  {
    return;
  }
  RCLCPP_WARN(get_logger(),
    "Watchdog: no frames for %.1fs while PLAYING — forcing pipeline restart",
    age);
  std::string err;
  if (!start_camera(mode_name, err)) {
    RCLCPP_ERROR(get_logger(), "Watchdog restart failed: %s", err.c_str());
  }
}

void Insta360X5Node::diag_status(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::string mode_name;
  std::string projection;
  uint32_t w = 0, h = 0, fps_target = 0;
  std::string pipe_state;
  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    mode_name = current_mode_.name;
    projection = current_mode_.projection;
    w = current_mode_.width;
    h = current_mode_.height;
    fps_target = current_mode_.fps;
    pipe_state = pipeline_state_;
  }

  double last_age = std::numeric_limits<double>::infinity();
  double fps_val = 0.0;
  uint64_t total = frame_count_total_.load();
  {
    std::lock_guard<std::mutex> sl(stats_mutex_);
    if (last_frame_stamp_.nanoseconds() > 0) {
      last_age = (now() - last_frame_stamp_).seconds();
    }
    fps_val = measured_fps_;
  }

  stat.add("current_mode", mode_name);
  stat.add("projection", projection);
  stat.add("width", w);
  stat.add("height", h);
  stat.add("target_fps", fps_target);
  stat.add("measured_fps", fps_val);
  stat.add("frames_total", total);
  stat.add("last_frame_age_sec", last_age);
  stat.add("pipeline_state", pipe_state);
  stat.add("device", get_parameter("device").as_string());

  const bool playing = (pipe_state == "PLAYING");
  if (!playing) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "Pipeline not playing (" + pipe_state + ")");
    return;
  }
  if (last_age >= frame_timeout_sec_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "No frames for >= frame_timeout_sec");
    return;
  }
  const double fps_thresh = static_cast<double>(fps_target) * 0.7;
  if (last_age >= 1.0 || (fps_target > 0 && fps_val < fps_thresh)) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "Low fps or stale frame");
    return;
  }
  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
}

}  // namespace insta360_x5_ros2

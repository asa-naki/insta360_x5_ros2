// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
#ifndef INSTA360_X5_ROS2__GST_CAMERA_HPP_
#define INSTA360_X5_ROS2__GST_CAMERA_HPP_

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

extern "C" {
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
}

#include "insta360_x5_ros2/camera_mode.hpp"

namespace insta360_x5_ros2
{

struct FrameView
{
  const uint8_t * data;
  size_t size;
  uint32_t width;
  uint32_t height;
  uint64_t pts_ns;   // GstClockTime; GST_CLOCK_TIME_NONE if invalid
  bool pts_valid;
};

/// Configuration for a single pipeline build.
struct GstCameraConfig
{
  std::string device{"/dev/video0"};
  int io_mode{4};                 // 4=DMABUF, 2=MMAP
  bool publish_preview{true};
  uint32_t preview_width{1440};
  uint32_t preview_height{720};
  bool publish_compressed{true};
  double reconnect_interval_sec{2.0};
};

class GstCamera
{
public:
  using FrameCallback = std::function<void (const FrameView &)>;
  using JpegCallback = std::function<void (const FrameView &)>;
  using StateCallback = std::function<void (const std::string & state)>;

  GstCamera();
  ~GstCamera();

  GstCamera(const GstCamera &) = delete;
  GstCamera & operator=(const GstCamera &) = delete;

  void set_full_callback(FrameCallback cb) {full_cb_ = std::move(cb);}
  void set_preview_callback(FrameCallback cb) {preview_cb_ = std::move(cb);}
  void set_jpeg_callback(JpegCallback cb) {jpeg_cb_ = std::move(cb);}
  void set_state_callback(StateCallback cb) {state_cb_ = std::move(cb);}

  /// Start pipeline with given mode/config. Blocks until PLAYING (or failure).
  bool start(const CameraMode & mode, const GstCameraConfig & cfg);

  /// Stop pipeline and tear down the bus thread. Safe to call multiple times.
  void stop();

  /// Restart pipeline with a (possibly new) mode. Thread-safe.
  bool restart(const CameraMode & mode, const GstCameraConfig & cfg);

  /// Returns current pipeline state as a human-readable string.
  std::string pipeline_state() const;

  /// Returns true if pipeline is currently constructed and PLAYING.
  bool is_playing() const;

  const CameraMode & current_mode() const {return current_mode_;}

private:
  // Internal helpers (must be called with mutex_ held).
  bool build_pipeline_locked(const CameraMode & mode, const GstCameraConfig & cfg, int io_mode);
  void teardown_pipeline_locked();
  std::string build_pipeline_str(
    const CameraMode & mode, const GstCameraConfig & cfg,
    int io_mode) const;

  // GStreamer callbacks (static -> instance).
  static GstFlowReturn on_new_sample_full(GstAppSink * sink, gpointer user_data);
  static GstFlowReturn on_new_sample_preview(GstAppSink * sink, gpointer user_data);
  static GstFlowReturn on_new_sample_jpeg(GstAppSink * sink, gpointer user_data);
  static gboolean on_bus_message(GstBus * bus, GstMessage * msg, gpointer user_data);

  void bus_thread_main();
  void notify_state(const std::string & state);

  mutable std::mutex mutex_;
  GstElement * pipeline_{nullptr};
  GMainLoop * main_loop_{nullptr};
  std::thread bus_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> reconnect_requested_{false};

  CameraMode current_mode_{};
  GstCameraConfig current_cfg_{};
  int current_io_mode_{4};

  FrameCallback full_cb_;
  FrameCallback preview_cb_;
  JpegCallback jpeg_cb_;
  StateCallback state_cb_;
};

}  // namespace insta360_x5_ros2

#endif  // INSTA360_X5_ROS2__GST_CAMERA_HPP_

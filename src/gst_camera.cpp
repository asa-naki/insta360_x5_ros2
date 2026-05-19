// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
#include "insta360_x5_ros2/gst_camera.hpp"

#include <sstream>
#include <utility>

#include <rclcpp/rclcpp.hpp>

namespace insta360_x5_ros2
{

namespace
{
constexpr const char * kLogger = "GstCamera";

void ensure_gst_init()
{
  static std::once_flag flag;
  std::call_once(flag, [] {
      gst_init(nullptr, nullptr);
    });
}
}  // namespace

GstCamera::GstCamera()
{
  ensure_gst_init();
}

GstCamera::~GstCamera()
{
  stop();
}

bool GstCamera::start(const CameraMode & mode, const GstCameraConfig & cfg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (pipeline_) {
    RCLCPP_WARN(rclcpp::get_logger(kLogger),
        "start() called while pipeline already exists; tearing down first");
    teardown_pipeline_locked();
  }

  current_mode_ = mode;
  current_cfg_ = cfg;

  // Try requested io_mode first, then fall back to MMAP(2).
  for (int io_mode : {cfg.io_mode, (cfg.io_mode == 2 ? 2 : 2)}) {
    if (build_pipeline_locked(mode, cfg, io_mode)) {
      current_io_mode_ = io_mode;
      // Spawn bus thread
      main_loop_ = g_main_loop_new(nullptr, FALSE);
      running_.store(true);
      bus_thread_ = std::thread(&GstCamera::bus_thread_main, this);
      notify_state("PLAYING");
      return true;
    }
    if (cfg.io_mode == io_mode) {
      RCLCPP_WARN(rclcpp::get_logger(kLogger),
        "Pipeline failed with io_mode=%d; will retry with io_mode=2 (MMAP)", io_mode);
    } else {
      break;
    }
  }
  notify_state("ERROR");
  return false;
}

void GstCamera::stop()
{
  std::unique_lock<std::mutex> lock(mutex_);
  if (!pipeline_ && !main_loop_ && !bus_thread_.joinable()) {
    return;
  }
  running_.store(false);

  if (main_loop_) {
    g_main_loop_quit(main_loop_);
  }

  // Unlock before joining to avoid deadlock vs. bus callbacks acquiring the mutex.
  lock.unlock();
  if (bus_thread_.joinable()) {
    bus_thread_.join();
  }
  lock.lock();

  if (main_loop_) {
    g_main_loop_unref(main_loop_);
    main_loop_ = nullptr;
  }

  teardown_pipeline_locked();
  notify_state("STOPPED");
}

bool GstCamera::restart(const CameraMode & mode, const GstCameraConfig & cfg)
{
  stop();
  return start(mode, cfg);
}

std::string GstCamera::pipeline_state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!pipeline_) {
    return "NULL";
  }
  GstState state{GST_STATE_NULL};
  gst_element_get_state(pipeline_, &state, nullptr, 0);
  return gst_element_state_get_name(state);
}

bool GstCamera::is_playing() const
{
  return pipeline_state() == std::string("PLAYING");
}

bool GstCamera::build_pipeline_locked(
  const CameraMode & mode, const GstCameraConfig & cfg,
  int io_mode)
{
  const std::string pipeline_str = build_pipeline_str(mode, cfg, io_mode);
  RCLCPP_INFO(rclcpp::get_logger(kLogger), "Building pipeline: %s", pipeline_str.c_str());

  GError * err = nullptr;
  GstElement * pipe = gst_parse_launch(pipeline_str.c_str(), &err);
  if (!pipe || err) {
    RCLCPP_ERROR(rclcpp::get_logger(kLogger), "gst_parse_launch failed: %s",
      err ? err->message : "unknown");
    if (err) {g_error_free(err);}
    if (pipe) {gst_object_unref(pipe);}
    return false;
  }

  // Hook appsinks.
  auto connect_sink = [&](const char * name, GstFlowReturn (*cb)(GstAppSink *, gpointer)) {
      GstElement * sink = gst_bin_get_by_name(GST_BIN(pipe), name);
      if (!sink) {return false;}
      GstAppSinkCallbacks callbacks{};
      callbacks.new_sample = cb;
      gst_app_sink_set_callbacks(GST_APP_SINK(sink), &callbacks, this, nullptr);
      gst_app_sink_set_drop(GST_APP_SINK(sink), TRUE);
      gst_app_sink_set_max_buffers(GST_APP_SINK(sink), 1);
      gst_object_unref(sink);
      return true;
    };

  if (!connect_sink("full_sink", &GstCamera::on_new_sample_full)) {
    RCLCPP_ERROR(rclcpp::get_logger(kLogger), "Missing appsink 'full_sink' in pipeline");
    gst_object_unref(pipe);
    return false;
  }
  if (cfg.publish_preview) {connect_sink("preview_sink", &GstCamera::on_new_sample_preview);}
  if (cfg.publish_compressed) {connect_sink("jpeg_sink", &GstCamera::on_new_sample_jpeg);}

  // Transition to PLAYING (synchronously wait).
  GstStateChangeReturn ret = gst_element_set_state(pipe, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(rclcpp::get_logger(kLogger), "Failed to set PLAYING state");
    gst_element_set_state(pipe, GST_STATE_NULL);
    gst_object_unref(pipe);
    return false;
  }
  GstState cur{GST_STATE_NULL};
  ret = gst_element_get_state(pipe, &cur, nullptr, 5 * GST_SECOND);
  if (ret == GST_STATE_CHANGE_FAILURE || cur != GST_STATE_PLAYING) {
    RCLCPP_ERROR(rclcpp::get_logger(kLogger),
      "Pipeline did not reach PLAYING (state=%s, ret=%d)",
      gst_element_state_get_name(cur), ret);
    gst_element_set_state(pipe, GST_STATE_NULL);
    gst_object_unref(pipe);
    return false;
  }

  pipeline_ = pipe;
  return true;
}

void GstCamera::teardown_pipeline_locked()
{
  if (!pipeline_) {return;}
  gst_element_set_state(pipeline_, GST_STATE_NULL);
  gst_object_unref(pipeline_);
  pipeline_ = nullptr;
}

std::string GstCamera::build_pipeline_str(
  const CameraMode & mode, const GstCameraConfig & cfg,
  int io_mode) const
{
  std::ostringstream ss;
  ss << "v4l2src device=" << cfg.device
     << " io-mode=" << io_mode
     << " do-timestamp=true"
     << " ! image/jpeg,width=" << mode.width
     << ",height=" << mode.height
     << ",framerate=" << mode.fps << "/1"
     << " ! queue leaky=downstream max-size-buffers=1"
     << " ! jpegparse";

  // We always need a tee after jpegparse if compressed is wanted; otherwise route directly to decode.
  if (cfg.publish_compressed) {
    ss << " ! tee name=jpeg_t"
       << " jpeg_t. ! queue leaky=downstream max-size-buffers=1"
       << " ! appsink name=jpeg_sink emit-signals=true sync=false"
       << " jpeg_t. ! queue leaky=downstream max-size-buffers=1"
       << " ! jpegdec ! videoconvert ! video/x-raw,format=BGR";
  } else {
    ss << " ! jpegdec ! videoconvert ! video/x-raw,format=BGR";
  }

  if (cfg.publish_preview) {
    ss << " ! tee name=raw_t"
       << " raw_t. ! queue leaky=downstream max-size-buffers=1"
       << " ! appsink name=full_sink emit-signals=true sync=false"
       << " raw_t. ! queue leaky=downstream max-size-buffers=1"
       << " ! videoscale ! video/x-raw,width=" << cfg.preview_width
       << ",height=" << cfg.preview_height
       << " ! appsink name=preview_sink emit-signals=true sync=false";
  } else {
    ss << " ! appsink name=full_sink emit-signals=true sync=false";
  }
  return ss.str();
}

GstFlowReturn GstCamera::on_new_sample_full(GstAppSink * sink, gpointer user_data)
{
  auto * self = static_cast<GstCamera *>(user_data);
  GstSample * sample = gst_app_sink_pull_sample(sink);
  if (!sample) {return GST_FLOW_OK;}

  GstBuffer * buf = gst_sample_get_buffer(sample);
  GstCaps * caps = gst_sample_get_caps(sample);
  GstStructure * s = caps ? gst_caps_get_structure(caps, 0) : nullptr;
  gint w = 0, h = 0;
  if (s) {gst_structure_get_int(s, "width", &w); gst_structure_get_int(s, "height", &h);}

  GstMapInfo map;
  if (buf && gst_buffer_map(buf, &map, GST_MAP_READ)) {
    FrameView fv{
      map.data, map.size,
      static_cast<uint32_t>(w), static_cast<uint32_t>(h),
      static_cast<uint64_t>(GST_BUFFER_PTS(buf)),
      GST_BUFFER_PTS_IS_VALID(buf) ? true : false
    };
    if (self->full_cb_) {self->full_cb_(fv);}
    gst_buffer_unmap(buf, &map);
  }
  gst_sample_unref(sample);
  return GST_FLOW_OK;
}

GstFlowReturn GstCamera::on_new_sample_preview(GstAppSink * sink, gpointer user_data)
{
  auto * self = static_cast<GstCamera *>(user_data);
  GstSample * sample = gst_app_sink_pull_sample(sink);
  if (!sample) {return GST_FLOW_OK;}

  GstBuffer * buf = gst_sample_get_buffer(sample);
  GstCaps * caps = gst_sample_get_caps(sample);
  GstStructure * s = caps ? gst_caps_get_structure(caps, 0) : nullptr;
  gint w = 0, h = 0;
  if (s) {gst_structure_get_int(s, "width", &w); gst_structure_get_int(s, "height", &h);}

  GstMapInfo map;
  if (buf && gst_buffer_map(buf, &map, GST_MAP_READ)) {
    FrameView fv{
      map.data, map.size,
      static_cast<uint32_t>(w), static_cast<uint32_t>(h),
      static_cast<uint64_t>(GST_BUFFER_PTS(buf)),
      GST_BUFFER_PTS_IS_VALID(buf) ? true : false
    };
    if (self->preview_cb_) {self->preview_cb_(fv);}
    gst_buffer_unmap(buf, &map);
  }
  gst_sample_unref(sample);
  return GST_FLOW_OK;
}

GstFlowReturn GstCamera::on_new_sample_jpeg(GstAppSink * sink, gpointer user_data)
{
  auto * self = static_cast<GstCamera *>(user_data);
  GstSample * sample = gst_app_sink_pull_sample(sink);
  if (!sample) {return GST_FLOW_OK;}

  GstBuffer * buf = gst_sample_get_buffer(sample);
  GstMapInfo map;
  if (buf && gst_buffer_map(buf, &map, GST_MAP_READ)) {
    FrameView fv{
      map.data, map.size,
      self->current_mode_.width, self->current_mode_.height,
      static_cast<uint64_t>(GST_BUFFER_PTS(buf)),
      GST_BUFFER_PTS_IS_VALID(buf) ? true : false
    };
    if (self->jpeg_cb_) {self->jpeg_cb_(fv);}
    gst_buffer_unmap(buf, &map);
  }
  gst_sample_unref(sample);
  return GST_FLOW_OK;
}

gboolean GstCamera::on_bus_message(GstBus * /*bus*/, GstMessage * msg, gpointer user_data)
{
  auto * self = static_cast<GstCamera *>(user_data);
  switch (GST_MESSAGE_TYPE(msg)) {
    case GST_MESSAGE_ERROR: {
        GError * err = nullptr;
        gchar * dbg = nullptr;
        gst_message_parse_error(msg, &err, &dbg);
        RCLCPP_ERROR(rclcpp::get_logger(kLogger), "GStreamer ERROR: %s (%s)",
          err ? err->message : "?", dbg ? dbg : "");
        if (err) {g_error_free(err);}
        g_free(dbg);
        self->notify_state("ERROR");
        self->reconnect_requested_.store(true);
        if (self->main_loop_) {g_main_loop_quit(self->main_loop_);}
        break;
      }
    case GST_MESSAGE_EOS:
      RCLCPP_WARN(rclcpp::get_logger(kLogger), "GStreamer EOS");
      self->notify_state("EOS");
      self->reconnect_requested_.store(true);
      if (self->main_loop_) {g_main_loop_quit(self->main_loop_);}
      break;
    default:
      break;
  }
  return TRUE;
}

void GstCamera::bus_thread_main()
{
  // Attach bus watch to current pipeline & loop context.
  GstBus * bus = gst_element_get_bus(pipeline_);
  guint watch_id = gst_bus_add_watch(bus, &GstCamera::on_bus_message, this);

  while (running_.load()) {
    g_main_loop_run(main_loop_);

    if (!running_.load()) {break;}

    if (reconnect_requested_.exchange(false)) {
      RCLCPP_WARN(rclcpp::get_logger(kLogger),
        "Pipeline lost; reconnecting in %.1fs", current_cfg_.reconnect_interval_sec);
      // Best-effort sleep without blocking too long if shutdown comes in.
      auto until = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(current_cfg_.reconnect_interval_sec);
      while (running_.load() && std::chrono::steady_clock::now() < until) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      if (!running_.load()) {break;}

      // Rebuild under lock.
      {
        std::lock_guard<std::mutex> lock(mutex_);
        g_source_remove(watch_id);
        gst_object_unref(bus);

        teardown_pipeline_locked();
        if (!build_pipeline_locked(current_mode_, current_cfg_, current_io_mode_)) {
          // Try the alternate io_mode once.
          int alt = (current_io_mode_ == 4) ? 2 : 4;
          RCLCPP_WARN(rclcpp::get_logger(kLogger),
            "Reconnect failed with io_mode=%d, trying io_mode=%d",
            current_io_mode_, alt);
          if (build_pipeline_locked(current_mode_, current_cfg_, alt)) {
            current_io_mode_ = alt;
          } else {
            notify_state("ERROR");
            reconnect_requested_.store(true);
            // Re-arm bus watch on (possibly null) pipeline; loop will re-enter sleep.
            // To avoid tight loop, just continue to top of while which will
            // attempt reconnect again after the same interval.
            bus = nullptr;
            watch_id = 0;
            continue;
          }
        }
        notify_state("PLAYING");
        bus = gst_element_get_bus(pipeline_);
        watch_id = gst_bus_add_watch(bus, &GstCamera::on_bus_message, this);
      }
    } else {
      // Loop quit without reconnect request -> shutdown path.
      break;
    }
  }

  if (watch_id) {g_source_remove(watch_id);}
  if (bus) {gst_object_unref(bus);}
}

void GstCamera::notify_state(const std::string & state)
{
  if (state_cb_) {state_cb_(state);}
}

}  // namespace insta360_x5_ros2

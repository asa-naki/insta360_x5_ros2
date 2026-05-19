# insta360_x5_ros2

ROS 2 (Jazzy) node for the Insta360 X5 in UVC mode. Built on GStreamer.
Exposes the camera as `sensor_msgs/Image`, `sensor_msgs/CompressedImage`
(MJPEG pass-through), `sensor_msgs/CameraInfo`, mode/projection metadata
and live diagnostics.

## Features

- **Mode-based** camera control: `equirectangular` (2880x1440) /
  `dual_fisheye` (1920x1080). Resolution and projection meaning are
  bound together in `CameraModeRegistry`.
- GStreamer pipeline (`v4l2src → jpegparse → tee → {appsink MJPEG,
  jpegdec → videoconvert(BGR) → tee → {full appsink, scaled preview appsink}}`)
  with `leaky=downstream max-size-buffers=1` for latest-frame-wins behaviour.
- `image_transport` for `image_raw` + `compressed_image_transport`
  plugin in `package.xml`, plus a **self-managed MJPEG pass-through**
  publisher on `image_raw/compressed` to avoid the plugin's re-encode CPU cost.
- `camera_info_manager` integration via `camera_info_url`.
- `diagnostic_updater` with `measured_fps`, `last_frame_age_sec`,
  `pipeline_state`, etc.
- GStreamer bus monitoring with **automatic reconnect** on `ERROR`/`EOS`
  (and `io_mode=4 → 2` fallback).
- Runtime mode switching via the `set_mode` service (rebuilds the pipeline).

## Topics (under namespace `/insta360_x5`)

| Topic | Type | QoS |
|---|---|---|
| `image_raw` | `sensor_msgs/Image` (bgr8) | SensorData |
| `image_raw/compressed` | `sensor_msgs/CompressedImage` (jpeg) | SensorData |
| `camera_info` | `sensor_msgs/CameraInfo` | SensorData |
| `image_preview` | `sensor_msgs/Image` (bgr8) | SensorData |
| `mode` | `std_msgs/String` | reliable, transient_local |
| `projection` | `std_msgs/String` | reliable, transient_local |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | reliable |

## Services

- `/insta360_x5/set_mode` (`insta360_x5_ros2/srv/SetCameraMode`)
- `/insta360_x5/get_mode` (`insta360_x5_ros2/srv/GetCameraMode`)

## Build

```bash
cd ~/workspace/insta360_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select insta360_x5_ros2 --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch insta360_x5_ros2 insta360_x5.launch.py
```

Or with a custom device / mode:

```bash
ros2 run insta360_x5_ros2 insta360_x5_node --ros-args \
  -p mode:=dual_fisheye -p device:=/dev/video0
```

Switch mode at runtime:

```bash
ros2 service call /insta360_x5/set_mode \
  insta360_x5_ros2/srv/SetCameraMode "{mode: 'equirectangular'}"
```

## Test

```bash
colcon test --packages-select insta360_x5_ros2
colcon test-result --verbose
```

Smoke test (requires camera connected):

```bash
ros2 run insta360_x5_ros2 smoke_test.sh
```

## Large-message DDS tuning (for raw `image_raw` / `image_preview`)

The primary high-throughput path is `image_raw/compressed` (MJPEG, ~200 KB/frame)
and works out of the box at the camera's native 30 fps.

`image_raw` is `sensor_msgs/Image` (bgr8). At 2880×1440 it is ~12 MB per frame
and at 1440×720 it is ~3 MB. The default CycloneDDS configuration cannot
reassemble such large UDP-fragmented messages on loopback, so a subscriber will
see the topic but receive zero messages. The node itself keeps running and the
camera does not freeze (GStreamer queues use `leaky=downstream max-size-buffers=1`).

To make raw `Image` flow, do **both**:

```bash
# 1) Raise kernel socket-buffer ceilings (once per boot).
sudo sysctl -w net.core.rmem_max=134217728
sudo sysctl -w net.core.wmem_max=134217728

# 2) Point CycloneDDS at the shipped XML profile.
export CYCLONEDDS_URI=file://$(ros2 pkg prefix insta360_x5_ros2)/share/insta360_x5_ros2/config/cyclonedds_large_msg.xml
```

Then re-launch the node. If you only need compressed frames (recommended for
remote subscribers and bag recording), no tuning is required.

## Reliability

- GStreamer pipeline uses `queue leaky=downstream max-size-buffers=1` on every
  branch and `gst_app_sink_set_drop(TRUE)` / `max-buffers=1`, so slow downstream
  (ROS subscribers, DDS) cannot backpressure the camera. v4l2 capture keeps
  running at full rate; only the ROS-side rate degrades.
- Bus monitor catches `ERROR`/`EOS` and rebuilds the pipeline after
  `reconnect_interval_sec` (default 2 s) with `io_mode=4 → 2` fallback.
- A node-side watchdog (running on the diagnostic timer) forces a pipeline
  rebuild if no frame is received for `frame_timeout_sec` (default 5 s) while
  the pipeline reports `PLAYING`. This recovers from silent UVC / v4l2 stalls
  that don't produce a GStreamer bus message.
- On SIGINT/SIGTERM the node sets the pipeline to `NULL` synchronously, which
  releases `/dev/video0` before the process exits.

## Permissions / udev

Allow the current user to access video devices:

```bash
sudo usermod -aG video $USER  # then log out / log in
```

Optional udev rule to give the camera a stable device node:

```
# /etc/udev/rules.d/99-insta360-x5.rules
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="2e1a", SYMLINK+="insta360_x5"
```

Then pass `device:=/dev/insta360_x5`.

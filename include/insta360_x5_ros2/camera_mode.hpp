// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
#ifndef INSTA360_X5_ROS2__CAMERA_MODE_HPP_
#define INSTA360_X5_ROS2__CAMERA_MODE_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace insta360_x5_ros2
{

struct CameraMode
{
  std::string name;
  uint32_t width{0};
  uint32_t height{0};
  uint32_t fps{0};
  std::string format;       // "MJPG"
  std::string projection;   // "equirectangular", "dual_fisheye_top_bottom"
};

class CameraModeRegistry
{
public:
  CameraModeRegistry();

  /// Returns the mode for the given name, or std::nullopt.
  std::optional<CameraMode> get(const std::string & name) const;

  /// Returns sorted list of mode names.
  std::vector<std::string> available_modes() const;

  /// True if mode is registered.
  bool has(const std::string & name) const;

private:
  std::unordered_map<std::string, CameraMode> modes_;
};

}  // namespace insta360_x5_ros2

#endif  // INSTA360_X5_ROS2__CAMERA_MODE_HPP_

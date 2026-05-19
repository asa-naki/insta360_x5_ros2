// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
#include "insta360_x5_ros2/camera_mode.hpp"

#include <algorithm>

namespace insta360_x5_ros2
{

CameraModeRegistry::CameraModeRegistry()
{
  modes_.emplace(
    "equirectangular",
    CameraMode{
      /*name=*/"equirectangular",
      /*width=*/2880,
      /*height=*/1440,
      /*fps=*/30,
      /*format=*/"MJPG",
      /*projection=*/"equirectangular"});

  modes_.emplace(
    "dual_fisheye",
    CameraMode{
      /*name=*/"dual_fisheye",
      /*width=*/1920,
      /*height=*/1080,
      /*fps=*/30,
      /*format=*/"MJPG",
      /*projection=*/"dual_fisheye_top_bottom"});
}

std::optional<CameraMode> CameraModeRegistry::get(const std::string & name) const
{
  auto it = modes_.find(name);
  if (it == modes_.end()) {
    return std::nullopt;
  }
  return it->second;
}

std::vector<std::string> CameraModeRegistry::available_modes() const
{
  std::vector<std::string> names;
  names.reserve(modes_.size());
  for (const auto & kv : modes_) {
    names.push_back(kv.first);
  }
  std::sort(names.begin(), names.end());
  return names;
}

bool CameraModeRegistry::has(const std::string & name) const
{
  return modes_.find(name) != modes_.end();
}

}  // namespace insta360_x5_ros2

// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include "insta360_x5_ros2/camera_mode.hpp"

using insta360_x5_ros2::CameraModeRegistry;

TEST(CameraModeRegistry, EquirectangularExists)
{
  CameraModeRegistry reg;
  auto m = reg.get("equirectangular");
  ASSERT_TRUE(m.has_value());
  EXPECT_EQ(m->width, 2880u);
  EXPECT_EQ(m->height, 1440u);
  EXPECT_EQ(m->fps, 30u);
  EXPECT_EQ(m->format, "MJPG");
  EXPECT_EQ(m->projection, "equirectangular");
}

TEST(CameraModeRegistry, DualFisheyeExists)
{
  CameraModeRegistry reg;
  auto m = reg.get("dual_fisheye");
  ASSERT_TRUE(m.has_value());
  EXPECT_EQ(m->width, 1920u);
  EXPECT_EQ(m->height, 1080u);
  EXPECT_EQ(m->fps, 30u);
  EXPECT_EQ(m->projection, "dual_fisheye_top_bottom");
}

TEST(CameraModeRegistry, UnknownModeReturnsNullopt)
{
  CameraModeRegistry reg;
  EXPECT_FALSE(reg.get("nope").has_value());
  EXPECT_FALSE(reg.has("nope"));
}

TEST(CameraModeRegistry, AvailableModesSorted)
{
  CameraModeRegistry reg;
  auto names = reg.available_modes();
  ASSERT_EQ(names.size(), 2u);
  EXPECT_EQ(names[0], "dual_fisheye");
  EXPECT_EQ(names[1], "equirectangular");
}

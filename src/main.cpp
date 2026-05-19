// Copyright 2026
// SPDX-License-Identifier: Apache-2.0
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "insta360_x5_ros2/insta360_x5_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<insta360_x5_ros2::Insta360X5Node>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

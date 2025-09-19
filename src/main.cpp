// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 WimbleRobotics
// https://github.com/wimblerobotics/Sigyn

#include "roboclaw_driver/roboclaw_driver_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RoboClawDriverNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}

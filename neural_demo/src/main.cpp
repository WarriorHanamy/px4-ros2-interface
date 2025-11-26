/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "rclcpp/rclcpp.hpp"

#include <neural_demo/executor.hpp>
#include <neural_demo/mode_demo_entry.hpp>
#include <neural_demo/mode_neural_ctrl.hpp>
#include <px4_ros2/components/node_with_mode.hpp>

using DemoNodeWithModeExecutor = px4_ros2::NodeWithModeExecutor<DemoModeExecutor, ModeDemoEntry, ModeNeuralCtrl>;

static const std::string kNodeName = "neural_demo";
static const bool kEnableDebugOutput = true;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DemoNodeWithModeExecutor>(kNodeName, kEnableDebugOutput));
  rclcpp::shutdown();
  return 0;
}

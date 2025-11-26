/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
/**
 * @file mode_demo_entry.hpp
 * @brief Mode to activate execution of neural control demo entry
 * @author steven cheng
 * @date created on 2025.11.19
 * @date last modified on 2025.11.20
 */


#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <rclcpp/rclcpp.hpp>

class ModeDemoEntry : public px4_ros2::ModeBase
{
public:
  explicit ModeDemoEntry(rclcpp::Node & node)
  : ModeBase(node, Settings{"Demo Start"}.activateEvenWhileDisarmed(
        true))
  {
    _rates_setpoint = std::make_shared<px4_ros2::RatesSetpointType>(*this);
  }

  ~ModeDemoEntry() override = default;

protected:
  void updateSetpoint(float dt_s) override
  {
    // echo now time
    // const Eigen::Vector3f rate{0.0f, 0.0f, 0.0f};
    // const Eigen::Vector3f thrust{0.0f, 0.0f, -0.5f};
    // _rates_setpoint->update(rate, thrust);
  }

  void onActivate() override
  {
    //TODO 检查条件是否满足启动
    RCLCPP_INFO(node().get_logger(), "ModeDemoEntry activated");
  }

  void onDeactivate() override
  {
    RCLCPP_INFO(node().get_logger(), "ModeDemoEntry deactivated");
  }

private:
  std::shared_ptr<px4_ros2::RatesSetpointType> _rates_setpoint;
};

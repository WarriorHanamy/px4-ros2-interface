/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/vehicle_thrust_acc_setpoint.hpp>
#include <Eigen/Core>

#include <px4_ros2/common/setpoint_base.hpp>

namespace px4_ros2
{
/** \ingroup setpoint_types_experimental
 *  @{
 */

/**
 * @brief Setpoint type for direct rate control
*/
class AccRatesSetpointType : public SetpointBase
{
public:
  explicit AccRatesSetpointType(Context & context);

  ~AccRatesSetpointType() override = default;

  Configuration getConfiguration() override;
  float desiredUpdateRateHz() override {return 200.f;}

void update(
    const float & thrust_setpoint,
  const Eigen::Vector3f & rate_setpoints_frd_rad
);

private:
  rclcpp::Node & _node;
  rclcpp::Publisher<px4_msgs::msg::VehicleThrustAccSetpoint>::SharedPtr _vehicle_acc_rates_setpoint_pub;
};

/** @}*/
} /* namespace px4_ros2 */

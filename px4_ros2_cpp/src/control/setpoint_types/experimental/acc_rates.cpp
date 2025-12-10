/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <px4_ros2/control/setpoint_types/experimental/acc_rates.hpp>
#include <px4_ros2/utils/message_version.hpp>

namespace px4_ros2
{

AccRatesSetpointType::AccRatesSetpointType(Context & context)
: SetpointBase(context), _node(context.node())
{
  _vehicle_acc_rates_setpoint_pub =
    context.node().create_publisher<px4_msgs::msg::VehicleThrustAccSetpoint>(
    context.topicNamespacePrefix() + "fmu/in/vehicle_thrust_acc_setpoint" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleThrustAccSetpoint>(),
    1);
}

void AccRatesSetpointType::update(
  const float & thrust_setpoint,
  const Eigen::Vector3f & rate_setpoints_frd_rad
)
{
  onUpdate();

  px4_msgs::msg::VehicleThrustAccSetpoint sp{};
  sp.rates_sp[0] = rate_setpoints_frd_rad(0);
  sp.rates_sp[1] = rate_setpoints_frd_rad(1);
  sp.rates_sp[2] = rate_setpoints_frd_rad(2);

  sp.thrust_acc_sp = thrust_setpoint;
  sp.timestamp = 0; // Let PX4 set the timestamp
  _vehicle_acc_rates_setpoint_pub->publish(sp);
}

SetpointBase::Configuration AccRatesSetpointType::getConfiguration()
{
  Configuration config{};
  config.attitude_enabled = false;
  config.altitude_enabled = false;
  config.climb_rate_enabled = false;
  config.acceleration_enabled = false;
  config.velocity_enabled = false;
  config.position_enabled = false;
  config.offboard_mode_enabled = true;
  return config;
}
} // namespace px4_ros2

/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/multicopter/goto.hpp>
// #include <px4_ros2/control/setpoint_types/multicopter/experimental/rates.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>

class ModeNeuralCtrl : public px4_ros2::ModeBase
{
public:
  explicit ModeNeuralCtrl(rclcpp::Node & arg_node)
  : ModeBase(arg_node, Settings{"Neural Control"}),
    _activation_time(0),
    _has_goto_cmd(false)
  {
    _goto_setpoint = std::make_shared<px4_ros2::MulticopterGotoSetpointType>(*this);
    // _rates_setpoint = std::make_shared<px4_ros2::MulticopterRatesSetpointType>(*this);

    // Set up OdometryLocalPosition using px4-ros2 API
    _odometry_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

    // Subscribe to target position from fake network node
    _target_sub = _node.create_subscription<geometry_msgs::msg::PoseStamped>(
      "/neural/target_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        _target_position = Eigen::Vector3f(
          msg->pose.position.x,
          msg->pose.position.y,
          msg->pose.position.z);
        _target_timestamp = _node.get_clock()->now();
        _has_goto_cmd = true;

        RCLCPP_DEBUG_ONCE(_node.get_logger(), "Received target: [%.2f, %.2f, %.2f]",
                     _target_position.x(), _target_position.y(), _target_position.z());
      });

    // subscribe to rate setpoint from neural network node
    // _neural_rate_sub = _node.create_subscription<px4_msgs::msg::VehicleRatesSetpoint>(
    //   "/neural/rates_sp", 10,
    //   [this](const px4_msgs::msg::VehicleRatesSetpoint::SharedPtr msg) {
    //     _neural_rates_sp_msg = *msg;
    //     _neural_rate_sp_msg_timestamp = _node.get_clock()->now();
    //     _has_rate_sp = true;
    //   });

    // Subscribe to RC input for interruption detection
    _rc_interrupt_sub = _node.create_subscription<px4_msgs::msg::ManualControlSetpoint>(
      "/fmu/out/manual_control_setpoint" + px4_ros2::getMessageNameVersion<px4_msgs::msg::ManualControlSetpoint>(),
      rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg) {
        checkRCInterruption(msg);
      });

    // Subscribe to stop neural control command from any node
    _stop_neural_ctrl_sub = _node.create_subscription<std_msgs::msg::Bool>(
      "/neural/stop_control", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          RCLCPP_INFO(_node.get_logger(), "Neural: Stop control command received");
          completed(px4_ros2::Result::Success);
        }
      });

    // Load configuration
    _node.declare_parameter("neural_target_tolerance", 0.5f);
    _node.declare_parameter("neural_position_timeout", 1.0f);
    _node.declare_parameter("neural_target_timeout", 2.0f);
    _node.declare_parameter("neural_max_velocity", 2.0f);

    _target_tolerance = _node.get_parameter("neural_target_tolerance").as_double();
    _position_timeout = _node.get_parameter("neural_position_timeout").as_double();
    _target_timeout = _node.get_parameter("neural_target_timeout").as_double();
    _max_velocity = _node.get_parameter("neural_max_velocity").as_double();

  }

  ~ModeNeuralCtrl() override = default;

protected:
  void onActivate() override
  {
    _activation_time = _node.get_clock()->now().seconds();
    _has_goto_cmd = false;
    _has_rate_sp = false;
    _interrupt_triggered = false;
  }

  void onDeactivate() override
  {
    _activation_time = 0;
    _has_goto_cmd = false;
    _has_rate_sp = false;
    _interrupt_triggered = false;
    RCLCPP_INFO(_node.get_logger(), "ModeNeuralCtrl deactivated");
  }

  void updateSetpoint(float dt_s) override
  {
    const auto now = _node.get_clock()->now();

    // 2. Check RC interruption
    if (_interrupt_triggered) {
      RCLCPP_WARN(_node.get_logger(), "Neural: RC interruption detected!");
      completed(px4_ros2::Result::ModeFailureOther);
      return;
    }

    // 3. Check target data availability and validity
    if (!_has_goto_cmd) {
      RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 1000,
                          "Neural: Waiting for target...");
      return;
    }

    // Check target data timeout
    const float time_since_target = (now - _target_timestamp).seconds();
    if (time_since_target > _target_timeout) {
      RCLCPP_ERROR(_node.get_logger(), "Neural: Target data timeout (%.1fs)", time_since_target);
      completed(px4_ros2::Result::ModeFailureOther);
      return;
    }

    // 4. Generate goto setpoint to target (stop command is handled by /neural/stop_control topic with bool message)
    generateGotoSetpoint();
  }

private:
  std::shared_ptr<px4_ros2::MulticopterGotoSetpointType> _goto_setpoint; // just for placeholder
  // std::shared_ptr<px4_ros2::MulticopterRatesSetpointType> _rates_setpoint;

  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _target_sub;
  rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr _rc_interrupt_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _stop_neural_ctrl_sub;

  
  // subscribe rate setpoint from onboard neural network node
  // rclcpp::Subscription<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr _neural_rates_sub;

  // Odometry
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _odometry_local_position;

  // *** mode state *** //
  double _activation_time;
  // Flags
  bool _has_goto_cmd;
  bool _has_rate_sp;
  bool _interrupt_triggered = false;


  // *** neural control *** //
  px4_msgs::msg::VehicleRatesSetpoint _neural_rates_sp_msg;
  float _neural_rate_sp_msg_timeout = 0.1f;
  rclcpp::Time _neural_rate_sp_msg_timestamp;


  // *** placeholder goto control *** //
  // State variables
  Eigen::Vector3f _target_position;
  rclcpp::Time _target_timestamp;

  // Configuration
  float _target_tolerance;
  float _position_timeout;
  float _target_timeout;
  float _max_velocity;

  void rate_setpoint_cb(const px4_msgs::msg::VehicleRatesSetpoint::SharedPtr msg)
  {
    _neural_rates_sp_msg = *msg;
    _neural_rate_sp_msg_timestamp = _node.get_clock()->now();
    _has_rate_sp = true;
  }

  
  // Get current position from OdometryLocalPosition API
  Eigen::Vector3f getCurrentPosition() const
  {
    return _odometry_local_position->positionNed();
  }

  // Get current velocity from OdometryLocalPosition API
  Eigen::Vector3f getCurrentVelocity() const
  {
    return _odometry_local_position->velocityNed();
  }

  // Check if position data is valid
  bool isPositionValid() const
  {
    return _odometry_local_position->positionXYValid() && _odometry_local_position->positionZValid();
  }

  // Goto setpoint generation using px4-ros2 OdometryLocalPosition API
  void generateGotoSetpoint()
  {
    if (!_has_goto_cmd) return;

    // Check if position data is available and valid
    if (!isPositionValid()) {
      return;
    }

    _goto_setpoint->update(
        _target_position,           // position target
        std::nullopt,               // no heading control
        _max_velocity,              // max horizontal speed
        _max_velocity,              // max vertical speed
        std::nullopt                // max heading rate (optional)
    );
  }

  // void generateRateSetpoint()
  // {
  //   if (!_has_rate_sp) return;

  //   const float time_since_rate_sp = (_node.get_clock()->now() - _neural_rate_sp_msg_timestamp).seconds();
  //   if (time_since_rate_sp > _neural_rate_sp_msg_timeout) {
  //     RCLCPP_ERROR(_node.get_logger(), "Neural: Rate setpoint data timeout (%.2fs)", time_since_rate_sp);
  //     completed(px4_ros2::Result::ModeFailureOther);
  //     return;
  //   }

  //   const Eigen::Vector3f rates{
  //     _neural_rates_sp_msg.roll,
  //     _neural_rates_sp_msg.pitch,
  //     _neural_rates_sp_msg.yaw
  //   };
  //   const Eigen::Vector3f thrust{
  //     0.0f,
  //     0.0f,
  //     _neural_rates_sp_msg.thrust
  //   };
  //   _rate_setpoint->update(rates, thrust);
  // }

  // RC interruption detection (temporarily disabled for debugging)
  void checkRCInterruption(const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg)
  {
    // if this mode is not active, ignore
    if (!isActive()) {
      return;
    }
    // Check if sticks are moving (interrupt condition)
    if (msg->sticks_moving) {
      if (!_interrupt_triggered) {
        RCLCPP_WARN(_node.get_logger(), "RC sticks movement detected - interrupting Neural control");
        _interrupt_triggered = true;
      }
    }
  }
};
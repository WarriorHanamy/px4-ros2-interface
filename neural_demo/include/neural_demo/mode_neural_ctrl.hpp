/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/multicopter/goto.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>

#define USE_GOTO_CTRL
#define USE_RATES_CTRL

class ModeNeuralCtrl : public px4_ros2::ModeBase
{
public:
  explicit ModeNeuralCtrl(rclcpp::Node &arg_node)
      : ModeBase(arg_node, Settings{"Neural Control"})
  {
    // Initialize state
    _activation_time = {0};
    _has_goto_cmd = false;
    _has_rates_sp = false;

    // Load configuration
    _node.declare_parameter("goto_cmd_timeout", 2.0f);
    _node.declare_parameter("goto_cmd_max_velocity", 2.0f);

    _goto_cmd_timeout = _node.get_parameter("goto_cmd_timeout").as_double();
    _goto_cmd_max_velocity = _node.get_parameter("goto_cmd_max_velocity").as_double();

    // Subscriber and publishers
    _goto_setpoint = std::make_shared<px4_ros2::MulticopterGotoSetpointType>(*this);
    _rates_setpoint = std::make_shared<px4_ros2::RatesSetpointType>(*this);
    _odometry_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
    _manual_control_input = std::make_shared<px4_ros2::ManualControlInput>(*this);

    // Subscribe to target position from fake network node
    _goto_cmd_sub = _node.create_subscription<geometry_msgs::msg::PoseStamped>(
        "/neural/target_pose", 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
          _goto_cmd_target = Eigen::Vector3f(
              msg->pose.position.x,
              msg->pose.position.y,
              msg->pose.position.z);
          _goto_cmd_timestamp = _node.get_clock()->now();
          _has_goto_cmd = true;
          RCLCPP_DEBUG_ONCE(_node.get_logger(), "Received goto cmd target: [%.2f, %.2f, %.2f]",
                            _goto_cmd_target.x(), _goto_cmd_target.y(), _goto_cmd_target.z());
        });

    // subscribe to rate setpoint from neural network node
    _neural_rates_sub = _node.create_subscription<px4_msgs::msg::VehicleRatesSetpoint>(
        "/neural/rates_sp", 10,
        [this](const px4_msgs::msg::VehicleRatesSetpoint::SharedPtr msg)
        {
          _rates_sp_msg = *msg;
          _rates_sp_msg_timestamp = _node.get_clock()->now();
          _has_rates_sp = true;
        });

    // Subscribe to stop neural control command from any node
    _stop_neural_ctrl_sub = _node.create_subscription<std_msgs::msg::Bool>(
        "/neural/stop_control", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
          if (msg->data)
          {
            RCLCPP_INFO(_node.get_logger(), "Neural: Stop control command received");
            completed(px4_ros2::Result::Success);
          }
        });

      _start_neural_ctrl_pub = _node.create_publisher<std_msgs::msg::Bool>(
            "/neural/mode_neural_ctrl", 10);
    
  }

  ~ModeNeuralCtrl() override = default;

protected:
  void onActivate() override
  {
    _activation_time = _node.get_clock()->now().seconds();
    _has_goto_cmd = false;
    _has_rates_sp = false;
    _interrupt_triggered = false;

    // Notify neural network node that neural control mode is activated
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    _start_neural_ctrl_pub->publish(msg);
  }

  void onDeactivate() override
  {
    _activation_time = 0;
    _has_goto_cmd = false;
    _has_rates_sp = false;
    _interrupt_triggered = false;
    RCLCPP_INFO(_node.get_logger(), "ModeNeuralCtrl deactivated");
  }

  void updateSetpoint(float dt_s) override
  {
    // Check RC interruption
    if (_manual_control_input->sticks_moving())
    {
      RCLCPP_WARN(_node.get_logger(), "Neural: RC interruption detected!");
      completed(px4_ros2::Result::ModeFailureOther);
      return;
    }

#ifdef USE_GOTO_CTRL
    generateGotoSetpoint();
#endif

#ifdef USE_RATES_CTRL
    generateRatesSetpoint();
#endif
  }

private:
  std::shared_ptr<px4_ros2::ManualControlInput> _manual_control_input;   // for RC interruption detection
  std::shared_ptr<px4_ros2::MulticopterGotoSetpointType> _goto_setpoint; // just for placeholder
  std::shared_ptr<px4_ros2::RatesSetpointType> _rates_setpoint;          // accept rates setpoint from neural network node

  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _goto_cmd_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _stop_neural_ctrl_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr _neural_rates_sub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _start_neural_ctrl_pub;

  // Odometry
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _odometry_local_position;

  // *** mode state *** //
  double _activation_time;
  // Flags
  bool _has_goto_cmd;
  bool _has_rates_sp;
  bool _interrupt_triggered = false;

  // *** neural control *** //
  px4_msgs::msg::VehicleRatesSetpoint _rates_sp_msg;
  float _rates_sp_msg_timeout = 0.05f;
  rclcpp::Time _rates_sp_msg_timestamp;

  // *** placeholder goto control *** //
  // State variables
  Eigen::Vector3f _goto_cmd_target;
  rclcpp::Time _goto_cmd_timestamp;

  // Configuration
  float _goto_cmd_timeout;
  float _goto_cmd_max_velocity;

  // Check if position data is valid
  inline bool isPositionValid() const
  {
    return _odometry_local_position->positionXYValid() && _odometry_local_position->positionZValid();
  }

  // Goto setpoint generation using px4-ros2 OdometryLocalPosition API
  inline void generateGotoSetpoint()
  {
    const auto now = _node.get_clock()->now();

    // Check target data availability and validity
    if (!_has_goto_cmd)
    {
      RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 1000,
                           "Neural: Waiting for target...");
      return;
    }

    // Check target data timeout
    const float time_since_target = (now - _goto_cmd_timestamp).seconds();
    if (time_since_target > _goto_cmd_timeout)
    {
      RCLCPP_ERROR(_node.get_logger(),
                   "Neural: Target data timeout (%.1fs)", time_since_target);
      completed(px4_ros2::Result::ModeFailureOther);
      return;
    }

    // Check if position data is available and valid
    if (!isPositionValid())
    {
      RCLCPP_ERROR(_node.get_logger(), "Neural: Position data not available or invalid");
      completed(px4_ros2::Result::ModeFailureOther);
      return;
    }

    _goto_setpoint->update(
        _goto_cmd_target,       // position target
        std::nullopt,           // no heading control
        _goto_cmd_max_velocity, // max horizontal speed
        _goto_cmd_max_velocity, // max vertical speed
        std::nullopt            // max heading rate (optional)
    );
  }

  inline void generateRatesSetpoint()
  {
    const auto now = _node.get_clock()->now();

    // Check target data availability and validity
    if (!_has_rates_sp)
    {
      RCLCPP_INFO_THROTTLE(_node.get_logger(), *_node.get_clock(), 1000,
                           "Neural: Waiting for rate setpoint...");
      return;
    }

    // Check rate setpoint data timeout
    const float time_since_rates_sp = (now - _rates_sp_msg_timestamp).seconds();
    if (time_since_rates_sp > _rates_sp_msg_timeout)
    {
      RCLCPP_ERROR(_node.get_logger(),
                   "Neural: Rate setpoint data timeout (%.1fs)", time_since_rates_sp);
      completed(px4_ros2::Result::ModeFailureOther);
      return;
    }

    const Eigen::Vector3f rates{
        _rates_sp_msg.roll,
        _rates_sp_msg.pitch,
        _rates_sp_msg.yaw};

    // For multicopters thrust_body[0] and thrust[1] are usually 0 and thrust[2] is the negative throttle demand.
    const Eigen::Vector3f thrust{
        _rates_sp_msg.thrust_body[0],
        _rates_sp_msg.thrust_body[1],
        _rates_sp_msg.thrust_body[2]};
    _rates_setpoint->update(rates, thrust);
  }
};
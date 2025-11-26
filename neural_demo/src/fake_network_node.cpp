/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_ros2/utils/message_version.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Eigen>
#include <fstream>
#include <string>

/**
 * @brief Fake Network Node for Neural Control Demo
 *
 * This node simulates a network-based navigation system by publishing
 * target waypoints to the Neural control mode. It monitors RC input
 * to start/stop publishing targets based on button triggers.
 */
class FakeNetworkNode : public rclcpp::Node
{
public:
  explicit FakeNetworkNode() : Node("fake_network_node"), _active(false), _current_waypoint(0)
  {
    // Declare parameters
    declare_parameter("config_file", "");
    declare_parameter("publish_rate", 10.0);

    // Load configuration
    loadConfig();

    // Publishers
    _target_pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>("/neural/target_pose", 10);
    _stop_control_pub = create_publisher<std_msgs::msg::Bool>("/neural/stop_control", 10);

    // Subscriptions
    _rc_sub = create_subscription<px4_msgs::msg::ManualControlSetpoint>(
      "/fmu/out/manual_control_setpoint" + px4_ros2::getMessageNameVersion<px4_msgs::msg::ManualControlSetpoint>(),
      rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg) {
        handleRCInput(msg);
      });

    _vehicle_position_sub = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position" + px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleLocalPosition>(),
      rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        if (msg->xy_valid && msg->z_valid && !msg->dead_reckoning) {
          _current_vehicle_pos = Eigen::Vector3f(msg->x, msg->y, msg->z);
          _position_valid = true;
        } else {
          _position_valid = false;
        }
      });

    // Timer for publishing targets
    double publish_rate = get_parameter("publish_rate").as_double();
    _publish_timer = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
      [this]() { publishTarget(); });

    RCLCPP_INFO(get_logger(), "FakeNetworkNode initialized with %zu waypoints", _waypoints.size());
    RCLCPP_INFO(get_logger(), "Waiting for RC Button 1024 to start publishing targets");
  }

private:
  // Components
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _target_pose_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _stop_control_pub;
  rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr _rc_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _vehicle_position_sub;
  rclcpp::TimerBase::SharedPtr _publish_timer;

  // State
  bool _active;
  size_t _current_waypoint;
  Eigen::Vector3f _current_vehicle_pos;
  bool _position_valid = false;

  // Configuration
  std::vector<Eigen::Vector3f> _waypoints;
  float _target_tolerance;
  float _max_velocity;

  void loadConfig()
  {
    std::string config_file = get_parameter("config_file").as_string();

    // Default waypoints if no config file
    if (config_file.empty() || !std::ifstream(config_file).good()) {
      RCLCPP_WARN(get_logger(), "No config file provided or file not found, using default waypoints");

      // Default waypoints in body frame (meters)
      _waypoints = {
        Eigen::Vector3f(5.0f, 0.0f, -2.0f),   // Forward 5m, altitude 2m
        Eigen::Vector3f(5.0f, 3.0f, -2.0f),   // Right 3m
        Eigen::Vector3f(0.0f, 3.0f, -2.0f),   // Back to original X, right 3m
        Eigen::Vector3f(0.0f, 0.0f, -2.0f)    // Back to start position
      };

      _target_tolerance = 0.5f;
      _max_velocity = 2.0f;
    } else {
      // Load from YAML file
      try {
        YAML::Node config = YAML::LoadFile(config_file);

        if (config["demo_targets"]["waypoints"]) {
          for (const auto& wp : config["demo_targets"]["waypoints"]) {
            if (wp.size() >= 3) {
              _waypoints.push_back(Eigen::Vector3f(
                wp[0].as<float>(),
                wp[1].as<float>(),
                wp[2].as<float>()
              ));
            }
          }
        }

        if (config["failsafe_config"]) {
          _target_tolerance = config["failsafe_config"]["target_tolerance"].as<float>(0.5f);
          _max_velocity = config["failsafe_config"]["max_velocity"].as<float>(2.0f);
        }

        RCLCPP_INFO(get_logger(), "Loaded %zu waypoints from config file: %s",
                   _waypoints.size(), config_file.c_str());
      } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to load config file: %s", e.what());
        // Use defaults
        _waypoints = {Eigen::Vector3f(5.0f, 0.0f, -2.0f)};
        _target_tolerance = 0.5f;
        _max_velocity = 2.0f;
      }
    }

    // Load parameters (override config file if set)
    declare_parameter("fake_target_tolerance", _target_tolerance);
    declare_parameter("fake_max_velocity", _max_velocity);

    _target_tolerance = get_parameter("fake_target_tolerance").as_double();
    _max_velocity = get_parameter("fake_max_velocity").as_double();

    RCLCPP_INFO(get_logger(), "Configuration: tolerance=%.1f, max_vel=%.1f",
               _target_tolerance, _max_velocity);
  }

  void handleRCInput(const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg)
  {
    // Check button press (Button=1024 -> aux1 > 0.8)
    bool button_pressed = (msg->buttons == 1024);
    // debug print msg->buttons
    // RCLCPP_INFO(get_logger(), "RC Button: %d", msg->buttons);

    if (button_pressed && !_active) {
      // Button pressed - activate target publishing
      _active = true;
      _current_waypoint = 0;
      RCLCPP_WARN(get_logger(), "RC Button 1024 detected - Starting fake network navigation");

      // Log the mission waypoints
      RCLCPP_INFO(get_logger(), "Mission waypoints (%zu total):", _waypoints.size());
      for (size_t i = 0; i < _waypoints.size(); ++i) {
        const auto& wp = _waypoints[i];
        RCLCPP_INFO(get_logger(), "  WP%zu: [%.1f, %.1f, %.1f]", i+1, wp.x(), wp.y(), wp.z());
      }
    }
  }

  void publishTarget()
  {
    if (!_active || _waypoints.empty()) {
      return;
    }

    if (_current_waypoint >= _waypoints.size()) {
      // Mission complete - stop publishing
      _active = false;
      RCLCPP_INFO(get_logger(), "All waypoints completed - stopping target publishing");
      return;
    }

    // Check if current waypoint is reached
    if (_position_valid && isCurrentWaypointReached()) {
      _current_waypoint++;

      if (_current_waypoint >= _waypoints.size()) {
        // Mission complete - publish stop control command
        _active = false;
        RCLCPP_INFO(get_logger(), "Final waypoint reached - stopping neural control!");

        // Publish stop control command
        std_msgs::msg::Bool stop_msg;
        stop_msg.data = true;
        _stop_control_pub->publish(stop_msg);

        return;
      } else {
        RCLCPP_INFO(get_logger(), "Waypoint %zu reached, moving to waypoint %zu",
                   _current_waypoint, _current_waypoint + 1);
      }
    }

    // Publish current target
    const auto& target = _waypoints[_current_waypoint];

    geometry_msgs::msg::PoseStamped target_msg;
    target_msg.header.stamp = now();
    target_msg.header.frame_id = "base_link"; // Body frame

    target_msg.pose.position.x = target.x();
    target_msg.pose.position.y = target.y();
    target_msg.pose.position.z = target.z();

    // Default orientation (no yaw control)
    target_msg.pose.orientation.w = 1.0f;
    target_msg.pose.orientation.x = 0.0f;
    target_msg.pose.orientation.y = 0.0f;
    target_msg.pose.orientation.z = 0.0f;

    RCLCPP_DEBUG(get_logger(), "Publishing target [%.2f, %.2f, %.2f]",
                 target.x(), target.y(), target.z());

    _target_pose_pub->publish(target_msg);
  }

  bool isCurrentWaypointReached() const
  {
    if (!_position_valid || _current_waypoint >= _waypoints.size()) {
      return false;
    }

    const float distance = (_current_vehicle_pos - _waypoints[_current_waypoint]).norm();
    return distance < _target_tolerance;
  }

  };

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeNetworkNode>());
  rclcpp::shutdown();
  return 0;
}
/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

// Include registration-related header
#include "registration.hpp"
// Include header for health check and arming check components
#include "px4_ros2/components/health_and_arming_checks.hpp"
// Include header for message version utilities
#include "px4_ros2/utils/message_version.hpp"

// Include assertion library for debugging
#include <cassert>
// Include std::move and other utility functions
#include <utility>

// Use std::chrono_literals namespace, allowing literal values like 4s for time intervals
using namespace std::chrono_literals;

// Enter px4_ros2 namespace
namespace px4_ros2
{

// Constructor: initialize health check and arming check objects
HealthAndArmingChecks::HealthAndArmingChecks(
  rclcpp::Node & node, CheckCallback check_callback,
  const std::string & topic_namespace_prefix)
// Use initializer list to initialize member variables
: _node(node), _registration(std::make_shared<Registration>(node, topic_namespace_prefix)),
  _check_callback(std::move(check_callback))
{
  // Create publisher for arming check reply message, used to send reply to FMU
  _arming_check_reply_pub = _node.create_publisher<px4_msgs::msg::ArmingCheckReply>(
    topic_namespace_prefix + "fmu/in/arming_check_reply" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::ArmingCheckReply>(),
    1);

  // Create subscriber for arming check request message, used to receive requests from FMU
  _arming_check_request_sub = _node.create_subscription<px4_msgs::msg::ArmingCheckRequest>(
    topic_namespace_prefix + "fmu/out/arming_check_request" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::ArmingCheckRequest>(),
    rclcpp::QoS(1).best_effort(),
    // Lambda function handles received arming check requests
    [this](px4_msgs::msg::ArmingCheckRequest::UniquePtr msg) {

      // Debug info: log received arming check request (printed only once)
      RCLCPP_DEBUG_ONCE(
        _node.get_logger(), "Arming check request (id=%i, only printed once)",
        msg->request_id);

      // Check if already registered
      if (_registration->registered()) {
        // Create arming check reply message
        px4_msgs::msg::ArmingCheckReply reply{};
        // Set registration ID
        reply.registration_id = _registration->armingCheckId();
        // Set request ID, same as original request
        reply.request_id = msg->request_id;
        // Default to true for can_arm_and_run
        reply.can_arm_and_run = true;

        // Create health check and arming check reporter object
        HealthAndArmingCheckReporter reporter(reply);
        // Call user-provided check callback function
        _check_callback(reporter);

        // Fill arming check reply information using mode requirements
        _mode_requirements.fillArmingCheckReply(reply);

        // Set timestamp to 0, let PX4 set actual timestamp
        reply.timestamp = 0; // Let PX4 set the timestamp
        // Publish arming check reply message
        _arming_check_reply_pub->publish(reply);

        // Check if our registration id is still valid. If not, we still send the reply,
        // as it might be flagged as unresponsive, but we don't update the timer check.
        // The first request might not have the bit set yet.
        if (_first_request || (msg->valid_registrations_mask & (1U << reply.registration_id))) {
          // Mark check as triggered
          _check_triggered = true;
        } else {
          // Error: registration ID marked as invalid (printed only once)
          RCLCPP_ERROR_ONCE(
            _node.get_logger(), "Registration id %i is flagged as invalid (only printed once)",
            reply.registration_id);
        }
        // Mark first request as processed
        _first_request = false;

      } else {
        // Debug: not yet registered
        RCLCPP_DEBUG(_node.get_logger(), "...not registered yet");
      }
    });

  // Create watchdog timer that executes watchdogTimerUpdate function every 4 seconds
  _watchdog_timer =
    _node.create_wall_timer(4s, [this] {watchdogTimerUpdate();});
}

// Override the registration object's method
void HealthAndArmingChecks::overrideRegistration(const std::shared_ptr<Registration> & registration)
{
  // Assert: ensure not yet registered before overriding
  assert(!_registration->registered());
  // Replace existing registration object with new one
  _registration = registration;
}

// Perform registration
bool HealthAndArmingChecks::doRegister(const std::string & name)
{
  // Assert: ensure not already registered before registering
  assert(!_registration->registered());
  // Create registration settings struct
  RegistrationSettings settings{};
  // Set component name
  settings.name = name;
  // Set whether to register arming check function to true
  settings.register_arming_check = true;
  // Mark first request as true for initialization
  _first_request = true;
  // Perform registration, return whether successful
  return _registration->doRegister(settings);
}

// Watchdog timer update callback function
void HealthAndArmingChecks::watchdogTimerUpdate()
{
  // Check if already registered
  if (_registration->registered()) {
    // If check not triggered and configured to shut down on timeout
    if (!_check_triggered && _shutdown_on_timeout) {
      // Gracefully shutdown ROS 2
      rclcpp::shutdown();
      // Throw exception indicating timeout exit
      throw Exception(
              "Timeout, no request received from FMU, exiting (this can happen on FMU reboots)");
    }

    // Reset check triggered flag for next cycle
    _check_triggered = false;

  } else {
    // When not registered, avoid false positives by setting check triggered flag to true
    _check_triggered = true;
  }
}

// Exit px4_ros2 namespace
} // namespace px4_ros2

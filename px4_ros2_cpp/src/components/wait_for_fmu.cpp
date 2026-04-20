/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

// Include header with function declaration for waiting for FMU
#include <px4_ros2/components/wait_for_fmu.hpp>
// Include definition of vehicle status message
#include <px4_msgs/msg/vehicle_status.hpp>
// Include header for message version utilities
#include <px4_ros2/utils/message_version.hpp>

// Enter px4_ros2 namespace
namespace px4_ros2
{

// waitForFMU function: Wait for FMU (Flight Management Unit) to connect and become ready
// bool: return type, true表示成功连接，false表示超时或失败 → true means successful connection, false means timeout or failure
// rclcpp::Node & node: reference to ROS 2 node for accessing ROS 2 functionality
// const rclcpp::Duration & timeout: reference to timeout duration constant
// const std::string & topic_namespace_prefix: reference to topic namespace prefix constant, used to locate FMU's message topics
// Function description:
//   1. Subscribe to vehicle status messages sent by FMU
//   2. Wait to receive the first message (indicates FMU is connected)
//   3. Return true if message received within timeout, otherwise return false
//   4. This is a blocking function that waits until message received or timeout
bool waitForFMU(
  rclcpp::Node & node, const rclcpp::Duration & timeout,
  const std::string & topic_namespace_prefix)
{
  // RCLCPP_DEBUG(): ROS 2 debug logging function
  // Function: output debug information for developers to track program execution
  RCLCPP_DEBUG(node.get_logger(), "Waiting for FMU...");

  // const rclcpp::Subscription<...>::SharedPtr:
  //   - const: constant, cannot be modified once initialized
  //   - rclcpp::Subscription<px4_msgs::msg::VehicleStatus>:
  //     * Subscription: subscriber object that receives messages
  //     * <px4_msgs::msg::VehicleStatus>: template parameter specifying the message type to subscribe to
  //   - SharedPtr: shared pointer, automatically manages memory
  // Function: create a subscriber object to receive vehicle status messages
  const rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub =
    // node.create_subscription<>(): create subscriber
    // Parameter 1: topic name
    //   - topic_namespace_prefix: prefix
    //   + "fmu/out/vehicle_status": relative topic path
    //   + px4_ros2::getMessageNameVersion<>(): get message version suffix
    // Parameter 2: QoS (Quality of Service) settings
    //   - rclcpp::QoS(1): set queue size to 1
    //   - .best_effort(): set to best-effort delivery mode (no guarantee of arrival, but low latency)
    // Parameter 3: callback function (using lambda expression)
    //   - [](px4_msgs::msg::VehicleStatus::UniquePtr msg) {}:
    //     * called when message received
    //     * msg: contains received message data
    //     * {}: function body is empty, because we use take() method to receive messages later
    node.create_subscription<px4_msgs::msg::VehicleStatus>(
    topic_namespace_prefix + "fmu/out/vehicle_status" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleStatus>(), rclcpp::QoS(
      1).best_effort(),
    [](px4_msgs::msg::VehicleStatus::UniquePtr msg) {});

  // rclcpp::WaitSet: wait set
  // Function: used to efficiently wait for multiple subscribers' message arrival
  // Principle: use operating system's waiting mechanism instead of busy polling
  rclcpp::WaitSet wait_set;

  // wait_set.add_subscription(vehicle_status_sub):
  // Function: add subscriber to wait set
  // so that wait_set monitors this subscriber's messages
  wait_set.add_subscription(vehicle_status_sub);

  // bool got_message = false:
  // flag variable recording whether message is successfully received
  bool got_message = false;

  // auto start_time = node.now():
  // auto: automatic type deduction
  // node.now(): get current time
  // start_time: record loop start time for timeout calculation
  auto start_time = node.now();

  // while (!got_message):
  // loop condition: continue looping as long as message not received
  // ! means logical NOT
  while (!got_message) {
    // auto now = node.now(): get current time
    auto now = node.now();

    // if (now >= start_time + timeout):
    // check if timeout has occurred
    // >= means greater than or equal
    // Function: if current time exceeds start time + timeout, indicates timeout
    if (now >= start_time + timeout) {
      // break: exit loop
      // Function: stop waiting and return failure
      break;
    }

    // auto wait_ret = wait_set.wait(...):
    // wait_set.wait(): wait for subscriber to receive message
    // Parameter: remaining wait time (microseconds)
    //   - (timeout - (now - start_time)): remaining timeout time
    //   - .to_chrono<std::chrono::microseconds>(): convert to std::chrono format microseconds
    // wait_ret: wait result object
    auto wait_ret = wait_set.wait(
      (timeout - (now - start_time)).to_chrono<std::chrono::microseconds>());

    // if (wait_ret.kind() == rclcpp::WaitResultKind::Ready):
    // check wait result
    // kind(): get the result type of wait
    // WaitResultKind::Ready: indicates a subscriber has received a message
    if (wait_ret.kind() == rclcpp::WaitResultKind::Ready) {
      // px4_msgs::msg::VehicleStatus msg:
      // create a vehicle status message object to store received data
      px4_msgs::msg::VehicleStatus msg;

      // rclcpp::MessageInfo info:
      // message metadata object storing timestamp, sender, etc.
      rclcpp::MessageInfo info;

      // if (vehicle_status_sub->take(msg, info)):
      // take(): extract a message from subscriber's message queue
      // Parameter 1: msg - object storing message data
      // Parameter 2: info - object storing message metadata
      // Return value: bool, true means message successfully extracted, false means no message
      // Function: try to extract message from queue
      if (vehicle_status_sub->take(msg, info)) {
        // got_message = true: set flag to true indicating message successfully received
        got_message = true;

      } else {
        // If take() fails (although wait() indicates message ready, but cannot extract)
        // output debug information explaining the situation
        RCLCPP_DEBUG(node.get_logger(), "no VehicleStatus message received");
      }

    } else {
      // If not Ready state, indicates a timeout occurred
      // output debug information
      RCLCPP_DEBUG(node.get_logger(), "timeout while waiting for FMU");
    }
  }

  // wait_set.remove_subscription(vehicle_status_sub):
  // remove subscriber from wait set
  // clean up resources, release subscriber from waiting
  wait_set.remove_subscription(vehicle_status_sub);

  // return got_message:
  // return result
  // true: successfully received message (FMU connected and ready)
  // false: timeout or failure
  return got_message;
}

// End px4_ros2 namespace
} // namespace px4_ros2

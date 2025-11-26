/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

// 包含注册相关的头文件
#include "registration.hpp"
// 包含健康检查和解锁检查的头文件
#include "px4_ros2/components/health_and_arming_checks.hpp"
// 包含消息版本工具的头文件
#include "px4_ros2/utils/message_version.hpp"

// 包含断言库用于调试
#include <cassert>
// 包含std::move等工具函数
#include <utility>

// 使用std::chrono_literals命名空间，允许使用字面量如4s表示时间间隔
using namespace std::chrono_literals;

// 进入px4_ros2命名空间
namespace px4_ros2
{

// 构造函数：初始化健康检查和解锁检查对象
HealthAndArmingChecks::HealthAndArmingChecks(
  rclcpp::Node & node, CheckCallback check_callback,
  const std::string & topic_namespace_prefix)
// 使用初始化列表初始化成员变量
: _node(node), _registration(std::make_shared<Registration>(node, topic_namespace_prefix)),
  _check_callback(std::move(check_callback))
{
  // 创建解锁检查回复消息的发布者，用于发送回复给FMU
  _arming_check_reply_pub = _node.create_publisher<px4_msgs::msg::ArmingCheckReply>(
    topic_namespace_prefix + "fmu/in/arming_check_reply" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::ArmingCheckReply>(),
    1);

  // 创建解锁检查请求消息的订阅者，用于接收来自FMU的请求
  _arming_check_request_sub = _node.create_subscription<px4_msgs::msg::ArmingCheckRequest>(
    topic_namespace_prefix + "fmu/out/arming_check_request" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::ArmingCheckRequest>(),
    rclcpp::QoS(1).best_effort(),
    // Lambda函数处理接收到的解锁检查请求
    [this](px4_msgs::msg::ArmingCheckRequest::UniquePtr msg) {

      // 调试信息：记录接收到的解锁检查请求（仅打印一次）
      RCLCPP_DEBUG_ONCE(
        _node.get_logger(), "Arming check request (id=%i, only printed once)",
        msg->request_id);

      // 检查是否已经注册
      if (_registration->registered()) {
        // 创建解锁检查回复消息
        px4_msgs::msg::ArmingCheckReply reply{};
        // 设置注册ID
        reply.registration_id = _registration->armingCheckId();
        // 设置请求ID，与原请求相同
        reply.request_id = msg->request_id;
        // 默认设置可以解锁并运行为true
        reply.can_arm_and_run = true;

        // 创建健康检查和解锁检查报告者对象
        HealthAndArmingCheckReporter reporter(reply);
        // 调用用户提供的检查回调函数
        _check_callback(reporter);

        // 使用模式要求填充解锁检查回复信息
        _mode_requirements.fillArmingCheckReply(reply);

        // 设置时间戳为0，让PX4设置实际时间戳
        reply.timestamp = 0; // Let PX4 set the timestamp
        // 发布解锁检查回复消息
        _arming_check_reply_pub->publish(reply);

        // Check if our registration id is still valid. If not, we still send the reply,
        // as it might be flagged as unresponsive, but we don't update the timer check.
        // The first request might not have the bit set yet.
        if (_first_request || (msg->valid_registrations_mask & (1U << reply.registration_id))) {
          // 标记检查已被触发
          _check_triggered = true;
        } else {
          // 错误信息：注册ID被标记为无效（仅打印一次）
          RCLCPP_ERROR_ONCE(
            _node.get_logger(), "Registration id %i is flagged as invalid (only printed once)",
            reply.registration_id);
        }
        // 标记第一次请求已处理
        _first_request = false;

      } else {
        // 调试信息：还未注册
        RCLCPP_DEBUG(_node.get_logger(), "...not registered yet");
      }
    });

  // 创建看门狗定时器，每4秒执行一次watchdogTimerUpdate函数
  _watchdog_timer =
    _node.create_wall_timer(4s, [this] {watchdogTimerUpdate();});
}

// 重写注册对象的方法
void HealthAndArmingChecks::overrideRegistration(const std::shared_ptr<Registration> & registration)
{
  // 断言：确保在重写之前还未注册
  assert(!_registration->registered());
  // 使用新的注册对象替换现有的注册对象
  _registration = registration;
}

// 执行注册的方法
bool HealthAndArmingChecks::doRegister(const std::string & name)
{
  // 断言：确保在注册之前还未注册过
  assert(!_registration->registered());
  // 创建注册设置结构体
  RegistrationSettings settings{};
  // 设置组件的名称
  settings.name = name;
  // 设置是否注册解锁检查功能为true
  settings.register_arming_check = true;
  // 标记第一次请求为true，用于初始化
  _first_request = true;
  // 执行注册，返回是否成功
  return _registration->doRegister(settings);
}

// 看门狗定时器更新的回调函数
void HealthAndArmingChecks::watchdogTimerUpdate()
{
  // 检查是否已经注册
  if (_registration->registered()) {
    // 如果检查未被触发且配置了超时时关闭
    if (!_check_triggered && _shutdown_on_timeout) {
      // 优雅地关闭ROS 2
      rclcpp::shutdown();
      // 抛出异常，提示超时退出
      throw Exception(
              "Timeout, no request received from FMU, exiting (this can happen on FMU reboots)");
    }

    // 重置检查触发标志，为下一个周期做准备
    _check_triggered = false;

  } else {
    // 当未注册时，避免误报，将检查触发标志设置为true
    _check_triggered = true;
  }
}

// 退出px4_ros2命名空间
} // namespace px4_ros2

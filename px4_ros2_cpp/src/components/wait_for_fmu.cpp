/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

// 包含等待FMU的头文件（函数声明）
#include <px4_ros2/components/wait_for_fmu.hpp>
// 包含无人机状态消息的定义
#include <px4_msgs/msg/vehicle_status.hpp>
// 包含消息版本工具的头文件
#include <px4_ros2/utils/message_version.hpp>

// 进入px4_ros2命名空间
namespace px4_ros2
{

// waitForFMU 函数：等待FMU（飞行管理单元）连接并就绪
// bool：返回类型，true表示成功连接，false表示超时或失败
// rclcpp::Node & node：ROS 2节点的引用，用于访问ROS 2功能
// const rclcpp::Duration & timeout：超时时间常量引用
// const std::string & topic_namespace_prefix：主题前缀常量引用，用于找到FMU的消息主题
// 功能说明：
//   1. 订阅FMU发送的车辆状态消息
//   2. 等待接收到第一条消息（表示FMU已连接）
//   3. 如果在超时时间内收到消息返回true，否则返回false
//   4. 这是一个阻塞函数，会一直等待直到收到消息或超时
bool waitForFMU(
  rclcpp::Node & node, const rclcpp::Duration & timeout,
  const std::string & topic_namespace_prefix)
{
  // RCLCPP_DEBUG()：ROS 2调试日志函数
  // 功能：输出调试信息，方便开发者跟踪程序执行
  RCLCPP_DEBUG(node.get_logger(), "Waiting for FMU...");
  
  // const rclcpp::Subscription<...>::SharedPtr：
  //   - const：常量，一旦初始化不能修改
  //   - rclcpp::Subscription<px4_msgs::msg::VehicleStatus>：
  //     * Subscription：订阅者对象，接收消息
  //     * <px4_msgs::msg::VehicleStatus>：模板参数，指定订阅的消息类型
  //   - SharedPtr：共享指针，自动管理内存
  // 功能：创建一个订阅者对象，用于接收车辆状态消息
  const rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub =
    // node.create_subscription<>()：创建订阅者
    // 参数1：主题名称
    //   - topic_namespace_prefix：前缀
    //   + "fmu/out/vehicle_status"：相对主题路径
    //   + px4_ros2::getMessageNameVersion<>()：获取消息版本后缀
    // 参数2：QoS（服务质量）设置
    //   - rclcpp::QoS(1)：设置队列大小为1
    //   - .best_effort()：设置为best-effort传递方式（不保证必到达，但低延迟）
    // 参数3：回调函数（使用lambda表达式）
    //   - [](px4_msgs::msg::VehicleStatus::UniquePtr msg) {}：
    //     * 收到消息时调用
    //     * msg：包含接收到的消息数据
    //     * {}：函数体为空，因为我们在后面用take()方式接收消息
    node.create_subscription<px4_msgs::msg::VehicleStatus>(
    topic_namespace_prefix + "fmu/out/vehicle_status" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleStatus>(), rclcpp::QoS(
      1).best_effort(),
    [](px4_msgs::msg::VehicleStatus::UniquePtr msg) {});

  // rclcpp::WaitSet：等待集合
  // 功能：用于高效等待多个订阅者的消息到达
  // 原理：使用操作系统的等待机制，而不是忙轮询
  rclcpp::WaitSet wait_set;
  
  // wait_set.add_subscription(vehicle_status_sub)：
  // 功能：将订阅者添加到等待集合中
  // 这样wait_set就会监视这个订阅者的消息
  wait_set.add_subscription(vehicle_status_sub);

  // bool got_message = false：
  // 标志变量，记录是否成功接收到消息
  bool got_message = false;
  
  // auto start_time = node.now()：
  // auto：自动类型推导
  // node.now()：获取当前时间
  // start_time：记录循环开始时间，用于计算超时
  auto start_time = node.now();

  // while (!got_message)：
  // 循环条件：只要还没收到消息，就继续循环
  // ! 表示逻辑非
  while (!got_message) {
    // auto now = node.now()：获取当前时间
    auto now = node.now();

    // if (now >= start_time + timeout)：
    // 检查是否已经超时
    // >= 表示 大于或等于
    // 功能：如果当前时间超过了开始时间+超时时间，说明已超时
    if (now >= start_time + timeout) {
      // break：跳出循环
      // 功能：停止等待，返回失败
      break;
    }

    // auto wait_ret = wait_set.wait(...)：
    // wait_set.wait()：等待订阅者接收到消息
    // 参数：剩余的等待时间（微秒）
    //   - (timeout - (now - start_time))：剩余超时时间
    //   - .to_chrono<std::chrono::microseconds>()：转换为std::chrono格式的微秒
    // wait_ret：等待结果对象
    auto wait_ret = wait_set.wait(
      (timeout - (now - start_time)).to_chrono<std::chrono::microseconds>());

    // if (wait_ret.kind() == rclcpp::WaitResultKind::Ready)：
    // 检查等待结果
    // kind()：获取等待的结果类型
    // WaitResultKind::Ready：表示有订阅者收到了消息
    if (wait_ret.kind() == rclcpp::WaitResultKind::Ready) {
      // px4_msgs::msg::VehicleStatus msg：
      // 创建一个车辆状态消息对象用于存储接收到的数据
      px4_msgs::msg::VehicleStatus msg;
      
      // rclcpp::MessageInfo info：
      // 消息元信息对象，存储消息的时间戳、发送者等信息
      rclcpp::MessageInfo info;

      // if (vehicle_status_sub->take(msg, info))：
      // take()：从订阅者的消息队列中取出一条消息
      // 参数1：msg - 存储消息数据的对象
      // 参数2：info - 存储消息元信息的对象
      // 返回值：bool，true表示成功取出消息，false表示没有消息
      // 功能：尝试从队列中取出消息
      if (vehicle_status_sub->take(msg, info)) {
        // got_message = true：设置标志为true，表示成功接收到消息
        got_message = true;

      } else {
        // 如果take()失败（虽然wait()说有消息准备好，但取不出来）
        // 输出调试信息说明情况
        RCLCPP_DEBUG(node.get_logger(), "no VehicleStatus message received");
      }

    } else {
      // 如果不是Ready状态，说明发生了超时
      // 输出调试信息
      RCLCPP_DEBUG(node.get_logger(), "timeout while waiting for FMU");
    }
  }

  // wait_set.remove_subscription(vehicle_status_sub)：
  // 从等待集合中移除订阅者
  // 清理资源，释放订阅者不再被等待
  wait_set.remove_subscription(vehicle_status_sub);
  
  // return got_message：
  // 返回结果
  // true：成功接收到消息（FMU已连接就绪）
  // false：超时或失败
  return got_message;
}

// 结束px4_ros2命名空间
} // namespace px4_ros2

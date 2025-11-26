/****************************************************************************
 * Copyright (c) 2025 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

// 包含ROS 2核心库的头文件
// rclcpp 是ROS 2 C++客户端库，提供节点、发布者、订阅者等基础功能
#include "rclcpp/rclcpp.hpp"

// 包含本地自定义的飞行模式头文件
// mode.hpp 中应该定义了 FlightModeTest 类
// 这个类表示一个自定义的飞行模式实现
#include <mode.hpp>
// 包含px4_ros2库中的NodeWithMode类
// NodeWithMode 是一个模板类，用于创建带有飞行模式的ROS 2节点
#include <px4_ros2/components/node_with_mode.hpp>
// using：类型别名关键字
// 为 px4_ros2::NodeWithMode<FlightModeTest> 类型创建一个更简洁的别名 MyNodeWithMode
// 这样后续可以直接用 MyNodeWithMode 代替冗长的模板类型
// NodeWithMode<FlightModeTest> 表示：
//   - NodeWithMode 是一个通用的节点模板类
//   - <FlightModeTest> 是模板参数，指定使用 FlightModeTest 作为具体的飞行模式实现
using MyNodeWithMode = px4_ros2::NodeWithMode<FlightModeTest>;

// static：静态关键字
// const std::string：常量字符串类型
// kNodeName：节点名称的常量（命名规范：k 前缀表示常量）
// 这个字符串作为ROS 2节点的唯一标识符
static const std::string kNodeName = "example_mode_vtol";

// kEnableDebugOutput：调试输出开关常量
// true：启用调试输出，系统会打印详细的调试信息
// false：禁用调试输出，系统只输出错误或重要信息
static const bool kEnableDebugOutput = true;

// main 函数：程序的入口点
// int argc：命令行参数的个数
// char * argv[]：命令行参数的数组，每个元素是一个字符串
// 返回值 int：程序退出码（0表示成功，非0表示出错）
int main(int argc, char * argv[])
{
  // rclcpp::init(argc, argv)：初始化ROS 2系统
  // 功能：
  //   1. 解析命令行参数
  //   2. 设置ROS 2环境
  //   3. 初始化内部数据结构
  // 这个函数必须在创建任何ROS 2对象之前调用
  rclcpp::init(argc, argv);
  
  // rclcpp::spin()：进入ROS 2事件循环
  // std::make_shared<MyNodeWithMode>(...)：创建共享指针指向新的节点对象
  //   - std::make_shared：创建智能指针的推荐方式
  //   - MyNodeWithMode：前面定义的类型别名
  //   - (kNodeName, kEnableDebugOutput)：传递给构造函数的参数
  //     * kNodeName："example_mode_vtol"，节点名称
  //     * kEnableDebugOutput：true，启用调试输出
  // 功能：
  //   1. 创建一个新的节点对象（包含飞行模式功能）
  //   2. 启动事件循环，处理所有的ROS 2回调、消息、定时器等
  //   3. 程序会在此处阻塞，直到节点被关闭
  rclcpp::spin(std::make_shared<MyNodeWithMode>(kNodeName, kEnableDebugOutput));
  
  // rclcpp::shutdown()：关闭ROS 2系统
  // 功能：
  //   1. 清理ROS 2资源
  //   2. 关闭发布者、订阅者、定时器等
  //   3. 释放内存
  // 当rclcpp::spin()返回时（通常是收到关闭信号），调用此函数进行清理
  rclcpp::shutdown();
  
  // return 0：返回成功退出码
  // 0 表示程序正常执行完成
  // 任何非零值都表示程序遇到错误
  return 0;
}

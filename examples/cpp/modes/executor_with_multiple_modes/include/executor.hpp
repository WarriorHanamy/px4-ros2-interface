/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
// #pragma once 是头文件保护指令，确保该头文件只被编译一次
#pragma once

// 包含多个飞行模式的定义
#include <modes.hpp>
// 包含飞行模式执行器的基类
#include <px4_ros2/components/mode_executor.hpp>

// ModeExecutorTest 类定义
// class：类关键字
// : public px4_ros2::ModeExecutorBase：公开继承
//   - 继承基类的所有公开和受保护的成员
//   - 获得飞行模式执行器的核心功能
// 功能说明：
//   这是一个飞行模式执行器的具体实现
//   它管理多个飞行模式的执行顺序和状态转换
//   用于演示如何在多种飞行模式间进行切换
class ModeExecutorTest : public px4_ros2::ModeExecutorBase
{
// public：公开成员，外部代码可以访问
public:
  // ModeExecutorTest 构造函数
  // px4_ros2::ModeBase & owned_mode：执行器拥有/管理的主飞行模式的引用
  // px4_ros2::ModeBase & second_mode：第二个飞行模式的引用
  // px4_ros2::ModeBase & third_mode：第三个飞行模式的引用
  // 功能：初始化执行器，存储所有需要管理的飞行模式
  ModeExecutorTest(
    px4_ros2::ModeBase & owned_mode,
    px4_ros2::ModeBase & second_mode, px4_ros2::ModeBase & third_mode)
  // 初始化列表：使用 : 符号开始
  // : ModeExecutorBase(...)：
  //   - 调用基类ModeExecutorBase的构造函数
  //   - px4_ros2::ModeExecutorBase::Settings{}：
  //     * Settings 是配置结构体
  //     * {} 表示使用默认值
  //   - owned_mode：将主飞行模式传递给基类
  // _node(owned_mode.node())：
  //   - 从飞行模式中获取ROS 2节点的引用
  //   - owned_mode.node()：获取该飞行模式关联的节点
  // _second_mode(second_mode), _third_mode(third_mode)：
  //   - 初始化成员变量，存储其他飞行模式的引用
  : ModeExecutorBase(px4_ros2::ModeExecutorBase::Settings{}, owned_mode),
    _node(owned_mode.node()), _second_mode(second_mode), _third_mode(third_mode)
  {
  }

  // enum class：强类型枚举
  // enum：枚举关键字，定义一组命名的常量
  // class：强类型枚举（C++11特性）
  //   - 枚举值在作用域内，必须使用 State:: 前缀访问
  //   - 避免命名冲突
  // 功能说明：
  //   定义飞行任务的各个状态阶段
  //   这个执行器会依次执行这些状态
  enum class State
  {
    // Reset：重置状态，初始化
    Reset,
    // TakingOff：起飞状态
    TakingOff,
    // MyFirstMode：执行第一个飞行模式
    MyFirstMode,
    // MySecondMode：执行第二个飞行模式
    MySecondMode,
    // MyThirdMode：执行第三个飞行模式
    MyThirdMode,
    // RTL：返回发射点（Return To Launch）
    RTL,
    // WaitUntilDisarmed：等待直到解锁
    WaitUntilDisarmed,
  };

  // onActivate() 函数
  // override：覆盖基类虚函数的关键字
  //   - 表示这个函数覆盖基类中同名的虚函数
  //   - 编译器会检查基类中是否存在该虚函数
  // 功能说明：
  //   当执行器被激活时调用此函数
  //   激活意味着执行器开始接收命令和执行任务
  void onActivate() override
  {
    // runState(State::TakingOff, px4_ros2::Result::Success)：
    //   - 启动状态机，进入TakingOff（起飞）状态
    //   - px4_ros2::Result::Success 表示前一个状态成功
    // 功能：当执行器激活时，开始执行起飞程序
    runState(State::TakingOff, px4_ros2::Result::Success);
  }

  // onDeactivate() 函数
  // override：覆盖基类虚函数
  // DeactivateReason reason：停用原因参数
  // 功能说明：
  //   当执行器被停用时调用此函数
  //   停用意味着执行器停止接收命令和执行任务
  void onDeactivate(DeactivateReason reason) override
  {
    // 函数体为空，表示停用时不需要特殊处理
    // 基类会自动处理必要的清理工作
  }

  // runState() 函数
  // 参数：
  //   - State state：当前要执行的状态
  //   - px4_ros2::Result previous_result：上一个状态的执行结果
  // 功能说明：
  //   状态机的主要执行函数
  //   根据当前状态和前一个状态的结果，决定下一步的操作
  //   通过递归调用自己来实现状态转换
  void runState(State state, px4_ros2::Result previous_result)
  {
    // if (previous_result != px4_ros2::Result::Success)：
    // 检查前一个状态是否成功
    // != 表示 不等于
    if (previous_result != px4_ros2::Result::Success) {
      // RCLCPP_ERROR()：输出错误日志
      // "State %i: previous state failed: %s"：格式化字符串
      // (int)state：将State枚举转换为整数用于输出
      // resultToString(previous_result)：将Result转换为字符串
      RCLCPP_ERROR(
        _node.get_logger(), "State %i: previous state failed: %s", (int)state,
        resultToString(previous_result));
      // return：提前返回函数，停止执行
      return;
    }

    // RCLCPP_DEBUG()：输出调试日志
    // 功能：记录正在执行的状态
    RCLCPP_DEBUG(_node.get_logger(), "Executing state %i", (int)state);

    // switch (state)：
    // switch 语句：多分支选择语句
    // 根据state的值，选择不同的执行路径
    // 功能：为每个状态编写对应的处理代码
    switch (state) {
      // case State::Reset：
      // 当state为Reset时执行
      case State::Reset:
        // break：跳出switch语句
        // 在Reset状态下不需要做任何操作
        break;

      // case State::TakingOff：
      // 当state为TakingOff时执行
      case State::TakingOff:
        // takeoff()：基类提供的起飞函数
        // lambda表达式：[this](px4_ros2::Result result)
        //   - [this]：捕获当前对象的指针
        //   - (px4_ros2::Result result)：参数，起飞的结果
        //   - {runState(State::MyFirstMode, result);}：
        //     * 起飞完成后，调用runState进入下一个状态
        //     * result 传递起飞的结果
        // 功能：执行起飞，起飞完成后进入第一个飞行模式
        takeoff([this](px4_ros2::Result result) {runState(State::MyFirstMode, result);});
        break;

      // case State::MyFirstMode：
      // 执行第一个飞行模式
      case State::MyFirstMode:
        // scheduleMode()：基类提供的函数，用于切换到指定的飞行模式
        // ownedMode().id()：
        //   - ownedMode()：获取主飞行模式对象
        //   - .id()：获取该飞行模式的唯一标识符
        // 功能：切换到第一个飞行模式执行
        scheduleMode(
          ownedMode().id(), [this](px4_ros2::Result result) {
            // 第一个模式执行完成后，进入第二个模式
            runState(State::MySecondMode, result);
          });
        break;

      // case State::MySecondMode：
      // 执行第二个飞行模式
      case State::MySecondMode:
        // _second_mode.id()：获取第二个飞行模式的ID
        scheduleMode(
          _second_mode.id(), [this](px4_ros2::Result result) {
            // 第二个模式执行完成后，进入第三个模式
            runState(State::MyThirdMode, result);
          });
        break;

      // case State::MyThirdMode：
      // 执行第三个飞行模式
      case State::MyThirdMode:
        // _third_mode.id()：获取第三个飞行模式的ID
        scheduleMode(
          _third_mode.id(), [this](px4_ros2::Result result) {
            // 第三个模式执行完成后，进入RTL（返回发射点）状态
            runState(State::RTL, result);
          });
        break;

      // case State::RTL：
      // 返回发射点（Return To Launch）状态
      case State::RTL:
        // rtl()：基类提供的RTL函数
        // 功能：执行返回发射点的操作
        rtl([this](px4_ros2::Result result) {runState(State::WaitUntilDisarmed, result);});
        break;

      // case State::WaitUntilDisarmed：
      // 等待直到解锁状态
      case State::WaitUntilDisarmed:
        // waitUntilDisarmed()：基类提供的函数，等待解锁
        // 功能：等待无人机完全解锁（停止旋转），然后输出任务完成信息
        waitUntilDisarmed(
          [this](px4_ros2::Result result) {
            // RCLCPP_INFO()：输出信息级日志
            // resultToString(result)：将Result转换为易读的字符串
            // 功能：任务全部完成，输出最终结果
            RCLCPP_INFO(_node.get_logger(), "All states complete (%s)", resultToString(result));
          });
        break;
    }
  }

// private：私有成员，只能在类内部访问
private:
  // rclcpp::Node & _node：
  // 存储ROS 2节点的引用
  // 用于访问日志记录等ROS 2功能
  rclcpp::Node & _node;
  
  // px4_ros2::ModeBase & _second_mode：
  // 存储第二个飞行模式的引用
  px4_ros2::ModeBase & _second_mode;
  
  // px4_ros2::ModeBase & _third_mode：
  // 存储第三个飞行模式的引用
  px4_ros2::ModeBase & _third_mode;
};

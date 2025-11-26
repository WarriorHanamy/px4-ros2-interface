/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

// #pragma once 是头文件保护指令，确保该头文件只被编译一次，防止重复包含导致的编译错误
#pragma once

// 包含ROS 2的核心库，提供节点、发布者、订阅者等基础功能
#include <rclcpp/rclcpp.hpp>
// 包含无人机解锁检查回复消息的定义
#include <px4_msgs/msg/arming_check_reply.hpp>
// 包含无人机解锁检查请求消息的定义
#include <px4_msgs/msg/arming_check_request.hpp>
// 包含需求标志的定义，用于设置模式的各种需求
#include <px4_ros2/common/requirement_flags.hpp>

// 包含事件处理相关的头文件
#include "events.hpp"

// 包含<memory>库，提供智能指针如shared_ptr、unique_ptr等内存管理工具
#include <memory>
// 包含<functional>库，提供std::function函数对象的支持
#include <functional>

// 前向声明：告诉编译器Registration类存在，但不需要完整定义（用于优化编译速度）
class Registration;

// 命名空间声明：所有代码都在px4_ros2命名空间中，可以通过px4_ros2::ClassName访问
// 命名空间声明：所有代码都在px4_ros2命名空间中，可以通过px4_ros2::ClassName访问
namespace px4_ros2
{

// 健康检查和解锁检查报告者类
// 这个类用于在检查过程中报告各种问题和健康状态
class HealthAndArmingCheckReporter
{
// public: 访问修饰符，表示这些成员可以被外部代码访问
public:
  // explicit 关键字：防止隐式类型转换，确保必须显式传入参数
  // 构造函数：用参数初始化HealthAndArmingCheckReporter对象
  explicit HealthAndArmingCheckReporter(px4_msgs::msg::ArmingCheckReply & arming_check_reply)
  // 初始化列表：用冒号(:)将成员变量初始化为参数值
  : _arming_check_reply(arming_check_reply) {}

  // template<typename ... Args>：模板声明
  // typename：表示模板参数是一个类型
  // ... Args：可变参数模板，允许接受任意个数的模板参数（如Args...会展开为多个参数）
  template<typename ... Args>
  // void：函数返回类型为空（不返回任何值）
  // 函数名：无人机解锁检查失败的扩展函数
  void armingCheckFailureExt(
    // uint32_t：无符号32位整数类型（范围0到4294967295）
    uint32_t event_id,
    // events::Log：事件日志级别（来自events命名空间）
    events::Log log_level, 
    // const char *：常量字符指针，指向不可修改的字符串
    const char * message, 
    // Args... args：展开的可变参数
    Args... args)
  {
    // const：常量限定符，const变量在初始化后不能被修改
    // uint16_t：无符号16位整数类型（范围0到65535）
    // {}：初始化为默认值（对于整数类型为0）
    const uint16_t navigation_mode_groups{};
    const uint8_t health_component_index{};

    // 赋值操作：将can_arm_and_run字段设置为false，表示不能进行无人机解锁
    _arming_check_reply.can_arm_and_run = false;

    // if条件语句：检查addEvent函数返回值（true为成功，false为失败）
    // !符号：逻辑非操作符，取反真假值
    if (!addEvent(
        event_id, log_level, message, navigation_mode_groups,
        health_component_index, args ...))
    {
      // printf：打印错误信息到标准输出
      printf("Error: too many events\n");
    }
  }

  // 函数声明：设置健康状态的函数
  // uint8_t：无符号8位整数类型（范围0到255）
  // bool：布尔类型（值为true或false）
  void setHealth(uint8_t health_component_index, bool is_present, bool warning, bool error);

// private: 访问修饰符，表示这些成员只能在类内部访问
private:
  // 模板成员函数：添加事件的私有函数
  // 这个函数的实现在下面有具体代码
  template<typename ... Args>
  bool addEvent(
    uint32_t event_id, const events::LogLevels & log_levels, const char * message,
    Args... args);

  // 成员变量：对无人机解锁检查回复消息的引用
  // & 符号：引用符，表示_arming_check_reply是对消息对象的引用而不是副本
  px4_msgs::msg::ArmingCheckReply & _arming_check_reply;
};


// 模板函数的具体实现
// template<typename ... Args>：定义可变参数模板
template<typename ... Args>
// bool：函数返回值类型为布尔值
bool HealthAndArmingCheckReporter::addEvent(
  uint32_t event_id, const events::LogLevels & log_levels,
  const char * message, Args... args)
{
  // 条件判断：检查事件数量是否已达到最大限制
  // >=：大于等于比较运算符
  if (_arming_check_reply.num_events >= _arming_check_reply.events.size()) {
    // 返回false表示添加失败
    return false;
  }

  // &：引用符号，e是对events数组中元素的引用，修改e会直接修改原数组
  events::EventType & e = _arming_check_reply.events[_arming_check_reply.num_events];
  // 赋值日志级别，使用位移操作符<<和按位或操作符|来组合两个级别
  e.log_levels =
    // <<：左移位操作符，将log_levels.internal左移4位
    // |：按位或操作符，将两个值合并
    (static_cast<uint8_t>(log_levels.internal) << 4) | static_cast<uint8_t>(log_levels.external);
  e.id = event_id;
  // static_assert：编译时断言，在编译期进行条件检查
  static_assert(
    events::util::sizeofArguments(args ...) <= sizeof(e.arguments),
    "Too many arguments");
  // 使用工具函数填充事件参数
  events::util::fillEventArguments(static_cast<uint8_t *>(e.arguments.data()), args ...);
  // ++：前置增量操作符，将num_events加1
  ++_arming_check_reply.num_events;
  // 返回true表示添加成功
  return true;
}


// inline：内联关键字，建议编译器将函数代码直接嵌入调用处而不是生成函数调用
// 这可以提高性能，特别是对于小函数
inline void HealthAndArmingCheckReporter::setHealth(
  uint8_t health_component_index, bool is_present, bool warning,
  bool error)
{
  // 设置健康组件的索引
  _arming_check_reply.health_component_index = health_component_index;
  // 设置该健康组件是否存在
  _arming_check_reply.health_component_is_present = is_present;
  // 设置该健康组件是否有警告
  _arming_check_reply.health_component_warning = warning;
  // 设置该健康组件是否有错误
  _arming_check_reply.health_component_error = error;
}


// 健康检查和解锁检查的主要类
class HealthAndArmingChecks
{
// public: 公开成员，可被外部代码访问
public:
  // using：类型别名关键字
  // std::function<void (HealthAndArmingCheckReporter &)>：定义CheckCallback为一个函数指针类型
  // 该函数接受一个HealthAndArmingCheckReporter引用，返回void
  using CheckCallback = std::function<void (HealthAndArmingCheckReporter &)>;

  // 构造函数：当创建HealthAndArmingChecks对象时调用
  // rclcpp::Node &：对ROS 2节点的引用（允许修改）
  // CheckCallback check_callback：检查回调函数
  // const std::string &：常量字符串引用（不能修改，不复制字符串）
  // = ""：默认参数值，如果未提供则使用空字符串
  HealthAndArmingChecks(
    rclcpp::Node & node, CheckCallback check_callback,
    const std::string & topic_namespace_prefix = "");
  
  // 删除拷贝构造函数：= delete表示禁止复制对象
  // 这防止了两个对象同时管理相同的资源
  HealthAndArmingChecks(const HealthAndArmingChecks &) = delete;

  // 公开成员函数：执行注册
  // 函数文档注释：/**...*/格式的注释说明函数用途
  /**
   * 进行注册。在启动时调用一次。这是一个阻塞方法。
   * @param name 注册名称。应该能唯一标识该组件，长度<25字符
   * @return 成功返回true，失败返回false
   */
  bool doRegister(const std::string & name);

  // 设置模式需求的函数
  // void：函数不返回任何值
  void setModeRequirements(const RequirementFlags & mode_requirements)
  {
    // 成员变量赋值
    _mode_requirements = mode_requirements;
  }

  // 获取模式需求的函数
  // & 返回值类型前的&：函数返回引用而不是副本，允许修改返回值
  RequirementFlags & modeRequirements() {return _mode_requirements;}

  // 禁用看门狗定时器的函数
  void disableWatchdogTimer()
  {
    // .reset()：智能指针的方法，释放其管理的资源
    _watchdog_timer.reset();
  }

// private: 私有成员，只能在类内部访问
private:
  // friend class：友元类声明
  // 这些类可以访问HealthAndArmingChecks的私有成员
  friend class ModeBase;
  friend class ModeExecutorBase;
  
  // 私有函数：覆盖注册（只有友元类可以调用）
  void overrideRegistration(const std::shared_ptr<Registration> & registration);

  // 私有函数：看门狗定时器的更新函数
  void watchdogTimerUpdate();

  // 私有成员变量
  // rclcpp::Node &：对节点的引用
  rclcpp::Node & _node;
  
  // std::shared_ptr<Registration>：智能指针，自动管理Registration对象的生命周期
  // shared_ptr允许多个指针指向同一个对象，当最后一个指针销毁时对象才被释放
  std::shared_ptr<Registration> _registration;
  
  // 检查回调函数的成员变量
  CheckCallback _check_callback;
  
  // bool：布尔成员变量，{true}表示初始化为true
  // _check_triggered：标记检查是否被触发
  bool _check_triggered{true};
  
  // _first_request：标记这是否是第一次请求
  bool _first_request{true};

  // rclcpp::Subscription<px4_msgs::msg::ArmingCheckRequest>::SharedPtr：
  // 订阅消息的智能指针
  // template<类型>：指定订阅的消息类型
  // ::SharedPtr：使用共享指针管理订阅对象
  rclcpp::Subscription<px4_msgs::msg::ArmingCheckRequest>::SharedPtr _arming_check_request_sub;
  
  // 发布消息的智能指针
  rclcpp::Publisher<px4_msgs::msg::ArmingCheckReply>::SharedPtr _arming_check_reply_pub;

  // 模式需求标志对象，{} 表示使用默认初始化
  RequirementFlags _mode_requirements{};
  
  // 看门狗定时器的智能指针
  rclcpp::TimerBase::SharedPtr _watchdog_timer;
  
  // 控制超时时是否关闭的标志
  bool _shutdown_on_timeout{true};
};

// 退出px4_ros2命名空间
} // namespace px4_ros2

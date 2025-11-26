/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

// #pragma once 是头文件保护指令，确保该头文件只被编译一次，防止重复包含导致的编译错误
#pragma once

// 包含无人机解锁检查回复消息的定义，其中包含Event消息类型
#include <px4_msgs/msg/arming_check_reply.hpp>

// 命名空间声明：px4_ros2::events
// :: 是作用域解析操作符，表示嵌套的命名空间
// 这里定义了px4_ros2命名空间下的events子命名空间
// 在使用这里的代码时，需要通过px4_ros2::events::ClassName来访问
namespace px4_ros2::events
{

// using 类型别名关键字
// 为px4_msgs::msg::Event类型创建一个别名EventType
// 这样可以在这个命名空间中更简洁地使用：EventType 代替 px4_msgs::msg::Event
using EventType = px4_msgs::msg::Event;

// enum class：强类型枚举声明
// enum：枚举关键字，定义一组命名的整数常量
// class：强类型枚举（C++11引入），与普通enum的区别：
//   - 强类型枚举的值在外部作用域中不可见，必须加枚举名前缀
//   - : uint8_t 指定枚举的底层类型为uint8_t（8位无符号整数）
// 功能：定义日志级别的枚举，用于区分不同严重程度的日志消息
enum class LogLevel : uint8_t
{
  // 紧急情况 - 最严重的错误级别
  Emergency = 0,
  // 警报 - 需要立即处理的情况
  Alert = 1,
  // 关键错误 - 系统功能受损
  Critical = 2,
  // 错误 - 功能无法工作
  Error = 3,
  // 警告 - 潜在问题但系统仍可运行
  Warning = 4,
  // 通知 - 正常但重要的信息
  Notice = 5,
  // 信息 - 一般信息消息
  Info = 6,
  // 调试 - 用于调试的详细信息
  Debug = 7,
  // 协议 - 协议相关的消息
  Protocol = 8,
  // 禁用 - 日志功能被禁用
  Disabled = 9,

  // Count：表示枚举值的总数，用于循环遍历所有枚举值
  Count
};

// 内部日志级别的枚举
// 与LogLevel类似，但用于区分内部日志和外部日志
// 功能：允许不同的日志级别应用于系统内部和外部用户
enum class LogLevelInternal : uint8_t
{
  Emergency = 0,
  Alert = 1,
  Critical = 2,
  Error = 3,
  Warning = 4,
  Notice = 5,
  Info = 6,
  Debug = 7,
  Protocol = 8,
  Disabled = 9,

  Count
};

// using 类型别名：创建更简洁的名字
// Log 是 LogLevel 的别名，便于代码书写
using Log = LogLevel;
// LogInternal 是 LogLevelInternal 的别名
using LogInternal = LogLevelInternal;

// struct：结构体声明（类似class但默认成员是公开的）
// 功能：表示日志级别的对象，包含外部和内部两个日志级别
struct LogLevels
{
  // = default：使用默认构造函数实现
  // 编译器会自动生成默认构造函数，将成员变量初始化为其默认值
  LogLevels() = default;
  
  // 自定义构造函数：只传入外部日志级别
  // explicit：防止隐式类型转换，确保必须显式调用该构造函数
  // // NOLINT：告诉代码风格检查工具忽略此行的检查警告
  LogLevels(Log external_level) // NOLINT
  // 初始化列表：使用 : 将成员变量初始化为参数值
  // external(external_level)：将external成员初始化为external_level参数
  // internal(...)：将internal成员初始化为external_level转换后的值
  : external(external_level), internal(static_cast<LogInternal>(external_level)) {}
  
  // 自定义构造函数：同时传入外部和内部日志级别
  LogLevels(Log external_level, LogInternal internal_level)
  : external(external_level), internal(internal_level) {}

  // 成员变量：外部日志级别，默认值为Info
  // {} 是C++11引入的统一初始化语法，用于初始化成员变量
  Log external{Log::Info};
  
  // 成员变量：内部日志级别，默认值为Info
  LogInternal internal{LogInternal::Info};
};

// namespace util：工具函数命名空间
// 这个子命名空间包含了一些辅助函数和常量，用于事件处理
namespace util
{
// source: https://gist.github.com/ruby0x1/81308642d0325fd386237cfa3b44785c

// source: 源代码出处，表示以下代码来自指定的链接
// constexpr：编译时常量关键字
// 这些常量值会在编译期被计算和硬编码到程序中
// uint32_t：无符号32位整数类型

// FNV-1a哈希算法的初始化常量
// 0x811c9dc5 是FNV-1a算法规定的初始值
constexpr uint32_t kVal32Const = 0x811c9dc5;

// FNV-1a哈希算法的质数常量
// 0x1000193 是FNV-1a算法规定的质数
constexpr uint32_t kPrime32Const = 0x1000193;

// 使用FNV-1a算法计算字符串哈希值的编译期常函数
// inline constexpr：同时指定函数为内联和编译期常量函数
// const char * const str：常量指针指向常量字符（字符串不可修改）
// const uint32_t value = kVal32Const：默认参数值
// noexcept：异常规范，表示该函数不会抛出异常
// 功能：递归计算字符串的哈希值，产生一个唯一的32位整数来标识该字符串
inline constexpr uint32_t hash32Fnv1aConst(
  const char * const str,
  const uint32_t value = kVal32Const) noexcept
{
  // 三元操作符：条件 ? 真值 : 假值
  // str[0] == '\0'：检查是否到达字符串末尾（'\0'是字符串终止符）
  return (str[0] == '\0') ? 
    // 如果是字符串末尾，返回计算得到的哈希值
    value : 
    // 否则继续递归处理下一个字符
    // (value ^ static_cast<uint32_t>(str[0])) * kPrime32Const：
    //   - value ^ static_cast<uint32_t>(str[0])：对当前字符进行异或操作
    //   - ^ 是按位异或操作符，用于混合哈希值
    //   - static_cast<uint32_t>：强制类型转换，将char转换为uint32_t
    //   - * kPrime32Const：乘以质数以增加哈希值的随机性
    hash32Fnv1aConst(
      &str[1], (value ^ static_cast<uint32_t>(str[0])) * kPrime32Const);
}

// 模板函数：填充事件参数的递推基础版本（参数列表为空）
// template<typename T>：定义一个泛型模板
// typename T：T可以是任何类型的模板参数
// inline constexpr：内联编译期常量函数
// uint8_t * buf：指向字节缓冲区的指针，用于存储参数数据
// T arg：单个模板参数
// 功能：将模板参数以二进制形式复制到缓冲区
// 这用于将事件参数序列化到消息中
template<typename T>
inline constexpr void fillEventArguments(uint8_t * buf, T arg)
{
  // This assumes we're on little-endian
  // memcpy：内存复制函数，将arg的内存内容复制到buf指向的位置
  // &arg：获取arg的地址（指针）
  // sizeof(T)：计算类型T占用的字节数
  // 功能：将一个参数序列化到缓冲区
  memcpy(buf, &arg, sizeof(T));
}

// 模板函数：填充事件参数的递归版本（参数列表不为空）
// template<typename T, typename ... Args>：定义多参数泛型模板
// typename ... Args：可变参数模板，表示可以有任意个额外参数
// 功能：递归地将多个参数序列化到缓冲区中
template<typename T, typename ... Args>
inline constexpr void fillEventArguments(uint8_t * buf, T arg, Args... args)
{
  // 首先填充第一个参数
  fillEventArguments(buf, arg);
  // 然后递归填充剩余的参数
  // buf + sizeof(T)：将缓冲区指针向前移动T的大小，以写入下一个参数
  // args ...：展开可变参数，继续处理
  fillEventArguments(buf + sizeof(T), args ...);
}

// 编译期常函数：计算单个参数的大小
// 功能：当没有参数时，返回0（用于递归终止条件）
constexpr unsigned sizeofArguments() {return 0;}

// 模板函数：计算所有参数的总大小
// template<typename T, typename ... Args>：定义参数列表类型
// const T & t：第一个参数的常量引用
// const Args &... args：剩余参数的常量引用包
// 功能：递归计算所有参数占用的总字节数
// 返回值：所有参数的总大小（单位：字节）
template<typename T, typename ... Args>
constexpr unsigned sizeofArguments(const T & t, const Args &... args)
{
  // sizeof(T)：当前参数的大小
  // + sizeofArguments(args ...)：加上剩余参数的大小（递归调用）
  return sizeof(T) + sizeofArguments(args ...);
}

// 结束util命名空间
} // namespace util

// 从事件名称生成事件ID的模板函数
// size_t N：模板参数，表示字符数组的大小（编译期常量）
// const char (& name)[N]：常量字符数组的引用
// & 表示这是一个数组引用而不是数组指针
// // NOLINT：忽略代码风格检查工具的命名规范警告
// 功能：从事件名称字符串生成一个唯一的32位ID
// 用于在系统中唯一标识每个事件类型
/**
 * 从事件名称生成事件ID
 * 该模板函数通过FNV-1a哈希算法将事件名称转换为唯一的32位整数ID
 * ID包括两部分：
 * - 低24位：事件名称的哈希值
 * - 高8位：组件ID（autopilot组件标识）
 */
template<size_t N>
constexpr uint32_t ID(const char (& name)[N]) // NOLINT(readability-identifier-naming)
{
  // Note: the generated ID must match with the python generator under Tools/px4events
  // 注意：生成的ID必须与Tools/px4events目录下的python生成器匹配
  
  // component_id：组件ID常量
  // 1U << 24：无符号1左移24位，结果为0x01000000
  // << 是左移位操作符，将位向左移动指定位数
  // 1U 表示无符号1，<< 24 表示左移24位
  // 功能：autopilot组件的标识符
  const uint32_t component_id = 1U << 24;       // autopilot component
  
  // 返回事件ID
  // (0xffffff & util::hash32Fnv1aConst(name))：
  //   - util::hash32Fnv1aConst(name)：使用FNV-1a算法计算名称的哈希值
  //   - 0xffffff & ...：按位与操作，保留低24位（清除高8位）
  // | component_id：按位或操作，将组件ID加到高8位
  // 功能：组合哈希值和组件ID生成最终的事件ID
  return (0xffffff & util::hash32Fnv1aConst(name)) | component_id;
}


// 结束px4_ros2::events命名空间
} // namespace px4_ros2::events

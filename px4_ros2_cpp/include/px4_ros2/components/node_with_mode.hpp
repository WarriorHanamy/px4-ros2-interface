/****************************************************************************
 * Copyright (c) 2024 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

// #pragma once 是头文件保护指令，确保该头文件只被编译一次，防止重复包含导致的编译错误
#pragma once

// 包含<tuple>库，提供std::tuple类型，可以存储多个不同类型的值
#include <tuple>
// 包含<utility>库，提供std::index_sequence等编译期工具函数
#include <utility>
// 包含ROS 2 C++客户端库的核心功能（节点、发布者、订阅者等）
#include <rclcpp/rclcpp.hpp>
// 包含飞行模式基类的定义
#include <px4_ros2/components/mode.hpp>
// 包含等待FMU连接的工具函数
#include <px4_ros2/components/wait_for_fmu.hpp>

// 进入px4_ros2命名空间
namespace px4_ros2
{
// \ingroup components：doxygen文档注释，表示这个代码属于components模块
// @{ 和 @} 是doxygen的分组标记，将多个类或函数组织在一起
/** \ingroup components
 *  @{
 */

/**
 * @brief A ROS2 node which instantiates a control mode and handles its registration with PX4.
 * @tparam ModeT The mode type, which must be derived from px4_ros2::ModeBase.
 *
 * Example usage:
 *
 * @code{.cpp}
 * class MyMode : public px4_ros2::ModeBase {...};
 * rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<MyMode>>("my_node"));
 * @endcode
 *
 * @ingroup components
 */

// template<typename ModeT>：定义一个模板类
//   - template：模板关键字
//   - typename ModeT：模板参数，代表任意飞行模式类型（用户提供）
//   - ModeT会被替换为实际的类型（如FlightModeTest等）
// class NodeWithMode : public rclcpp::Node：类定义
//   - 继承rclcpp::Node：获得ROS 2节点的所有功能
// 功能：这个类为每个飞行模式创建一个完整的ROS 2节点，并自动注册到PX4
template<typename ModeT>
class NodeWithMode : public rclcpp::Node
{
  // static_assert：编译期断言（编译时检查条件）
  //   - 在编译时进行条件检查，如果条件为false，编译失败
  //   - std::is_base_of<ModeBase, ModeT>::value：
  //     * is_base_of：检查第一个参数是否是第二个参数的基类
  //     * ::value：获取布尔结果
  // 功能：确保ModeT必须派生自ModeBase，否则无法编译
  //   这是编译时的类型安全检查，防止用户传入错误的类型
  static_assert(
    std::is_base_of<ModeBase, ModeT>::value,
    "Template type ModeT must be derived from px4_ros2::ModeBase");

// public：公开成员，外部代码可以访问
public:
  // explicit：禁止隐式类型转换
  //   - 防止编译器自动将参数转换为其他类型
  //   - 例如：禁止字符串自动转换为bool
  // NodeWithMode(std::string node_name, bool enable_debug_output = false)：构造函数
  //   - std::string node_name：节点名称（ROS 2中的唯一标识）
  //   - bool enable_debug_output = false：是否启用调试输出（默认值false）
  // 功能：创建一个新的节点对象
  explicit NodeWithMode(std::string node_name, bool enable_debug_output = false)
  // : Node(node_name)：初始化列表
  //   - 调用基类rclcpp::Node的构造函数
  //   - 将node_name传递给父类，完成ROS 2节点的初始化
  : Node(node_name)
  {
    // if (enable_debug_output)：条件判断
    //   - 如果用户要求启用调试输出
    if (enable_debug_output) {
      // auto：自动类型推导
      //   - 编译器自动推导ret的类型
      // rcutils_logging_set_logger_level()：ROS工具函数，设置日志级别
      //   - get_logger().get_name()：获取这个节点logger的名称
      //   - RCUTILS_LOG_SEVERITY_DEBUG：设置日志级别为DEBUG（最详细的日志）
      // 功能：开启详细的调试日志输出，便于开发和调试
      auto ret =
        rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

      // if (ret != RCUTILS_RET_OK)：检查设置是否成功
      //   - RCUTILS_RET_OK：表示操作成功的返回值
      //   - !=：不等于，如果不等于OK说明出错
      if (ret != RCUTILS_RET_OK) {
        // RCLCPP_ERROR()：ROS 2日志函数，输出错误信息
        // rcutils_get_error_string().str：获取错误信息的字符串
        // 功能：如果设置日志级别失败，输出错误信息
        RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
        // rcutils_reset_error()：重置错误状态
        rcutils_reset_error();
      }
    }

    // std::make_unique<ModeT>(*this)：创建unique_ptr智能指针
    //   - std::make_unique：创建unique_ptr的推荐方式
    //   - <ModeT>：指定指针指向的类型
    //   - (*this)：将当前NodeWithMode对象（this指针解引用）传递给ModeT的构造函数
    //   - unique_ptr：唯一指针，确保只有一个所有者管理该对象
    // _mode：私有成员变量，存储飞行模式对象的指针
    // 功能：
    //   1. 动态创建ModeT类型的飞行模式对象
    //   2. 将当前节点对象传递给模式，让它访问ROS 2功能
    //   3. 用智能指针存储，自动释放内存（析构时）
    _mode = std::make_unique<ModeT>(*this);

    // if (!_mode->doRegister())：检查注册是否成功
    //   - _mode->：访问指针指向的对象的成员
    //   - doRegister()：调用飞行模式的注册函数
    //   - !：逻辑非，如果返回false（注册失败），进入if块
    // 功能：尝试向PX4飞行控制器注册该飞行模式
    if (!_mode->doRegister()) {
      // throw Exception()：抛出异常
      //   - 表示发生了严重错误，程序无法继续
      //   - Exception是自定义的异常类
      // 功能：如果注册失败，立即停止程序执行，通知调用者发生了错误
      throw Exception("Registration failed");
    }
  }

  // ModeT &：返回类型
  //   - &：引用符号，返回对象的引用而不是副本
  //   - 这样调用者可以修改飞行模式对象
  // getMode()：公开成员函数，获取飞行模式对象
  // const：表示这个函数不修改对象的任何成员变量
  // 功能：允许外部代码访问和操作内部的飞行模式对象
  ModeT & getMode() const
  {
    // *_mode：解引用unique_ptr，获得实际的ModeT对象
    // return：返回该对象的引用
    return *_mode;
  }

// private：私有成员，只能在类内部访问
private:
  // std::unique_ptr<ModeT>：唯一指针
  //   - unique_ptr：智能指针，当对象被销毁时自动释放内存
  //   - <ModeT>：指针指向ModeT类型的对象
  // _mode：成员变量名称（以下划线开头表示私有成员）
  // 功能：存储飞行模式对象的指针，自动管理其生命周期
  std::unique_ptr<ModeT> _mode;
};

/**
 * @brief A ROS2 node which instantiates a mode executor and its owned mode, and handles their registration with PX4.
 *
 * Assumes mode executor constructor signature is `ModeExecutorT(rclcpp::Node, OwnedModeT, OtherModesT...)`.
 *
 * @tparam ModeExecutorT The mode executor type, which must be derived from px4_ros2::ModeExecutorBase.
 * @tparam OwnedModeT The mode type owned by the executor, which must be derived from px4_ros2::ModeBase.
 * @tparam OtherModesT Optional additional modes not owned by the executor but registered by the node, which must be derived from px4_ros2::ModeBase.
 *
 * Example usage:
 *
 * @code{.cpp}
 * class MyMode : public px4_ros2::ModeBase {...};
 * class MyExecutor : public px4_ros2::ModeExecutor {...};
 * rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<MyExecutor, MyMode>>("my_node"));
 * @endcode
 *
 * @ingroup components
 */

// template<typename ModeExecutorT, typename OwnedModeT, typename ... OtherModesT>：
//   - ModeExecutorT：飞行模式执行器的类型
//   - OwnedModeT：执行器拥有/管理的主飞行模式类型
//   - typename ... OtherModesT：可变模板参数
//     * ... 表示零个或多个额外的飞行模式类型
//     * 这是C++11引入的可变参数模板，允许接受任意数量的模板参数
// class NodeWithModeExecutor : public rclcpp::Node：
//   - 继承自rclcpp::Node，获得ROS 2节点的功能
// 功能：这个类为更复杂的场景提供支持
//   - 支持一个"主"飞行模式（由执行器管理）
//   - 支持多个"其他"飞行模式（由节点注册但不由执行器管理）
//   - 执行器可以在这些模式间切换
template<typename ModeExecutorT, typename OwnedModeT, typename ... OtherModesT>
class NodeWithModeExecutor : public rclcpp::Node
{
  // static_assert：编译期断言
  
  // std::is_base_of_v<ModeExecutorBase, ModeExecutorT>：
  //   - is_base_of_v：_v后缀表示返回bool值（不需要::value）
  //   - 检查ModeExecutorT是否派生自ModeExecutorBase
  // 功能：确保ModeExecutorT是有效的执行器类型
  static_assert(
    std::is_base_of_v<ModeExecutorBase, ModeExecutorT>,
    "Template type ModeExecutorT must be derived from px4_ros2::ModeExecutorBase");
  
  // (std::is_base_of_v<ModeBase, OwnedModeT>&& ... && std::is_base_of_v<ModeBase, OtherModesT>)：
  //   - && ... &&：fold expression（折叠表达式）
  //     * 这是C++17引入的特性
  //     * ... 在操作符 && 左侧表示对可变参数进行递归应用
  //     * 相当于：(条件1 && 条件2 && ... && 条件n)
  //   - 第一部分检查OwnedModeT是否派生自ModeBase
  //   - 第二部分检查所有OtherModesT是否派生自ModeBase
  // 功能：确保所有飞行模式类型都是有效的（派生自ModeBase）
  static_assert(
    (std::is_base_of_v<ModeBase, OwnedModeT>&& ... && std::is_base_of_v<ModeBase, OtherModesT>),
    "Template types OwnedModeT and OtherModesT must be derived from px4_ros2::ModeBase");

public:
  // explicit：禁止隐式类型转换
  // NodeWithModeExecutor(std::string node_name, bool enable_debug_output = false)：构造函数
  //   - 功能：创建支持多模式的ROS 2节点
  explicit NodeWithModeExecutor(std::string node_name, bool enable_debug_output = false)
  // : Node(node_name)：初始化列表，调用基类构造函数
  : Node(node_name)
  {
    // 启用调试输出的逻辑（同前面的NodeWithMode类）
    if (enable_debug_output) {
      auto ret =
        rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

      if (ret != RCUTILS_RET_OK) {
        RCLCPP_ERROR(get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
        rcutils_reset_error();
      }
    }

    // _owned_mode = std::make_unique<OwnedModeT>(*this)：
    //   - 创建执行器将管理的主飞行模式对象
    //   - OwnedModeT：主飞行模式类型（由执行器拥有/控制）
    // 功能：实例化主飞行模式
    _owned_mode = std::make_unique<OwnedModeT>(*this);
    
    // _other_modes = std::make_tuple(std::make_unique<OtherModesT>(*this)...)：
    //   - std::tuple：容器类型，可以存储多个不同类型的值
    //   - std::make_unique<OtherModesT>(*this)...：
    //     * OtherModesT...：展开可变参数（pack expansion）
    //     * 为每个OtherModesT类型创建一个unique_ptr
    //   - std::make_tuple()：将所有这些指针打包成一个tuple
    // 功能：
    //   1. 为每个其他飞行模式类型创建一个对象
    //   2. 将所有指针存储在一个tuple中
    //   3. tuple可以存储不同类型的元素
    _other_modes = std::make_tuple(std::make_unique<OtherModesT>(*this)...);
    
    // _mode_executor = createModeExecutor(std::index_sequence_for<OtherModesT...>{})：
    //   - std::index_sequence_for<OtherModesT...>：
    //     * 生成一个编译期整数序列：[0, 1, 2, ..., sizeof...(OtherModesT)-1]
    //     * 用于索引tuple中的各个元素
    //   - createModeExecutor()：辅助函数（下面定义）
    //     * 根据其他飞行模式的个数，创建适当的执行器
    // 功能：创建飞行模式执行器，并传递所有飞行模式给它
    _mode_executor = createModeExecutor(std::index_sequence_for<OtherModesT...>{});

    // if (!_mode_executor->doRegister() || !std::apply(...))：
    //   - _mode_executor->doRegister()：注册执行器到PX4
    //   - std::apply()：对tuple中的每个元素应用一个函数
    //   - ||：逻辑或，任何一个失败都返回false
    // 功能：确保执行器和所有飞行模式都成功注册到PX4
    if (!_mode_executor->doRegister() || !std::apply(
        // lambda表达式：[](const auto &... mode)
        //   - []：捕获列表（这里为空，不捕获外部变量）
        //   - (const auto &... mode)：参数
        //     * auto：自动类型推导
        //     * ... mode：可变参数，代表tuple中的每个元素
        //     * const &：常量引用，不能修改
        // 功能：创建一个匿名函数，接收tuple中的所有飞行模式
        [](const auto &... mode) {
          // *INDENT-OFF* 和 *INDENT-ON*：指令告诉代码格式化工具忽略缩进检查
          // (mode->doRegister() && ...)：
          //   - mode->doRegister()...：对每个飞行模式调用doRegister()
          //   - && ...：fold expression（折叠表达式）
          //     * 展开为：(mode1->doRegister() && mode2->doRegister() && ...)
          //     * 只要有一个返回false，整个表达式就为false
          // 功能：对所有飞行模式进行注册
          return (mode->doRegister() && ...);
          // *INDENT-ON*
        }, _other_modes))
    {
      // 如果任何注册失败，抛出异常
      throw Exception("Registration failed");
    }
  }

  // template<typename ModeT = OwnedModeT>：
  //   - 这是一个模板成员函数
  //   - ModeT = OwnedModeT：默认参数
  //     * 如果不指定ModeT，默认使用OwnedModeT（主飞行模式）
  // ModeT &：返回指定类型的飞行模式对象的引用
  // const：不修改对象的成员变量
  // 功能：获取指定类型的飞行模式对象
  //   - 可以是主飞行模式（OwnedModeT）
  //   - 也可以是其他飞行模式之一（OtherModesT）
  template<typename ModeT = OwnedModeT>
  ModeT & getMode() const
  {
    // if constexpr (std::is_same_v<ModeT, OwnedModeT>)：
    //   - if constexpr：编译期条件判断
    //     * 在编译时进行条件检查
    //     * 只有真条件的代码会被编译
    //     * 如果条件为true，else块的代码会被删除
    //   - std::is_same_v<ModeT, OwnedModeT>：
    //     * is_same_v：检查两个类型是否相同
    //     * 返回true或false
    // 功能：判断请求的是哪个飞行模式
    if constexpr (std::is_same_v<ModeT, OwnedModeT>) {
      // 如果请求的是主飞行模式，返回_owned_mode
      return *_owned_mode;
    } else {
      // 否则从_other_modes tuple中获取指定类型的飞行模式
      // std::get<ModeT>(_other_modes)：
      //   - 从tuple中获取类型为ModeT的元素
      //   - 这个操作是类型安全的，编译时检查类型是否存在
      return *std::get<ModeT>(_other_modes);
    }
  }

// private：私有成员，只能在类内部访问
private:
  // template<std::size_t... Idx>：
  //   - 这是一个模板成员函数
  //   - std::size_t：无符号整数类型
  //   - ... Idx：可变参数，表示多个整数
  //   - 这些参数由std::index_sequence提供
  // auto：自动返回类型推导
  // createModeExecutor(std::index_sequence<Idx...>)：
  //   - std::index_sequence<Idx...>：编译期整数序列
  //     * 例如：如果有2个OtherModesT，序列为[0, 1]
  //   - 这个参数类型用于索引tuple中的元素
  // 功能：
  //   1. 这是一个工厂函数（工具函数）
  //   2. 根据其他飞行模式的个数，创建不同的执行器
  //   3. 自动处理参数传递的细节
  template<std::size_t... Idx>
  auto createModeExecutor(std::index_sequence<Idx...>)
  {
    // if constexpr (sizeof...(Idx) == 0)：
    //   - if constexpr：编译期条件判断
    //   - sizeof...(Idx)：获取可变参数的个数
    //     * 如果没有其他飞行模式，Idx为空，个数为0
    if constexpr (sizeof...(Idx) == 0) {
      // 如果没有其他飞行模式，只传递主飞行模式给执行器
      // 调用：ModeExecutorT(*_owned_mode)
      return std::make_unique<ModeExecutorT>(*_owned_mode);
    } else {
      // 如果有其他飞行模式
      // std::get<Idx>(_other_modes)...：
      //   - std::get<Idx>(_other_modes)：从tuple中获取第Idx个元素
      //   - ... 进行pack expansion（包展开）
      //   - 展开为：*std::get<0>(...), *std::get<1>(...), ...
      // 功能：从tuple中逐个提取其他飞行模式，传递给执行器构造函数
      // 调用：ModeExecutorT(*_owned_mode, *other_mode_1, *other_mode_2, ...)
      return std::make_unique<ModeExecutorT>(*_owned_mode, *std::get<Idx>(_other_modes)...);
    }
  }

  // std::unique_ptr<ModeExecutorT>：唯一指针存储飞行模式执行器
  //   - unique_ptr：智能指针，对象销毁时自动释放内存
  //   - ModeExecutorT：执行器的类型
  // 功能：管理飞行模式执行器的生命周期
  std::unique_ptr<ModeExecutorT> _mode_executor;
  
  // std::unique_ptr<OwnedModeT>：唯一指针存储被拥有的主飞行模式
  //   - unique_ptr：智能指针
  //   - OwnedModeT：主飞行模式的类型
  // 功能：管理主飞行模式的生命周期
  std::unique_ptr<OwnedModeT> _owned_mode;
  
  // std::tuple<std::unique_ptr<OtherModesT>...>：
  //   - std::tuple：容器，可以存储多个不同类型的值
  //   - std::unique_ptr<OtherModesT>...：
  //     * OtherModesT...：展开可变参数
  //     * 为每个OtherModesT类型创建一个unique_ptr
  //     * 例如：std::tuple<unique_ptr<Mode1>, unique_ptr<Mode2>, ...>
  // 功能：存储所有其他飞行模式的唯一指针，自动管理它们的生命周期
  std::tuple<std::unique_ptr<OtherModesT>...> _other_modes;
};

// @}：结束doxygen文档分组标记
/** @}*/
// 结束px4_ros2命名空间
}  // namespace px4_ros2

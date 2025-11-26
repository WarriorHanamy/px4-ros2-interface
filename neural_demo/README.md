# Neural Control Demo

这是一个基于px4-ros2-interface的端到端无人机控制系统，集成了RC触发、完善的failsafe机制和简单的轨迹控制演示。

## 功能特性

### 核心功能
- **RC触发集成**: 无需单独的触发节点，RC触发机制直接集成在Executor中
- **完善的Failsafe**: 多层次安全检查，包括位置有效性、RC状态、数据完整性
- **假网络节点**: 模拟网络导航系统，支持配置文件驱动的航点飞行
- **Trajectory控制**: 使用PX4的TrajectorySetpoint实现平滑轨迹控制

### 安全机制
- **进入前检查**: 验证位置信息有效性、RC连接状态
- **运行时监控**: 实时检查位置超时、RC中断、目标数据时效性
- **自动恢复**: 故障时自动返回安全的Position模式
- **中断保护**: 支持通过移动摇杆中断End2End控制

## 使用流程

### 完整飞行流程
1. **QGC设定**: 通过QGC设定Demo Start模式
2. **等待解锁**: 系统等待无人机解锁
3. **自动起飞**: 解锁后自动起飞到指定高度
4. **进入Position模式**: 起飞完成后进入Position模式等待
5. **RC触发**: 通过QGC joystick按Button 1024触发End2End控制
6. **End2End控制**: 自动飞往配置的目标航点
7. **目标到达**: 到达目标后自动返回Position模式
8. **静止检测**: 等待无人机静止后开始降落
9. **自动降落**: 执行自动降落程序
10. **任务完成**: 等待熄火状态

### RC控制说明
- **触发Button**: Button 1024 (对应RC的aux1通道 > 0.8)
- **中断条件**: sticks_moving=true (移动任何摇杆)
- **安全检查**: 必须在Position模式下才能触发

## 配置文件

### `config/demo_targets.yaml`
```yaml
demo_targets:
  # 机体坐标系航点 [x, y, z] (米)
  # x: 前进, y: 右侧, z: 向下 (负值=向上)
  waypoints:
    - [5.0, 0.0, -2.0]   # 前进5米，高度2米
    - [5.0, 3.0, -2.0]   # 右侧3米
    - [0.0, 3.0, -2.0]   # 回到起始X位置，右侧3米
    - [0.0, 0.0, -2.0]   # 回到起始位置

failsafe_config:
  position_timeout: 1.0     # 位置超时(秒)
  rc_timeout: 0.5           # RC超时(秒)
  target_tolerance: 0.5     # 目标到达容差(米)
  still_wait_time: 5.0       # 静止最小等待时间(秒)
  max_velocity: 2.0         # 最大速度限制(m/s)
  target_timeout: 2.0       # 目标数据超时(秒)
```

## 启动方式

### 使用Justfile (推荐)
```bash
# 启动完整演示
just neural-demo

# 调试模式
just neural-demo-debug

# 仅启动假网络节点(调试用)
just fake-network

# 使用自定义配置文件
just neural-demo-config
```

### 使用ROS2命令
```bash
# 标准启动
ros2 launch neural_demo neural_demo.launch.py

# 调试模式
ros2 launch neural_demo neural_demo.launch.py debug:=true

# 自定义配置
ros2 launch neural_demo neural_demo.launch.py config_file:=/path/to/config.yaml
```

## 系统架构

### 核心组件
1. **DemoModeExecutor**: 集成RC触发的状态机执行器
2. **ModeNeuralCtrl**: 使用TrajectorySetpoint的控制模式
3. **FakeNetworkNode**: 模拟网络导航系统的假节点

### 状态机设计
```
TakingOff → Position → NeuralCtrl → Position → WaitingStill → Land → WaitUntilDisarmed
```

### 通信架构
- **RC触发**: `/fmu/out/manual_control_setpoint`
- **目标发布**: `/neural/target_pose` (geometry_msgs/PoseStamped)
- **位置验证**: `/fmu/out/vehicle_local_position`
- **轨迹控制**: TrajectorySetpoint (PX4内部)

## 技术特点

### 安全设计
- **位置验证**: 检查xy_valid、z_valid、dead_reckoning标志
- **时效性检查**: 位置和目标数据都有超时机制
- **多重保护**: 进入检查、运行时监控、故障恢复
- **优雅降级**: 任何异常都安全返回Position模式

### 代码质量
- **模块化设计**: 清晰的组件分离和接口定义
- **配置驱动**: 支持YAML配置文件和ROS2参数
- **错误处理**: 完善的异常处理和日志记录
- **框架兼容**: 完全兼容px4-ros2-interface框架

## 开发调试

### 日志级别
- **INFO**: 基本状态信息
- **WARN**: 警告和恢复操作
- **ERROR**: 严重错误和模式失败
- **DEBUG**: 详细调试信息(需要debug参数)

### 常用调试命令
```bash
# 查看节点状态
ros2 node info /neural_demo_node

# 查看话题列表
ros2 topic list | grep neural

# 监控目标发布
ros2 topic echo /neural/target_pose

# 查看参数配置
ros2 param list /neural_demo_node
```

## 故障排除

### 常见问题
1. **RC触发无效**: 检查是否在Position模式，aux1通道是否正确映射
2. **位置数据异常**: 检查GPS/定位系统状态，确认xy_valid标志
3. **目标不更新**: 检查假网络节点是否正常启动和配置
4. **模式切换失败**: 检查PX4模式权限和预飞检查

### 调试建议
1. 使用`just neural-demo-debug`启用详细日志
2. 检查PX4系统健康状态
3. 验证QGC的joystick配置
4. 确认配置文件路径和格式正确

## 依赖项

- **px4_ros2_cpp**: PX4 ROS2接口库
- **geometry_msgs**: 几何消息类型
- **px4_msgs**: PX4消息定义
- **yaml_cpp_vendor**: YAML解析库
- **Eigen3**: 线性代数库

## 许可证

BSD-3-Clause License (与PX4项目一致)
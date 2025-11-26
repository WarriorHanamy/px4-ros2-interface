#!/usr/bin/env python3

"""
Neural Demo Launch File

This launch file starts the Neural demo system with:
- Neural Demo Executor with integrated RC triggering
- Fake Network Node for trajectory generation
- Comprehensive failsafe mechanisms
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('neural_demo')

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'demo.yaml'),
        description='Configuration file for demo targets and failsafe parameters'
    )

    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug output'
    )

    # Parameters for DemoModeExecutor and ModeNeuralCtrl
    common_params = {
        'config_file': LaunchConfiguration('config_file'),
        # DemoModeExecutor parameters (can override YAML values)
        'position_timeout': 1.0,
        'rc_timeout': 0.5,
        'target_tolerance': 0.5,
        'still_wait_time': 5.0,
        'max_velocity': 8.0,
        # ModeNeuralCtrl parameters
        'neural_target_tolerance': 0.5,
        'neural_position_timeout': 1.0,
        'neural_target_timeout': 2.0,
        'neural_max_velocity': 8.0,
        'use_sim_time': False
    }

    # Neural Demo Executor Node
    executor_node = Node(
        package='neural_demo',
        executable='neural_demo_node',
        name='neural_demo_node',
        output='screen',
        parameters=[common_params],
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info' if LaunchConfiguration('debug') == 'false' else 'debug', '--log-level', 'rcl:=warn', '--log-level', 'rmw_fastrtps_cpp:=warn']
    )

    return LaunchDescription([
        config_file_arg,
        debug_arg,
        executor_node
    ])
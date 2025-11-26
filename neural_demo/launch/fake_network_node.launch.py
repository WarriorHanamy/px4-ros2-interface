#!/usr/bin/env python3

"""
Fake Network Node Launch File

This launch file starts only the Fake Network Node for trajectory generation
with the same configuration parameters as the main e2e_demo launch file.
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

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Target pose publish rate in Hz'
    )

    # Parameters for fake_network_node
    common_params = {
        'config_file': LaunchConfiguration('config_file'),
        'publish_rate': LaunchConfiguration('publish_rate'),
        'fake_target_tolerance': 0.5,
        'fake_max_velocity': 8.0,
        'use_sim_time': False
    }

    # Fake Network Node
    fake_network_node = Node(
        package='neural_demo',
        executable='fake_network_node',
        name='fake_network_node',
        output='screen',
        parameters=[common_params],
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info' if LaunchConfiguration('debug') == 'false' else 'debug', '--log-level', 'rcl:=warn', '--log-level', 'rmw_fastrtps_cpp:=warn']
    )

    return LaunchDescription([
        config_file_arg,
        debug_arg,
        publish_rate_arg,
        fake_network_node
    ])
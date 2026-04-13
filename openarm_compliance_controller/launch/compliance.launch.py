# Copyright 2026 OpenArm Contributors
# SPDX-License-Identifier: Apache-2.0
"""
Launch file to spawn the compliance controller alongside the existing
JointTrajectoryController. Must be run AFTER openarm bringup is launched.

Usage:
  ros2 launch openarm_compliance_controller compliance.launch.py
  ros2 launch openarm_compliance_controller compliance.launch.py side:=left
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    side_arg = DeclareLaunchArgument(
        'side', default_value='right',
        description='Which arm to control: "right" or "left"'
    )

    side = LaunchConfiguration('side')

    config_file = PathJoinSubstitution([
        FindPackageShare('openarm_compliance_controller'),
        'config', 'compliance_controller.yaml'
    ])

    # Load controller parameters into controller_manager
    load_params = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            [side, '_compliance_controller'],
            '--controller-manager', '/controller_manager',
            '--param-file', config_file,
        ],
        output='screen',
    )

    return LaunchDescription([
        side_arg,
        load_params,
    ])

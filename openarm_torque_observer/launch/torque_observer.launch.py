# Copyright 2026 OpenArm Contributors
# SPDX-License-Identifier: Apache-2.0
"""
Launch file for the torque observer (shadow mode) — bimanual aware.

Usage:
  # Start observer for the RIGHT arm (default) alongside bimanual bringup:
  ros2 launch openarm_torque_observer torque_observer.launch.py

  # Start observer for the LEFT arm:
  ros2 launch openarm_torque_observer torque_observer.launch.py arm_prefix:=left_

  # Start observers for BOTH arms simultaneously:
  ros2 launch openarm_torque_observer torque_observer.launch.py arm_prefix:=both
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _create_observer_node(config_file, arm_prefix, namespace=""):
    """Create a single torque observer node for one arm.
    
    Node is always named 'torque_observer' so the YAML namespace
    'torque_observer: ros__parameters:' matches. We use ROS namespace
    to disambiguate when running both arms.
    """
    return Node(
        package="openarm_torque_observer",
        executable="torque_observer",
        name="torque_observer",
        namespace=namespace or None,
        output="screen",
        parameters=[
            config_file,
            {"arm_prefix": arm_prefix},
        ],
    )


def _launch_setup(context):
    config_file = LaunchConfiguration("config_file").perform(context)
    arm_prefix = LaunchConfiguration("arm_prefix").perform(context)

    nodes = []
    if arm_prefix == "both":
        # Launch one observer for each arm, in separate namespaces
        nodes.append(_create_observer_node(config_file, "right_", "observer_right"))
        nodes.append(_create_observer_node(config_file, "left_", "observer_left"))
    else:
        nodes.append(_create_observer_node(config_file, arm_prefix))

    return nodes


def generate_launch_description():
    pkg_dir = get_package_share_directory("openarm_torque_observer")
    default_config = os.path.join(pkg_dir, "config", "friction_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="Path to friction parameters YAML file",
        ),
        DeclareLaunchArgument(
            "arm_prefix",
            default_value="right_",
            description="Arm prefix: 'right_', 'left_', 'both', or '' for single arm",
        ),
        OpaqueFunction(function=_launch_setup),
    ])

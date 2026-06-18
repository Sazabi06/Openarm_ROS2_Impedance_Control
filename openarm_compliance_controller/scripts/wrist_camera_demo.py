#!/usr/bin/env python3
"""
Wrist camera mount verification demo.
- Moves right arm J4 to 90° (elbow bend) while keeping other joints at 0
- Opens the right gripper
- Holds for viewing, then returns to zero

Usage (requires ROS 2 bringup running):
    source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
    python3 wrist_camera_demo.py
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from builtin_interfaces.msg import Duration


class WristCameraDemo(Node):
    def __init__(self):
        super().__init__('wrist_camera_demo')

        # JTC publisher for right arm
        self.jtc_pub = self.create_publisher(
            JointTrajectory,
            '/right_joint_trajectory_controller/joint_trajectory',
            10
        )

        # Gripper action client
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            '/right_gripper_controller/gripper_cmd'
        )

        self.joints = [
            'openarm_right_joint1',
            'openarm_right_joint2',
            'openarm_right_joint3',
            'openarm_right_joint4',
            'openarm_right_joint5',
            'openarm_right_joint6',
            'openarm_right_joint7',
        ]

    def send_trajectory(self, positions, duration_sec=3.0):
        """Send a joint trajectory to the right arm."""
        msg = JointTrajectory()
        msg.joint_names = self.joints

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * 7
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9)
        )
        msg.points = [point]

        self.jtc_pub.publish(msg)
        self.get_logger().info(f'Sent trajectory (duration={duration_sec}s)')

    def open_gripper(self):
        """Open the right gripper."""
        if not self.gripper_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn('Gripper server not available')
            return

        goal = GripperCommand.Goal()
        goal.command.position = 0.020  # Open = 24mm
        goal.command.max_effort = 10.0

        self.get_logger().info('Opening gripper...')
        self.gripper_client.send_goal_async(goal)

    def close_gripper(self):
        """Close the right gripper."""
        if not self.gripper_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn('Gripper server not available')
            return

        goal = GripperCommand.Goal()
        goal.command.position = 0.0  # Closed
        goal.command.max_effort = 10.0

        self.get_logger().info('Closing gripper...')
        self.gripper_client.send_goal_async(goal)


def main():
    rclpy.init()
    node = WristCameraDemo()

    # Wait for publishers to connect
    time.sleep(1.0)

    print('\n' + '=' * 50)
    print('  WRIST CAMERA MOUNT VERIFICATION')
    print('=' * 50)

    # Step 1: Move J4 to 90 degrees (all others stay at 0)
    j4_angle = math.pi / 2  # 90 degrees
    print(f'\n[1] Moving J4 to {math.degrees(j4_angle):.0f}° ...')
    #                    J1   J2   J3   J4        J5   J6   J7
    node.send_trajectory([0.0, 0.0, 0.0, j4_angle, 0.0, 0.0, 0.0], duration_sec=3.0)
    time.sleep(4.0)

    # Step 2: Open gripper
    print('[2] Opening gripper...')
    node.open_gripper()
    time.sleep(2.0)

    print('\n[✓] Check the camera viewer!')
    print('    - Wrist camera should show what the hand is looking at')
    print('    - Gripper fingers should be visible at the edges')
    input('\n    Press ENTER to return to zero position...')

    # Step 3: Close gripper and return to zero
    print('\n[3] Closing gripper and returning to zero...')
    node.close_gripper()
    time.sleep(1.0)
    node.send_trajectory([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], duration_sec=3.0)
    time.sleep(4.0)

    print('[✓] Done!\n')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

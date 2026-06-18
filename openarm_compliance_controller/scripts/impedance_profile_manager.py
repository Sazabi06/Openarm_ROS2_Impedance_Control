#!/usr/bin/env python3
"""
Task 2.4: Impedance Profile Manager

Maps phase names to impedance gain profiles and publishes them to the
compliance controller and gripper stiffness controller.

Subscribes:
    /impedance_phase (std_msgs/String) — phase name
    
Publishes:
    /{side}_compliance_controller/impedance_params (Float64MultiArray)
      → 16 values: [kp1..kp7, kd1..kd7, grip_kp, grip_kd]

Predefined profiles: transit, approach, contact, grasp, teach

Usage:
    python3 impedance_profile_manager.py
    python3 impedance_profile_manager.py --side left

    # Then switch profiles:
    ros2 topic pub --once /impedance_phase std_msgs/String '{data: "teach"}'

Author: Agent-C1
Date: 2026-04-30
"""

import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray


# ═══════════════════════════════════════════════════════════════════
# Impedance profiles — Kp (7 joints), Kd (7 joints), grip_kp (1)
#
# Design rationale:
#   transit  — stiff for precise positioning during free-space motion
#   approach — medium stiffness as end-effector nears an object
#   contact  — soft for safe interaction upon contact detection
#   grasp    — stiff arm + high grip force for secure holding
#   teach    — very soft for manual guidance (human-in-the-loop)
# ═══════════════════════════════════════════════════════════════════
PROFILES = {
    "transit": {
        "kp": [70, 70, 70, 60, 10, 10, 10],
        "kd": [2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5],
        "grip_kp": 5.0,   # must be ≥5.0 for gripper to overcome friction
        "grip_kd": 0.1,
    },
    "approach": {
        "kp": [50, 50, 50, 40, 8, 8, 8],
        "kd": [2.5, 2.0, 1.5, 1.5, 0.5, 0.5, 0.4],
        "grip_kp": 5.0,
        "grip_kd": 0.1,
    },
    "contact": {
        "kp": [30, 30, 30, 20, 5, 5, 5],
        "kd": [2.0, 1.5, 1.0, 0.8, 0.3, 0.3, 0.2],
        "grip_kp": 3.0,   # softer for contact, still moves
        "grip_kd": 0.1,
    },
    "grasp": {
        "kp": [70, 70, 70, 60, 10, 10, 10],
        "kd": [2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5],
        "grip_kp": 7.0,   # firm grip for holding objects
        "grip_kd": 0.2,
    },
    "teach": {
        "kp": [12, 12, 12, 3, 2, 2, 2],
        "kd": [0.5, 0.5, 0.4, 0.4, 0.15, 0.12, 0.1],
        "grip_kp": 0.3,   # at kp_min → triggers teach mode (freely draggable)
        "grip_kd": 0.05,
    },
}


class ImpedanceProfileManager(Node):
    """Maps phase names to impedance gain profiles."""

    def __init__(self):
        super().__init__('impedance_profile_manager')

        # Parameters
        self.declare_parameter('side', 'right')
        self.side = self.get_parameter('side').value

        # Current profile
        self.current_profile = None

        # Subscriber
        self.create_subscription(
            String, '/impedance_phase', self._phase_cb, 10)

        # Publishers — unified impedance params:
        # [kp1..kp7, kd1..kd7, grip_kp, grip_kd] = 16 values
        self.impedance_pub = self.create_publisher(
            Float64MultiArray,
            f'/{self.side}_compliance_controller/impedance_params',
            10)

        # Status publisher
        self.status_pub = self.create_publisher(
            String, '/impedance_profile_manager/status', 10)

        # Log available profiles
        profile_list = ', '.join(PROFILES.keys())
        self.get_logger().info(
            f'Impedance Profile Manager ready.\n'
            f'  Side: {self.side}\n'
            f'  Profiles: {profile_list}\n'
            f'  Listening on: /impedance_phase\n'
            f'  Publishing to:\n'
            f'    /{self.side}_compliance_controller/impedance_params (16 values)')

        # Start with transit profile
        self._apply_profile('transit')

    def _phase_cb(self, msg: String):
        """Handle phase transition request."""
        phase = msg.data.strip().lower()

        if phase not in PROFILES:
            self.get_logger().warn(
                f'Unknown profile "{phase}". '
                f'Available: {", ".join(PROFILES.keys())}')
            self._publish_status(f'UNKNOWN_PROFILE: {phase}')
            return

        if phase == self.current_profile:
            self.get_logger().debug(
                f'Already in "{phase}" profile — no change.')
            return

        self._apply_profile(phase)

    def _apply_profile(self, profile_name: str):
        """Apply an impedance profile by publishing gains."""
        profile = PROFILES[profile_name]
        old_profile = self.current_profile
        self.current_profile = profile_name

        # Publish unified impedance params:
        # [kp1..kp7, kd1..kd7, grip_kp, grip_kd] = 16 values
        impedance_msg = Float64MultiArray()
        impedance_msg.data = [float(v) for v in
            profile['kp'] + profile['kd'] +
            [profile['grip_kp'], profile['grip_kd']]]
        self.impedance_pub.publish(impedance_msg)

        # Log transition
        kp_str = ', '.join(f'{v:.0f}' for v in profile['kp'])
        kd_str = ', '.join(f'{v:.2f}' for v in profile['kd'])

        transition = f'{old_profile} → {profile_name}' if old_profile else profile_name
        self.get_logger().info(
            f'Profile: {transition}\n'
            f'  Kp: [{kp_str}]\n'
            f'  Kd: [{kd_str}]\n'
            f'  Grip: Kp={profile["grip_kp"]}, Kd={profile["grip_kd"]}')

        self._publish_status(f'ACTIVE: {profile_name}')

    def _publish_status(self, status: str):
        """Publish manager status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main():
    rclpy.init()

    # Check for --side argument
    import argparse
    parser = argparse.ArgumentParser(
        description='Impedance Profile Manager (Task 2.4)')
    parser.add_argument('--side', type=str, default='right',
                        choices=['left', 'right'],
                        help='Which arm to manage (default: right)')
    args, _ = parser.parse_known_args()

    # Override ROS parameter with CLI arg
    node = ImpedanceProfileManager()
    # Note: the 'side' parameter is set via declare_parameter default

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down.')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

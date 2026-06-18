#!/usr/bin/env python3
"""
Force Monitor — Human-readable external force detection.

Subscribes to the compliance controller's external_force topic and prints
friendly messages when someone pushes/pulls the robot arm.

Reports which joints are being pushed or pulled, with estimated torque.

Usage:
    python3 force_monitor.py                   # monitor right arm (default)
    python3 force_monitor.py --side left       # monitor left arm
    python3 force_monitor.py --rate 5          # update 5 times/sec

Author: Agent-C1
Date: 2026-04-30
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


# Per-joint detection thresholds (Nm), calibrated from real hardware baseline.
# Baseline at J4=90° rest: [-0.82, -0.45, -0.06, 0.50, 0.16, 0.08, -0.15]
# We set thresholds at ~2x baseline to avoid false positives.
FORCE_THRESHOLDS = [1.5, 1.0, 0.5, 1.0, 0.5, 0.3, 0.5]

# Joint labels for human-readable output
JOINT_LABELS = ['J1 (shoulder)', 'J2 (shoulder)', 'J3 (upper arm)',
                'J4 (elbow)', 'J5 (wrist)', 'J6 (wrist)', 'J7 (wrist)']


class ForceMonitor(Node):
    """Real-time human-readable force detection display."""

    def __init__(self, side='right', rate=2.0):
        super().__init__('force_monitor')
        self.side = side
        self.rate = rate
        self.num_joints = 7

        # Latest force data
        self.tau_ext = [0.0] * self.num_joints
        self.has_data = False

        # Subscribe to external force topic
        topic = f'/{side}_compliance_controller/external_force'
        self.sub = self.create_subscription(
            Float64MultiArray, topic, self._force_cb, 10)

        # Display timer
        self.timer = self.create_timer(1.0 / rate, self._display)

        self.get_logger().info(
            f'Force Monitor started — listening on {topic}\n'
            f'  Thresholds (Nm): {FORCE_THRESHOLDS}\n'
            f'  Update rate: {rate} Hz\n'
            f'  Press Ctrl+C to stop.\n'
        )

    def _force_cb(self, msg: Float64MultiArray):
        """Store latest external force estimate."""
        if len(msg.data) >= self.num_joints:
            self.tau_ext = list(msg.data[:self.num_joints])
            self.has_data = True

    def _display(self):
        """Print human-readable force detection."""
        if not self.has_data:
            return

        # Find joints with significant force
        active_joints = []
        for i in range(self.num_joints):
            torque = self.tau_ext[i]
            threshold = FORCE_THRESHOLDS[i]
            if abs(torque) > threshold:
                direction = 'pushing' if torque > 0 else 'pulling'
                active_joints.append((i, torque, direction))

        # Build human-readable message
        if not active_joints:
            # Show raw values at lower frequency (every other update)
            raw = '  '.join(f'J{i+1}:{self.tau_ext[i]:+.2f}' for i in range(self.num_joints))
            print(f'\r🟢 No external force detected  [{raw}]', end='', flush=True)
        elif len(active_joints) == 1:
            i, torque, direction = active_joints[0]
            print(f'\n🔴 You are {direction} {JOINT_LABELS[i]}, '
                  f'with estimated torque of {abs(torque):.2f} Nm')
        else:
            # Multiple joints
            joint_strs = []
            for i, torque, direction in active_joints:
                joint_strs.append(f'{JOINT_LABELS[i]} ({direction}, {abs(torque):.2f} Nm)')
            joints_text = ' and '.join(joint_strs) if len(active_joints) == 2 \
                else ', '.join(joint_strs[:-1]) + f', and {joint_strs[-1]}'
            total_torque = math.sqrt(sum(t*t for _, t, _ in active_joints))
            print(f'\n🔴 Force detected on {joints_text}  '
                  f'[total: {total_torque:.2f} Nm]')


def main():
    parser = argparse.ArgumentParser(
        description='Human-readable external force monitor')
    parser.add_argument('--side', type=str, default='right',
                        choices=['left', 'right'],
                        help='Which arm to monitor (default: right)')
    parser.add_argument('--rate', type=float, default=2.0,
                        help='Display update rate in Hz (default: 2.0)')

    args, _ = parser.parse_known_args()

    rclpy.init()
    node = ForceMonitor(side=args.side, rate=args.rate)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nForce monitor stopped.')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

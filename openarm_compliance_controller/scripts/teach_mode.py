#!/usr/bin/env python3
"""Enable drag-to-teach mode for OpenArm data collection.

Publishes 'teach' to /impedance_phase which triggers the compliance controller
to set Kp=min, Kd=min with tau_ff (gravity compensation) still active.
The arm becomes "weightless" — safe to move by hand for recording demos.

Usage:
    python3 teach_mode.py [--side right]

Controls:
    g  — toggle gripper (open/close)
    t  — switch to teach mode (soft)
    s  — switch to transit mode (stiff, for safety)
    q  — restore transit mode and quit

Depends on:
    - C1's compliance_controller (running)
    - C1's impedance_profile_manager (running)

Agent-C2 (Vision/VLA) — Phase 3, Task 3.2
"""

import sys
import threading
import argparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState

# For non-blocking keyboard input
try:
    import termios
    import tty

    def _getch():
        """Read a single character from stdin without echo."""
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
except ImportError:
    def _getch():
        return input("key> ")[0] if input("key> ") else ""


class TeachMode(Node):
    """Enable teach mode for drag-to-teach data collection."""

    def __init__(self, side='right'):
        super().__init__('teach_mode')
        self.side = side
        self.gripper_open = False

        # Publishers
        self.phase_pub = self.create_publisher(
            String, '/impedance_phase', 10)
        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            f'/{side}_gripper_controller/commands',
            10)

        # Subscriber for joint state feedback
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 10)
        self.latest_joints = None

        # Set teach mode on startup
        self._set_phase('teach')
        self.get_logger().info(
            '╔══════════════════════════════════════════════╗')
        self.get_logger().info(
            '║       Teach Mode Active — Arm is Soft        ║')
        self.get_logger().info(
            '╠══════════════════════════════════════════════╣')
        self.get_logger().info(
            '║  g = toggle gripper                          ║')
        self.get_logger().info(
            '║  t = teach mode (soft)                       ║')
        self.get_logger().info(
            '║  s = transit mode (stiff/safe)               ║')
        self.get_logger().info(
            '║  p = print current joint positions           ║')
        self.get_logger().info(
            '║  q = quit (restores transit mode)            ║')
        self.get_logger().info(
            '╚══════════════════════════════════════════════╝')

    def _joint_cb(self, msg):
        """Cache latest joint state."""
        self.latest_joints = msg

    def _set_phase(self, phase):
        """Publish impedance phase."""
        msg = String()
        msg.data = phase
        self.phase_pub.publish(msg)
        self.get_logger().info(f'Impedance phase → {phase}')

    def _toggle_gripper(self):
        """Toggle gripper open/close."""
        self.gripper_open = not self.gripper_open
        msg = Float64MultiArray()
        msg.data = [0.032 if self.gripper_open else 0.0]  # 32mm = fully open
        self.gripper_pub.publish(msg)
        state = 'OPEN' if self.gripper_open else 'CLOSED'
        self.get_logger().info(f'Gripper → {state}')

    def _print_joints(self):
        """Print current joint positions."""
        if self.latest_joints is None:
            self.get_logger().info('No joint state received yet')
            return

        # Filter for right arm joints
        prefix = f'openarm_{self.side}_joint'
        joints = {}
        for name, pos in zip(self.latest_joints.name, self.latest_joints.position):
            if name.startswith(prefix) or 'finger' in name:
                short = name.replace(f'openarm_{self.side}_', '')
                joints[short] = pos

        joint_str = ', '.join(f'{k}={v:.4f}' for k, v in sorted(joints.items()))
        self.get_logger().info(f'Joints: {joint_str}')

    def run_keyboard_loop(self):
        """Block on keyboard input in a separate thread."""
        while rclpy.ok():
            try:
                key = _getch()
            except (EOFError, KeyboardInterrupt):
                break

            if key == 'g':
                self._toggle_gripper()
            elif key == 't':
                self._set_phase('teach')
            elif key == 's':
                self._set_phase('transit')
            elif key == 'p':
                self._print_joints()
            elif key == 'q':
                self._set_phase('transit')
                self.get_logger().info('Exiting teach mode — arm stiffened')
                rclpy.shutdown()
                break


def main():
    parser = argparse.ArgumentParser(description='Enable drag-to-teach mode')
    parser.add_argument('--side', default='right', choices=['left', 'right'],
                        help='Which arm to control')
    args = parser.parse_args()

    rclpy.init()
    node = TeachMode(side=args.side)

    # Spin ROS in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Keyboard input in main thread
    try:
        node.run_keyboard_loop()
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C — restoring transit mode')
        msg = String()
        msg.data = 'transit'
        node.phase_pub.publish(msg)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Move OpenArm right arm to the community's baseline pose.

This script commands the right arm to the community π0.5 model's
"home" position — the average pose from their Isaac Sim training data.

Purpose: Verify that the community's J4=+1.99, J7=-1.33 values correspond
to a natural "ready to pick" pose on our physical robot.

Usage:
    # Step 1: Launch robot in FAKE hardware mode first!
    ros2 launch openarm_bringup openarm.bimanual.launch.py use_fake_hardware:=true

    # Step 2: Run this script to see the pose in RViz
    python3 move_to_community_pose.py --mode mean

    # Step 3: If the RViz pose looks reasonable, try on real hardware:
    ros2 launch openarm_bringup openarm.bimanual.launch.py
    python3 move_to_community_pose.py --mode mean --speed slow
"""

import argparse
import time
import sys

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
import numpy as np

# Community data statistics (RIGHT arm: J1-J7)
COMMUNITY_POSES = {
    "mean": {
        "description": "Average pose during pick tasks (elbow bent, wrist rotated)",
        "joints": [0.393, 0.376, -0.196, 1.992, 0.496, -0.034, -1.327],
    },
    "q01": {
        "description": "Starting pose / minimum values during tasks",
        "joints": [-0.334, -0.140, -1.003, 0.573, -0.144, -0.786, -1.540],
    },
    "q50": {
        "description": "Estimated median (midpoint of q01-q99)",
        "joints": [0.567, 0.525, -0.234, 1.497, 0.603, -0.030, -1.096],
    },
    "model_pred": {
        "description": "What the model predicts from zero state (its 'go-to' pose)",
        "joints": [0.254, 0.066, -0.110, 1.275, 0.069, -0.025, -0.802],
    },
    "zero": {
        "description": "Our zero/home position (all joints at 0)",
        "joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    },
}

RIGHT_JOINT_NAMES = [
    "openarm_right_joint1",
    "openarm_right_joint2",
    "openarm_right_joint3",
    "openarm_right_joint4",
    "openarm_right_joint5",
    "openarm_right_joint6",
    "openarm_right_joint7",
]

SPEED_DURATIONS = {
    "fast": 2.0,    # 2 seconds to reach target
    "medium": 4.0,  # 4 seconds
    "slow": 8.0,    # 8 seconds (safest for real hardware)
}


class PoseMover(Node):
    def __init__(self):
        super().__init__("pose_mover")
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            "/right_joint_trajectory_controller/joint_trajectory",
            10,
        )
        self.current_joints = None
        self.create_subscription(
            JointState, "/joint_states",
            self._joint_cb, 10,
        )

    def _joint_cb(self, msg):
        positions = dict(zip(msg.name, msg.position))
        self.current_joints = [
            positions.get(name, 0.0) for name in RIGHT_JOINT_NAMES
        ]

    def move_to(self, target_joints, duration_s=4.0):
        """Send trajectory command to move to target pose."""
        msg = JointTrajectory()
        msg.joint_names = RIGHT_JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = list(target_joints)
        point.velocities = [0.0] * 7  # zero velocity at target
        point.time_from_start = Duration(
            sec=int(duration_s),
            nanosec=int((duration_s % 1) * 1e9),
        )
        msg.points = [point]
        self.traj_pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(
        description="Move right arm to community baseline pose"
    )
    parser.add_argument(
        "--mode", type=str, default="model_pred",
        choices=list(COMMUNITY_POSES.keys()),
        help="Which community pose to target",
    )
    parser.add_argument(
        "--speed", type=str, default="medium",
        choices=list(SPEED_DURATIONS.keys()),
        help="Movement speed (use 'slow' for real hardware)",
    )
    parser.add_argument(
        "--list", action="store_true",
        help="List all available poses and exit",
    )
    args = parser.parse_args()

    if args.list:
        print("Available poses:")
        for name, data in COMMUNITY_POSES.items():
            joints = data["joints"]
            print(f"\n  {name}: {data['description']}")
            for i, (jname, val) in enumerate(zip(["J1","J2","J3","J4","J5","J6","J7"], joints)):
                print(f"    {jname}: {val:+.3f} rad ({val*180/3.14159:+.1f}°)")
        return

    pose = COMMUNITY_POSES[args.mode]
    target = pose["joints"]
    duration = SPEED_DURATIONS[args.speed]

    print(f"=" * 60)
    print(f"Moving RIGHT arm to: {args.mode}")
    print(f"Description: {pose['description']}")
    print(f"Speed: {args.speed} ({duration}s)")
    print(f"=" * 60)
    print()
    for i, (jname, val) in enumerate(zip(["J1","J2","J3","J4","J5","J6","J7"], target)):
        print(f"  {jname}: {val:+.4f} rad ({val*180/3.14159:+.1f}°)")
    print()

    rclpy.init()
    node = PoseMover()

    # Wait for joint states
    print("Waiting for /joint_states...")
    timeout = time.time() + 10
    while node.current_joints is None and time.time() < timeout:
        rclpy.spin_once(node, timeout_sec=0.5)

    if node.current_joints is None:
        print("ERROR: No joint states. Is the robot launched?")
        node.destroy_node()
        rclpy.shutdown()
        return

    current = node.current_joints
    print(f"\nCurrent right arm position:")
    for i, (jname, val) in enumerate(zip(["J1","J2","J3","J4","J5","J6","J7"], current)):
        delta = target[i] - val
        print(f"  {jname}: {val:+.4f} → {target[i]:+.4f} (Δ={delta:+.4f} rad, {delta*180/3.14159:+.1f}°)")

    max_delta = max(abs(target[i] - current[i]) for i in range(7))
    print(f"\nMax movement: {max_delta:.3f} rad ({max_delta*180/3.14159:.1f}°)")

    # Confirmation
    response = input(f"\nProceed with movement? (yes/no): ").strip().lower()
    if response != "yes":
        print("Cancelled.")
        node.destroy_node()
        rclpy.shutdown()
        return

    print(f"\nMoving to {args.mode} pose over {duration}s...")
    node.move_to(target, duration)

    # Wait and monitor
    t0 = time.time()
    while time.time() - t0 < duration + 1.0:
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.current_joints:
            err = [abs(target[i] - node.current_joints[i]) for i in range(7)]
            max_err = max(err)
            elapsed = time.time() - t0
            if elapsed > 0 and int(elapsed * 2) % 2 == 0:
                print(f"  [{elapsed:.1f}s] Max error: {max_err:.4f} rad ({max_err*180/3.14159:.1f}°)")
            if max_err < 0.01 and elapsed > 1.0:
                print(f"  ✅ Target reached!")
                break

    print(f"\nFinal position:")
    if node.current_joints:
        for i, (jname, val) in enumerate(zip(["J1","J2","J3","J4","J5","J6","J7"], node.current_joints)):
            err = abs(target[i] - val)
            status = "✅" if err < 0.02 else "⚠️"
            print(f"  {jname}: {val:+.4f} (target: {target[i]:+.4f}, err: {err:.4f}) {status}")

    print("\n💡 Look at RViz — does this look like a 'ready to pick' pose?")
    print("   If yes, the community's J4/J7 values are just their default arm configuration.")
    print("   We can use this as our baseline pose for zero-shot inference.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

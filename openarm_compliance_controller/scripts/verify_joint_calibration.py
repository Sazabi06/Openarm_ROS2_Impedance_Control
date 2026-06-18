#!/usr/bin/env python3
"""Hardware-level joint verification for OpenArm.

Reads current joint states from the real robot and compares with
the community (Isaac Sim) checkpoint norm_stats to identify:
  1. Fixed offset constants
  2. Axis direction flips
  3. Gripper normalization differences

Usage:
    # 1. Power on robot, start ROS2 stack
    # 2. Put robot in a known pose (e.g. zero/home position)
    # 3. Run this script
    python3 verify_joint_calibration.py

    # Or read from a running ROS2 topic:
    python3 verify_joint_calibration.py --live
"""

import argparse
import json
import sys
import time

import numpy as np

# Community checkpoint norm stats (right arm, [J1..J7, grip])
COMM_RIGHT_MEAN = [0.3929, 0.3757, -0.1958, 1.9916, 0.4961, -0.0338, -1.3274, 0.1453]
COMM_RIGHT_Q01 =  [-0.334, -0.140, -1.003, 0.573, -0.144, -0.786, -1.540, 0.000]
COMM_RIGHT_Q99 =  [1.468, 1.190, 0.536, 2.420, 1.350, 0.726, -0.651, 0.900]

JOINT_LABELS = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'J7', 'Grip']

# ROS joint names for right arm
RIGHT_JOINTS = [
    'openarm_right_joint1',
    'openarm_right_joint2',
    'openarm_right_joint3',
    'openarm_right_joint4',
    'openarm_right_joint5',
    'openarm_right_joint6',
    'openarm_right_joint7',
    'openarm_right_finger_joint1',
]


def read_joint_states_ros():
    """Read current joint states from ROS2 /joint_states topic."""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = Node('joint_verifier')

    joint_positions = {}
    received = [False]

    def callback(msg):
        for name, pos in zip(msg.name, msg.position):
            joint_positions[name] = pos
        received[0] = True

    sub = node.create_subscription(JointState, '/joint_states', callback, 10)

    print("Waiting for /joint_states...")
    timeout = time.time() + 10
    while not received[0] and time.time() < timeout:
        rclpy.spin_once(node, timeout_sec=0.5)

    node.destroy_node()
    rclpy.shutdown()

    if not received[0]:
        print("ERROR: No joint states received within 10s!")
        return None

    # Extract right arm in order [J1..J7, grip]
    values = []
    for jname in RIGHT_JOINTS:
        if jname in joint_positions:
            values.append(joint_positions[jname])
        else:
            print(f"WARNING: Joint '{jname}' not found! Available: {list(joint_positions.keys())}")
            values.append(float('nan'))

    return np.array(values)


def analyze_differences(our_values, label="Current Pose"):
    """Compare our joint values with community norm stats."""
    print(f"\n{'='*70}")
    print(f"JOINT CALIBRATION ANALYSIS: {label}")
    print(f"{'='*70}")
    print()
    print(f"{'Joint':<6} {'Ours':>10} {'Comm_mean':>10} {'Diff':>10} {'Comm [q01, q99]':>24} {'Status':>12}")
    print('-' * 74)

    issues = []
    for i in range(8):
        ov = our_values[i]
        cm = COMM_RIGHT_MEAN[i]
        diff = ov - cm
        q01 = COMM_RIGHT_Q01[i]
        q99 = COMM_RIGHT_Q99[i]

        in_range = q01 <= ov <= q99
        status = "✅ OK" if in_range else "⚠️ OUT"

        if abs(diff) > 0.5:
            status = "❌ BIG DIFF"
            issues.append((i, JOINT_LABELS[i], diff))

        print(f"{JOINT_LABELS[i]:<6} {ov:>+10.4f} {cm:>+10.4f} {diff:>+10.4f} [{q01:>+8.4f}, {q99:>+8.4f}] {status:>12}")

    print()

    if not issues:
        print("🎉 All joints within community range!")
        print("   → Community checkpoint might work with minimal offset mapping.")
    else:
        print(f"⚠️  {len(issues)} joints have large differences:")
        for idx, label, diff in issues:
            deg = diff * 180 / 3.14159
            print(f"   {label}: offset = {diff:+.4f} rad ({deg:+.1f}°)")

            # Check if it's a sign flip
            if abs(our_values[idx] + COMM_RIGHT_MEAN[idx]) < abs(diff) * 0.5:
                print(f"      → Likely AXIS DIRECTION FLIP (sign flip)")
            elif abs(diff) > 1.0:
                print(f"      → Likely ZERO POSITION OFFSET")
            else:
                print(f"      → Could be task-dependent (different poses)")

    print()
    print("RECOMMENDATION:")
    if len(issues) == 0:
        print("  Try community checkpoint directly with safety clamping.")
    elif all(abs(d) < 0.3 for _, _, d in issues):
        print("  Differences are small. Fine-tune on our data should fix them.")
    else:
        print("  Large offsets detected. Investigate:")
        print("  1. Run zero-position calibration: openarm-can-zero-position-calibration")
        print("  2. If offsets are constant, add offset mapping in inference pipeline")
        print("  3. If offsets vary, must fine-tune on our own data")


def main():
    parser = argparse.ArgumentParser(description='Verify OpenArm joint calibration')
    parser.add_argument('--live', action='store_true',
                        help='Read from live ROS2 /joint_states topic')
    parser.add_argument('--values', nargs=8, type=float,
                        metavar=('J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'J7', 'GRIP'),
                        help='Manual joint values [J1..J7, grip] in radians')
    args = parser.parse_args()

    if args.live:
        values = read_joint_states_ros()
        if values is None:
            sys.exit(1)
        analyze_differences(values, "Live Joint States")

    elif args.values:
        values = np.array(args.values)
        analyze_differences(values, "Manual Input")

    else:
        print("No input specified. Analyzing existing recorded data...")
        print()

        # Load our existing dataset and analyze
        import pyarrow.parquet as pq
        t = pq.read_table('/home/user/lerobot_dataset/data/chunk-000/episode_000000.parquet',
                          columns=['observation.state'])
        states = np.array(t.column('observation.state').to_pylist())

        # Data is in alphabetical order: [gripper, J1, J2, J3, J4, J5, J6, J7]
        # Reorder to [J1..J7, grip]
        reordered_mean = np.array([states[:, i].mean() for i in [1, 2, 3, 4, 5, 6, 7, 0]])
        analyze_differences(reordered_mean, "Recorded Dataset (ep 0 mean)")

        # Also show frame 0 (likely home/start pose)
        frame0 = np.array([states[0, i] for i in [1, 2, 3, 4, 5, 6, 7, 0]])
        analyze_differences(frame0, "Recorded Dataset (ep 0 frame 0 - start pose)")


if __name__ == '__main__':
    main()

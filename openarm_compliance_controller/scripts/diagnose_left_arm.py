#!/usr/bin/env python3
"""
Left Arm Joint Direction Diagnostic
====================================
Usage: Enable teach mode, then manually move each left arm joint one at a time.
This script shows the raw motor reading and what RViz "sees", helping determine
which joints need negation or offset in the hardware interface.

Run:
  python3 ~/ros2_ws/src/impedance_control/openarm_compliance_controller/scripts/diagnose_left_arm.py

Requires: bimanual bringup running, both arms in teach mode.
"""
import rclpy
import math
import time
from sensor_msgs.msg import JointState

def main():
    rclpy.init()
    node = rclpy.create_node('left_arm_diag')

    # Store latest joint states
    latest = {}

    def js_cb(msg):
        for name, pos in zip(msg.name, msg.position):
            latest[name] = pos

    sub = node.create_subscription(JointState, '/joint_states', js_cb, 10)

    print("=" * 70)
    print("LEFT ARM JOINT DIRECTION DIAGNOSTIC")
    print("=" * 70)
    print()
    print("Instructions:")
    print("  1. Both arms should be in teach mode (floating)")
    print("  2. Start with both arms at home/zero position")
    print("  3. Move ONE joint at a time on the LEFT arm")
    print("  4. Compare the direction with the SAME joint on the right arm")
    print()
    print("URDF axis definitions (from generated URDF):")
    print("  J1: axis=(0,0,1)   — same for both arms")
    print("  J2: axis=(-1,0,0)  — same, but link RPY flipped ±90°")
    print("  J3: axis=(0,0,1)   — identical")
    print("  J4: axis=(0,1,0)   — identical (forced reflect=1)")
    print("  J5: axis=(0,0,1)   — identical")
    print("  J6: axis=(1,0,0)   — identical")
    print("  J7: axis=(0,±1,0)  — LEFT=-1, RIGHT=+1 (FLIPPED!)")
    print()
    print("-" * 70)
    print(f"{'Joint':>5} | {'Left (raw)':>12} | {'Right (raw)':>12} | {'Left Δ':>10} | {'Right Δ':>10}")
    print("-" * 70)

    # Capture initial values
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)

    initial = dict(latest)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)

            lines = []
            for i in range(1, 8):
                ln = f'openarm_left_joint{i}'
                rn = f'openarm_right_joint{i}'
                lv = latest.get(ln, 0)
                rv = latest.get(rn, 0)
                l0 = initial.get(ln, 0)
                r0 = initial.get(rn, 0)
                ld = lv - l0
                rd = rv - r0

                # Highlight if delta is significant
                flag = ""
                if abs(ld) > 0.05:  # > ~3 degrees
                    flag = " ←← MOVING"
                lines.append(
                    f"  J{i}  | {math.degrees(lv):+10.2f}° | {math.degrees(rv):+10.2f}° | "
                    f"{math.degrees(ld):+8.2f}° | {math.degrees(rd):+8.2f}°{flag}"
                )

            # Clear and redraw
            print(f"\033[{7}A", end="")  # Move cursor up
            for line in lines:
                print(f"\r{line:80s}")

    except KeyboardInterrupt:
        print("\n\nFinal readings:")
        print("-" * 70)
        for i in range(1, 8):
            ln = f'openarm_left_joint{i}'
            rn = f'openarm_right_joint{i}'
            lv = latest.get(ln, 0)
            rv = latest.get(rn, 0)
            l0 = initial.get(ln, 0)
            r0 = initial.get(rn, 0)
            ld = lv - l0
            rd = rv - r0
            print(f"  J{i}: Left Δ={math.degrees(ld):+.1f}°  Right Δ={math.degrees(rd):+.1f}°")
            if abs(ld) > 0.05:
                if (ld > 0) == (rd > 0):
                    print(f"       → Same direction (no negation needed)")
                else:
                    print(f"       → OPPOSITE direction (NEEDS negation)")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

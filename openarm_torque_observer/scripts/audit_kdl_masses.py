#!/usr/bin/env python3
# Copyright 2026 OpenArm Contributors
# SPDX-License-Identifier: Apache-2.0
"""
KDL Mass Audit Script — Standalone diagnostic tool.

Generates the URDF from xacro, builds the KDL chain, and prints:
  1. Each segment's mass, CoM, and joint type
  2. Cumulative downstream mass for each joint
  3. Gravity torques at key poses (0°, 45°, 90° for each joint)

Usage:
  python3 audit_kdl_masses.py [--bimanual] [--side right|left]
"""

import argparse
import subprocess
import sys
import math

try:
    import PyKDL
except ImportError:
    print("ERROR: PyKDL not found. Install with: sudo apt install ros-humble-python-orocos-kdl-vendor")
    sys.exit(1)

# Reuse the URDF→KDL builder from the observer
sys.path.insert(0, str(__import__('pathlib').Path(__file__).resolve().parent.parent / 'openarm_torque_observer'))
from torque_observer_node import kdl_tree_from_urdf_string


def generate_urdf(bimanual: bool) -> str:
    """Generate URDF string from xacro."""
    xacro_path = '/home/user/ros2_ws/src/openarm_description/urdf/robot/v10.urdf.xacro'
    cmd = ['xacro', xacro_path]
    if bimanual:
        cmd.append('bimanual:=true')
    cmd.append('hand:=true')

    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"ERROR: xacro failed:\n{result.stderr}")
        sys.exit(1)
    return result.stdout


def audit_chain(tree, root_link: str, tip_link: str, label: str):
    """Audit a single KDL chain."""
    chain = tree.getChain(root_link, tip_link)
    n_joints = chain.getNrOfJoints()
    n_segments = chain.getNrOfSegments()

    print(f"\n{'='*70}")
    print(f"  Chain: {root_link} → {tip_link}  ({label})")
    print(f"  Joints: {n_joints}, Segments: {n_segments}")
    print(f"{'='*70}")

    # ---- Segment details ----
    print(f"\n{'Segment':<35} {'Joint Type':<12} {'Mass (kg)':<12} {'CoM (x,y,z) m'}")
    print("-" * 80)

    total_mass = 0.0
    segment_masses = []
    for i in range(n_segments):
        seg = chain.getSegment(i)
        inertia = seg.getInertia()
        m = inertia.getMass()
        total_mass += m
        segment_masses.append((seg.getName(), m))
        cog = inertia.getCOG()
        jtype = seg.getJoint().getTypeName()
        mass_str = f"{m:.4f}" if m > 0.001 else "  —"
        cog_str = f"({cog.x():.4f}, {cog.y():.4f}, {cog.z():.4f})" if m > 0.001 else "—"
        print(f"  {seg.getName():<33} {jtype:<12} {mass_str:<12} {cog_str}")

    print(f"\n  TOTAL CHAIN MASS: {total_mass:.4f} kg")

    # ---- Cumulative downstream mass (joints only) ----
    print(f"\n{'Joint':<10} {'Downstream Mass (kg)':<25} {'Description'}")
    print("-" * 60)
    joint_idx = 0
    for i in range(n_segments):
        seg = chain.getSegment(i)
        if seg.getJoint().getTypeName() == "None":
            continue  # fixed joint
        # Sum mass of all segments from i+1 to end
        downstream = sum(chain.getSegment(j).getInertia().getMass()
                         for j in range(i + 1, n_segments))
        # Include this segment's own mass (the link moved by this joint)
        self_mass = seg.getInertia().getMass()
        downstream += self_mass
        joint_idx += 1
        print(f"  J{joint_idx:<7} {downstream:<25.4f} {seg.getName()}")

    # ---- Gravity torques at key poses ----
    print(f"\nGravity torques at key poses (Nm):")
    print(f"{'Pose':<25}", end="")
    for j in range(1, n_joints + 1):
        print(f"{'J' + str(j):<10}", end="")
    print()
    print("-" * (25 + 10 * n_joints))

    solver = PyKDL.ChainDynParam(chain, PyKDL.Vector(0, 0, -9.81))

    # All zeros
    poses = {"All zeros": [0.0] * n_joints}

    # Each joint at 45° and 90°
    for target_j in range(n_joints):
        for angle_deg, angle_rad in [(45, math.pi / 4), (90, math.pi / 2)]:
            name = f"J{target_j + 1}={angle_deg}°"
            q_list = [0.0] * n_joints
            q_list[target_j] = angle_rad
            poses[name] = q_list

    for pose_name, q_list in poses.items():
        q = PyKDL.JntArray(n_joints)
        for i in range(n_joints):
            q[i] = q_list[i]
        tau = PyKDL.JntArray(n_joints)
        solver.JntToGravity(q, tau)
        print(f"  {pose_name:<23}", end="")
        for i in range(n_joints):
            print(f"{tau[i]:<10.3f}", end="")
        print()


def main():
    parser = argparse.ArgumentParser(description="KDL Mass Audit for OpenArm")
    parser.add_argument("--bimanual", action="store_true",
                        help="Use bimanual URDF (default: single arm)")
    parser.add_argument("--side", default="right", choices=["right", "left"],
                        help="Which arm to audit in bimanual mode")
    args = parser.parse_args()

    print("Generating URDF from xacro...")
    urdf_string = generate_urdf(args.bimanual)
    print(f"URDF generated ({len(urdf_string)} bytes)")

    print("Building KDL tree...")
    ok, tree = kdl_tree_from_urdf_string(urdf_string)
    if not ok:
        print("ERROR: Failed to build KDL tree")
        sys.exit(1)

    if args.bimanual:
        root = "openarm_body_link0"
        tip = f"openarm_{args.side}_hand"
        audit_chain(tree, root, tip, f"{args.side} arm (bimanual)")
    else:
        # Try single arm chain
        audit_chain(tree, "openarm_link0", "openarm_hand", "single arm")


if __name__ == "__main__":
    main()

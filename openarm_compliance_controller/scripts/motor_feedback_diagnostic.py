#!/usr/bin/env python3
"""
Motor Feedback Diagnostic Script for OpenArm

Purpose: Determine exactly what feedback data is available from DaMiao motors
via the ros2_control hardware interface. This is critical for understanding
whether we can use motor torque feedback for proprioceptive force estimation.

What it checks:
  1. /joint_states — position, velocity, effort (what is 'effort'?)
  2. /right_joint_temperatures — tmos, trotor
  3. /right_compliance_controller/tau_ff — model-predicted feedforward torque

Usage:
  # First, launch the robot (real or sim):
  ros2 launch openarm_bringup openarm.bimanual.launch.py use_fake_hardware:=true

  # Then run this diagnostic:
  python3 motor_feedback_diagnostic.py

  # For real hardware:
  ros2 launch openarm_bringup openarm.bimanual.launch.py use_fake_hardware:=false
  python3 motor_feedback_diagnostic.py --real
"""

import sys
import argparse
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class MotorDiagnostic(Node):
    def __init__(self, real_hw: bool):
        super().__init__('motor_feedback_diagnostic')
        self.real_hw = real_hw
        self.joint_data = None
        self.temp_data = None
        self.tau_ff_data = None
        self.gains_data = None

        # Subscribe to joint states
        self.js_sub = self.create_subscription(
            JointState, '/joint_states', self._js_cb, 10)

        # Subscribe to temperatures (real hardware only)
        self.temp_sub = self.create_subscription(
            Float64MultiArray, '/right_joint_temperatures', self._temp_cb, 10)

        # Subscribe to tau_ff (if compliance controller is running)
        self.tau_sub = self.create_subscription(
            Float64MultiArray,
            '/right_compliance_controller/tau_ff', self._tau_cb, 10)

        # Subscribe to gains (if compliance controller is running)
        self.gains_sub = self.create_subscription(
            Float64MultiArray,
            '/right_compliance_controller/gains', self._gains_cb, 10)

        self.get_logger().info(
            f'Motor Diagnostic started ({"REAL HW" if real_hw else "SIMULATION"})')
        self.get_logger().info('Waiting 3 seconds for data...')

    def _js_cb(self, msg):
        self.joint_data = msg

    def _temp_cb(self, msg):
        self.temp_data = msg

    def _tau_cb(self, msg):
        self.tau_ff_data = msg

    def _gains_cb(self, msg):
        self.gains_data = msg

    def print_report(self):
        print("\n" + "=" * 80)
        print("  OPENARM MOTOR FEEDBACK DIAGNOSTIC REPORT")
        print("  " + ("REAL HARDWARE" if self.real_hw else "SIMULATION (fake_hardware)"))
        print("=" * 80)

        # ─── 1. Joint States ───
        print("\n┌─── 1. /joint_states ───────────────────────────────────────────┐")
        if self.joint_data is None:
            print("│  ❌ NO DATA — /joint_states not publishing!")
            print("│     Is the robot bringup running?")
        else:
            # Filter right arm joints only
            right_joints = []
            for i, name in enumerate(self.joint_data.name):
                if 'right_joint' in name and 'finger' not in name:
                    pos = self.joint_data.position[i] if i < len(self.joint_data.position) else float('nan')
                    vel = self.joint_data.velocity[i] if i < len(self.joint_data.velocity) else float('nan')
                    eff = self.joint_data.effort[i] if i < len(self.joint_data.effort) else float('nan')
                    right_joints.append((name, pos, vel, eff))

            if not right_joints:
                print("│  ❌ No right arm joints found in /joint_states")
            else:
                print(f"│  ✅ Found {len(right_joints)} right arm joints")
                print("│")
                print("│  {:30s} {:>10s} {:>10s} {:>10s}".format(
                    "Joint", "Position", "Velocity", "Effort"))
                print("│  " + "-" * 62)
                for name, pos, vel, eff in right_joints:
                    print("│  {:30s} {:>10.4f} {:>10.4f} {:>10.4f}".format(
                        name, pos, vel, eff))
                print("│")
                print("│  ┌─── KEY QUESTION ─────────────────────────────────────┐")
                print("│  │ What does 'effort' represent?                        │")
                all_zero = all(abs(eff) < 0.001 for _, _, _, eff in right_joints)
                if all_zero:
                    print("│  │ ⚠️  All effort values are ~0.                      │")
                    print("│  │ In SIMULATION: This is expected (fake HW).         │")
                    print("│  │ On REAL HW: Motor may not be reporting torque.     │")
                else:
                    print("│  │ ✅ Non-zero effort values detected!                │")
                    print("│  │ This likely represents MOTOR TORQUE FEEDBACK       │")
                    print("│  │ from DaMiao MIT mode (τ = Kt × I_motor).          │")
                    print("│  │                                                    │")
                    print("│  │ This is the key signal for proprioceptive          │")
                    print("│  │ force estimation WITHOUT an F/T sensor!            │")
                print("│  └────────────────────────────────────────────────────┘")
        print("└────────────────────────────────────────────────────────────────┘")

        # ─── 2. Temperature ───
        print("\n┌─── 2. /right_joint_temperatures ───────────────────────────────┐")
        if self.temp_data is None:
            print("│  ❌ NO DATA — temperatures not publishing")
            if not self.real_hw:
                print("│     (Expected in simulation — fake HW has no thermal data)")
        else:
            print(f"│  ✅ Received {len(self.temp_data.data)} temperature values")
            for i, t in enumerate(self.temp_data.data):
                label = f"J{i+1}" if i < 7 else "Gripper"
                status = "🟢 OK" if t < 55 else "🟡 Warm" if t < 65 else "🔴 HOT"
                print(f"│  {label:10s}: {t:6.1f}°C  {status}")
        print("└────────────────────────────────────────────────────────────────┘")

        # ─── 3. Compliance Controller ───
        print("\n┌─── 3. Compliance Controller Status ────────────────────────────┐")
        if self.tau_ff_data is None:
            print("│  ⚠️  /right_compliance_controller/tau_ff — NOT PUBLISHING")
            print("│     Compliance controller may not be spawned.")
        else:
            print(f"│  ✅ tau_ff: {len(self.tau_ff_data.data)} values")
            for i, t in enumerate(self.tau_ff_data.data):
                print(f"│     J{i+1}: τ_ff = {t:+8.4f} Nm")

        if self.gains_data is None:
            print("│  ⚠️  /right_compliance_controller/gains — NOT PUBLISHING")
        else:
            n = len(self.gains_data.data) // 2
            print(f"│  ✅ gains: {n} joints")
            for i in range(n):
                kp = self.gains_data.data[i]
                kd = self.gains_data.data[i + n]
                print(f"│     J{i+1}: Kp = {kp:.1f}, Kd = {kd:.2f}")
        print("└────────────────────────────────────────────────────────────────┘")

        # ─── 4. Force Estimation Feasibility ───
        print("\n┌─── 4. PROPRIOCEPTIVE FORCE ESTIMATION FEASIBILITY ─────────────┐")
        if self.joint_data and self.tau_ff_data:
            # Build name→effort map from /joint_states (which is alphabetically sorted)
            effort_map = {}
            for i, name in enumerate(self.joint_data.name):
                if 'right_joint' in name and 'finger' not in name:
                    eff = self.joint_data.effort[i] if i < len(self.joint_data.effort) else 0.0
                    effort_map[name] = eff

            # tau_ff is in controller joint order: J1, J2, ..., J7
            controller_joint_order = [f"openarm_right_joint{j}" for j in range(1, 8)]
            n_tau = len(self.tau_ff_data.data)

            if len(effort_map) == n_tau and n_tau == 7:
                print("│  Both motor effort feedback and model τ_ff available!")
                print("│  (Matched by joint name to fix ordering)")
                print("│")
                print("│  {:5s} {:>12s} {:>12s} {:>12s}".format(
                    "Joint", "τ_motor(HW)", "τ_model(KDL)", "τ_ext_est"))
                print("│  " + "-" * 45)
                efforts_ordered = []
                for i, jname in enumerate(controller_joint_order):
                    tau_motor = effort_map.get(jname, 0.0)
                    tau_model = self.tau_ff_data.data[i]
                    tau_ext = tau_motor - tau_model
                    efforts_ordered.append(tau_motor)
                    print(f"│  J{i+1}:   {tau_motor:>+10.4f}   {tau_model:>+10.4f}   {tau_ext:>+10.4f}")
                print("│")
                print("│  τ_ext_estimate = τ_motor(from HW) - τ_model(from KDL)")
                print("│  Note: At zero position with no external load, τ_ext")
                print("│  represents model error + friction + cable forces.")
                print("│")

                all_effort_zero = all(abs(e) < 0.001 for e in efforts_ordered)
                if all_effort_zero:
                    print("│  ⚠️  Motor effort is all zeros.")
                    if self.real_hw:
                        print("│     → Check if DaMiao CAN feedback includes torque.")
                        print("│     → The HW interface reads arm_motors[i].get_torque()")
                        print("│     → This should be the MIT mode torque feedback.")
                    else:
                        print("│     → Expected in simulation (fake hardware).")
                        print("│     → Run with --real on actual robot to verify.")
                else:
                    print("│  ✅ FORCE ESTIMATION IS FEASIBLE!")
                    print("│     Motor torque feedback is non-zero.")
                    print("│     τ_ext ≈ τ_motor - τ_model can estimate external forces.")
            else:
                print(f"│  ⚠️  Size mismatch: effort_map={len(effort_map)}, tau_ff={n_tau}")
        else:
            missing = []
            if not self.joint_data:
                missing.append("/joint_states")
            if not self.tau_ff_data:
                missing.append("tau_ff (compliance controller)")
            print(f"│  ❌ Cannot assess — missing: {', '.join(missing)}")
        print("└────────────────────────────────────────────────────────────────┘")

        # ─── 5. Action Items ───
        print("\n┌─── 5. RECOMMENDED NEXT STEPS ─────────────────────────────────┐")
        if not self.real_hw:
            print("│  1. Re-run this script with --real on actual hardware")
            print("│     to verify motor torque feedback is non-zero.")
        else:
            if self.joint_data:
                right_efforts = [
                    self.joint_data.effort[i]
                    for i, n in enumerate(self.joint_data.name)
                    if 'right_joint' in n and 'finger' not in n
                ]
                if any(abs(e) > 0.001 for e in right_efforts):
                    print("│  ✅ Motor torque feedback confirmed!")
                    print("│  → We can use τ_ext = τ_motor - τ_model for")
                    print("│    proprioceptive force estimation (no F/T needed).")
                    print("│  → Next: Implement momentum observer for accuracy.")
                else:
                    print("│  ⚠️  Motor effort is zero on real hardware.")
                    print("│  → DaMiao MIT mode SHOULD return torque feedback.")
                    print("│  → Check: Does state_tau_ get populated in CAN decode?")
                    print("│  → Check: openarm_can dm_motor.hpp update_state()")
        print("│")
        print("│  To test with the compliance controller active:")
        print("│  ros2 run controller_manager spawner right_compliance_controller \\")
        print("│    -c /controller_manager \\")
        print("│    --param-file $(ros2 pkg prefix openarm_compliance_controller)\\"
              )
        print("│    /share/openarm_compliance_controller/config/compliance_controller.yaml")
        print("│")
        print("│  Then re-run this script to see tau_ff vs motor effort.")
        print("└────────────────────────────────────────────────────────────────┘")
        print()


def main():
    parser = argparse.ArgumentParser(description='OpenArm Motor Feedback Diagnostic')
    parser.add_argument('--real', action='store_true',
                        help='Indicate running on real hardware (affects interpretation)')
    parser.add_argument('--wait', type=float, default=3.0,
                        help='Seconds to wait for data collection (default: 3.0)')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = MotorDiagnostic(real_hw=args.real)

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # Spin for a few seconds to collect data
    start = time.time()
    while time.time() - start < args.wait:
        executor.spin_once(timeout_sec=0.1)

    # Print report
    node.print_report()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

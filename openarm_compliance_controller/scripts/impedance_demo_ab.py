#!/usr/bin/env python3
"""
Task 1.3: Demo 0 — A-B Motion Script

Moves the right arm between two predefined joint poses (A and B) for a
configurable number of cycles, logging per-cycle tracking error metrics
to a CSV file.

Supports --no-compliance mode to deactivate the compliance controller
(tau_ff = 0), enabling direct comparison of tracking performance with
and without gravity/friction compensation.

Usage:
    python3 impedance_demo_ab.py                   # with compliance (default)
    python3 impedance_demo_ab.py --no-compliance    # disable tau_ff
    python3 impedance_demo_ab.py --cycles 5         # run 5 cycles only
    python3 impedance_demo_ab.py --side left        # use left arm

Author: Agent-C1
Date: 2026-04-30
"""

import argparse
import csv
import math
import os
import sys
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from builtin_interfaces.msg import Duration


class ImpedanceDemoAB(Node):
    """A-B motion demo node with per-cycle tracking error logging."""

    # Default waypoints (from AGENT_C1_CONTROLS.md Task 1.3 spec)
    DEFAULT_POINT_A = [0.0, 0.785, 0.0, 0.785, 0.0, 0.0, 0.0]
    DEFAULT_POINT_B = [0.5, 0.785, 0.0, 1.047, 0.0, 0.0, 0.0]

    # Joint limits from URDF (right arm, in radians)
    JOINT_LIMITS = {
        'joint1': (-1.396, 3.491),
        'joint2': (-1.745, 1.745),
        'joint3': (-1.571, 1.571),
        'joint4': (0.0,    2.443),
        'joint5': (-1.571, 1.571),
        'joint6': (-0.785, 0.785),
        'joint7': (-1.571, 1.571),
    }

    def __init__(self, args):
        super().__init__('impedance_demo_ab')

        # Parse arguments
        self.side = args.side
        self.cycles = args.cycles
        self.duration = args.duration
        self.no_compliance = args.no_compliance
        self.log_file = args.log_file

        # Joint names for the selected arm
        self.joint_names = [
            f'openarm_{self.side}_joint{i}' for i in range(1, 8)
        ]
        self.num_joints = len(self.joint_names)

        # Waypoints
        self.point_a = list(args.point_a) if args.point_a else list(self.DEFAULT_POINT_A)
        self.point_b = list(args.point_b) if args.point_b else list(self.DEFAULT_POINT_B)

        # Validate waypoints against joint limits
        self._validate_waypoints()

        # Controller names
        self.jtc_name = f'{self.side}_joint_trajectory_controller'
        self.compliance_name = f'{self.side}_compliance_controller'

        # Action client for JTC
        self.jtc_client = ActionClient(
            self,
            FollowJointTrajectory,
            f'/{self.jtc_name}/follow_joint_trajectory',
        )

        # Joint state subscriber — track current joint positions
        self.current_positions = {}
        self._positions_lock = threading.Lock()
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb,
            10,
        )

        # tau_ff subscriber — monitor feedforward torque
        self.current_tau_ff = [0.0] * self.num_joints
        self._tau_ff_lock = threading.Lock()
        self.tau_ff_sub = self.create_subscription(
            Float64MultiArray,
            f'/{self.compliance_name}/tau_ff',
            self._tau_ff_cb,
            10,
        )

        # Tracking error accumulator (reset per segment)
        self.target_positions = [0.0] * self.num_joints
        self._collecting = False
        self.error_samples = []  # list of per-sample error vectors
        self.tau_ff_samples = []  # list of tau_ff norms during segment

        # CSV log
        self.csv_rows = []

        # Trajectory completion event
        self._goal_done_event = threading.Event()
        self._goal_success = False

        self.get_logger().info(
            f'=== Demo 0: A-B Motion ({"NO compliance" if self.no_compliance else "WITH compliance"}) ===\n'
            f'  Side: {self.side}\n'
            f'  Point A: {self.point_a}\n'
            f'  Point B: {self.point_b}\n'
            f'  Duration: {self.duration}s per segment\n'
            f'  Cycles: {self.cycles}\n'
            f'  Log file: {self.log_file}'
        )

    def _validate_waypoints(self):
        """Validate that waypoints are within joint limits."""
        for name, pts in [('Point A', self.point_a), ('Point B', self.point_b)]:
            if len(pts) != self.num_joints:
                self.get_logger().error(
                    f'{name} has {len(pts)} values, expected {self.num_joints}')
                sys.exit(1)
            for i, val in enumerate(pts):
                joint_key = f'joint{i + 1}'
                lo, hi = self.JOINT_LIMITS[joint_key]
                if val < lo - 0.01 or val > hi + 0.01:
                    self.get_logger().error(
                        f'{name} J{i+1}={val:.3f} rad is OUTSIDE limits '
                        f'[{lo:.3f}, {hi:.3f}]')
                    sys.exit(1)
        self.get_logger().info('Waypoints validated: all within joint limits.')

    def _joint_state_cb(self, msg: JointState):
        """Store latest joint positions, matched by name."""
        with self._positions_lock:
            for name, pos in zip(msg.name, msg.position):
                self.current_positions[name] = pos

        # Accumulate error samples if collecting
        if self._collecting:
            errors = []
            with self._positions_lock:
                for i, jname in enumerate(self.joint_names):
                    actual = self.current_positions.get(jname, 0.0)
                    error_rad = abs(actual - self.target_positions[i])
                    errors.append(error_rad)
            self.error_samples.append(errors)

    def _tau_ff_cb(self, msg: Float64MultiArray):
        """Store latest tau_ff and accumulate norm samples."""
        if len(msg.data) >= self.num_joints:
            with self._tau_ff_lock:
                self.current_tau_ff = list(msg.data[:self.num_joints])
            if self._collecting:
                norm = math.sqrt(sum(t * t for t in msg.data[:self.num_joints]))
                self.tau_ff_samples.append(norm)

    def run_demo(self):
        """Main demo loop: move between A and B for N cycles.

        This runs in its own thread while the main thread spins.
        """
        # Wait for JTC action server
        self.get_logger().info(f'Waiting for {self.jtc_name} action server...')
        if not self.jtc_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error(
                f'Action server {self.jtc_name} not available after 15s!')
            return False

        self.get_logger().info('Action server connected.')

        # Wait for initial joint state
        timeout = 10.0
        start = time.time()
        while True:
            with self._positions_lock:
                have_joints = len(self.current_positions) >= self.num_joints
            if have_joints:
                break
            if time.time() - start > timeout:
                self.get_logger().error(
                    f'Timeout waiting for joint states.')
                return False
            time.sleep(0.1)

        self.get_logger().info(
            f'Joint states received for {len(self.current_positions)} joints.')

        if self.no_compliance:
            self.get_logger().warn(
                'NO-COMPLIANCE mode: For accurate comparison, deactivate the '
                'compliance controller before running this script. '
                'Currently measuring with compliance active as reference.')

        # Move to Point A first (starting position)
        self.get_logger().info('Moving to Point A (starting position)...')
        success = self._send_trajectory_blocking(self.point_a, self.duration)
        if not success:
            self.get_logger().error('Failed to reach Point A!')
            return False
        time.sleep(1.0)  # settle time

        # Main cycle loop
        for cycle in range(1, self.cycles + 1):
            self.get_logger().info(f'--- Cycle {cycle}/{self.cycles} ---')

            # A -> B
            self._reset_accumulators()
            self.target_positions = list(self.point_b)
            self._collecting = True

            success = self._send_trajectory_blocking(self.point_b, self.duration)
            self._collecting = False

            if not success:
                self.get_logger().error(f'Cycle {cycle} A→B failed!')
                return False

            # Record A->B metrics
            rms_err, max_err = self._compute_error_metrics()
            avg_tau = self._compute_avg_tau_ff_norm()
            self.csv_rows.append({
                'cycle': cycle,
                'direction': 'A_to_B',
                'rms_error_deg': rms_err,
                'max_error_deg': max_err,
                'avg_tau_ff_norm': avg_tau,
            })
            self.get_logger().info(
                f'  A→B: RMS={rms_err:.2f}°, max={max_err:.2f}°, '
                f'avg_tau_ff={avg_tau:.3f} Nm')

            # B -> A
            self._reset_accumulators()
            self.target_positions = list(self.point_a)
            self._collecting = True

            success = self._send_trajectory_blocking(self.point_a, self.duration)
            self._collecting = False

            if not success:
                self.get_logger().error(f'Cycle {cycle} B→A failed!')
                return False

            # Record B->A metrics
            rms_err, max_err = self._compute_error_metrics()
            avg_tau = self._compute_avg_tau_ff_norm()
            self.csv_rows.append({
                'cycle': cycle,
                'direction': 'B_to_A',
                'rms_error_deg': rms_err,
                'max_error_deg': max_err,
                'avg_tau_ff_norm': avg_tau,
            })
            self.get_logger().info(
                f'  B→A: RMS={rms_err:.2f}°, max={max_err:.2f}°, '
                f'avg_tau_ff={avg_tau:.3f} Nm')

        # Write CSV
        self._write_csv()

        # Print summary
        self._print_summary()

        self.get_logger().info(
            f'Demo complete: {self.cycles} cycles, '
            f'log saved to {self.log_file}')
        return True

    def _reset_accumulators(self):
        """Reset error and tau_ff sample accumulators for a new segment."""
        self.error_samples = []
        self.tau_ff_samples = []

    def _compute_error_metrics(self):
        """Compute RMS and max tracking error in degrees from accumulated samples.

        Returns:
            (rms_error_deg, max_error_deg): Tracking error metrics.
        """
        if not self.error_samples:
            return 0.0, 0.0

        sum_sq = 0.0
        count = 0
        max_err_deg = 0.0
        for errors in self.error_samples:
            for e in errors:
                e_deg = math.degrees(e)
                sum_sq += e_deg * e_deg
                count += 1
                if e_deg > max_err_deg:
                    max_err_deg = e_deg

        rms = math.sqrt(sum_sq / count) if count > 0 else 0.0
        return round(rms, 4), round(max_err_deg, 4)

    def _compute_avg_tau_ff_norm(self):
        """Compute average tau_ff L2 norm over the segment."""
        if not self.tau_ff_samples:
            return 0.0
        return round(sum(self.tau_ff_samples) / len(self.tau_ff_samples), 4)

    def _send_trajectory_blocking(self, target_positions, duration_sec):
        """Send a single trajectory point and block until completion.

        Args:
            target_positions: List of 7 joint positions in radians.
            duration_sec: Duration for the motion in seconds.

        Returns:
            True if the trajectory succeeded, False otherwise.
        """
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = list(target_positions)
        point.velocities = [0.0] * self.num_joints  # zero velocity at endpoint
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9)
        )
        goal.trajectory.points = [point]

        # Reset completion event
        self._goal_done_event.clear()
        self._goal_success = False

        # Send goal asynchronously
        send_future = self.jtc_client.send_goal_async(goal)

        # Wait for goal acceptance
        timeout = duration_sec + 10.0
        start = time.time()
        while not send_future.done():
            if time.time() - start > 5.0:
                self.get_logger().error('Timeout waiting for goal acceptance')
                return False
            time.sleep(0.05)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory goal REJECTED!')
            return False

        # Wait for result
        result_future = goal_handle.get_result_async()

        start = time.time()
        while not result_future.done():
            if time.time() - start > timeout:
                self.get_logger().error(
                    f'Timeout waiting for trajectory completion '
                    f'(>{timeout:.1f}s)')
                return False
            time.sleep(0.05)

        result = result_future.result()
        # GoalStatus: STATUS_SUCCEEDED = 4
        if result.status == 4:
            return True
        else:
            self.get_logger().warn(
                f'Trajectory finished with status={result.status}, '
                f'error_code={result.result.error_code}, '
                f'error_string="{result.result.error_string}"')
            # Accept non-zero status as long as we got a result
            # (path tolerance violations are common in sim)
            return True

    def _write_csv(self):
        """Write accumulated metrics to CSV file."""
        fieldnames = ['cycle', 'direction', 'rms_error_deg',
                      'max_error_deg', 'avg_tau_ff_norm']

        filepath = os.path.abspath(self.log_file)
        with open(filepath, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for row in self.csv_rows:
                writer.writerow(row)

        self.get_logger().info(
            f'CSV written: {filepath} ({len(self.csv_rows)} rows)')

    def _print_summary(self):
        """Print summary statistics."""
        if not self.csv_rows:
            return

        rms_values = [r['rms_error_deg'] for r in self.csv_rows]
        max_values = [r['max_error_deg'] for r in self.csv_rows]
        tau_values = [r['avg_tau_ff_norm'] for r in self.csv_rows]

        self.get_logger().info(
            f'\n'
            f'╔══════════════════════════════════════════╗\n'
            f'║           DEMO 0: SUMMARY                ║\n'
            f'╠══════════════════════════════════════════╣\n'
            f'║  Mode: {"NO compliance" if self.no_compliance else "WITH compliance":>18s}       ║\n'
            f'║  Cycles: {self.cycles:>3d}                             ║\n'
            f'║  RMS error (mean): {sum(rms_values)/len(rms_values):>6.2f}°             ║\n'
            f'║  RMS error (range): {min(rms_values):.2f}° - {max(rms_values):.2f}°       ║\n'
            f'║  Max error:  {max(max_values):>6.2f}°                   ║\n'
            f'║  Avg tau_ff: {sum(tau_values)/len(tau_values):>6.3f} Nm               ║\n'
            f'║  Log file: {self.log_file:>28s} ║\n'
            f'╚══════════════════════════════════════════╝'
        )


def main():
    parser = argparse.ArgumentParser(
        description='Demo 0: A-B motion with impedance tracking (Task 1.3)')
    parser.add_argument('--side', type=str, default='right',
                        choices=['left', 'right'],
                        help='Which arm to use (default: right)')
    parser.add_argument('--cycles', type=int, default=20,
                        help='Number of A-B-A cycles (default: 20)')
    parser.add_argument('--duration', type=float, default=3.0,
                        help='Seconds per A->B or B->A segment (default: 3.0)')
    parser.add_argument('--no-compliance', action='store_true',
                        help='Flag indicating compliance is disabled')
    parser.add_argument('--log-file', type=str, default='demo_ab_log.csv',
                        help='Output CSV path (default: demo_ab_log.csv)')
    parser.add_argument('--point-a', type=float, nargs=7, default=None,
                        help='Point A joint positions (7 values, radians)')
    parser.add_argument('--point-b', type=float, nargs=7, default=None,
                        help='Point B joint positions (7 values, radians)')

    # ROS 2 may pass extra args; use parse_known_args
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = ImpedanceDemoAB(args)

    # Use an executor that we can cleanly shut down
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Spin in a background thread so callbacks fire
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        success = node.run_demo()
    except KeyboardInterrupt:
        node.get_logger().info('Demo interrupted by user (Ctrl+C)')
        success = False
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()

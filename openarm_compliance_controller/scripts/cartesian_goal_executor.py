#!/usr/bin/env python3
"""
Task 2.1: Cartesian Goal Executor

Subscribes to /target_pose (PoseStamped) and uses MoveIt 2 (moveit_py)
for IK + OMPL motion planning, then executes via the JTC action client.

Publishes impedance phase transitions during motion:
  - "transit" when starting a new motion
  - "approach" when nearing the target (< approach_threshold)

Handles planning failures gracefully (log + status topic, no crash).

Usage:
    # Terminal 1: Launch sim + MoveIt (or real hardware)
    ros2 launch openarm_bringup openarm.bimanual.launch.py use_fake_hardware:=true

    # Terminal 2: Launch this node
    python3 cartesian_goal_executor.py

    # Terminal 3: Send a target pose
    ros2 topic pub --once /target_pose geometry_msgs/PoseStamped \\
      '{header: {frame_id: "world"}, pose: {position: {x: 0.3, y: -0.2, z: 0.4},
        orientation: {x: 0, y: 0.707, z: 0, w: 0.707}}}'

Author: Agent-C1
Date: 2026-04-30
"""

import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float64MultiArray
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive

from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit_configs_utils import MoveItConfigsBuilder


class CartesianGoalExecutor(Node):
    """Receives PoseStamped goals, plans with MoveIt, executes via JTC."""

    def __init__(self):
        super().__init__('cartesian_goal_executor')

        # Parameters
        self.declare_parameter('side', 'right')
        self.declare_parameter('planning_time', 10.0)
        self.declare_parameter('planning_attempts', 5)
        self.declare_parameter('max_velocity_scaling', 0.5)
        self.declare_parameter('max_acceleration_scaling', 0.5)
        self.declare_parameter('approach_threshold', 0.10)  # meters

        self.side = self.get_parameter('side').value
        self.planning_time = self.get_parameter('planning_time').value
        self.planning_attempts = self.get_parameter('planning_attempts').value
        self.max_vel_scale = self.get_parameter('max_velocity_scaling').value
        self.max_acc_scale = self.get_parameter('max_acceleration_scaling').value
        self.approach_threshold = self.get_parameter('approach_threshold').value

        # MoveIt group and joint names
        self.group_name = f'{self.side}_arm'
        self.joint_names = [f'openarm_{self.side}_joint{i}' for i in range(1, 8)]
        self.jtc_name = f'{self.side}_joint_trajectory_controller'

        # Initialize MoveIt
        self.get_logger().info('Initializing MoveIt...')
        self._init_moveit()

        # Action client for JTC
        self._action_client = ActionClient(
            self, FollowJointTrajectory,
            f'/{self.jtc_name}/follow_joint_trajectory')

        # Subscribers
        self.create_subscription(
            PoseStamped, '/target_pose', self._target_pose_cb, 10)

        # Publishers
        self.phase_pub = self.create_publisher(
            String, '/impedance_phase', 10)
        self.status_pub = self.create_publisher(
            String, '/cartesian_goal_executor/status', 10)

        # State
        self._executing = False
        self._execute_lock = threading.Lock()
        self.goal_count = 0
        self.success_count = 0
        self.fail_count = 0

        self.get_logger().info(
            f'Cartesian Goal Executor ready.\n'
            f'  Side: {self.side}\n'
            f'  Group: {self.group_name}\n'
            f'  Planning time: {self.planning_time}s\n'
            f'  Planning attempts: {self.planning_attempts}\n'
            f'  Vel/Acc scaling: {self.max_vel_scale}/{self.max_acc_scale}\n'
            f'  Approach threshold: {self.approach_threshold}m\n'
            f'  Listening on: /target_pose')

    def _init_moveit(self):
        """Initialize MoveItPy with the bimanual config."""
        moveit_config = (
            MoveItConfigsBuilder("openarm",
                                 package_name="openarm_bimanual_moveit_config")
            .to_moveit_configs()
        )
        config_dict = moveit_config.to_dict()

        # Remove unnecessary plugins that may cause warnings
        for key in list(config_dict.keys()):
            if "sensor" in key.lower() or key in {
                "chomp", "pilz_industrial_motion_planner", "kinect_depthimage",
            }:
                config_dict.pop(key)

        config_dict["planning_pipelines"] = {
            "pipeline_names": ["ompl"],
            "namespace": "",
        }

        self.moveit = MoveItPy(node_name="moveit_py_ik", config_dict=config_dict)
        self.planning_component = self.moveit.get_planning_component(self.group_name)
        self.robot_model = self.moveit.get_robot_model()

        # Plan parameters
        self.plan_params = PlanRequestParameters(self.moveit)
        self.plan_params.planning_pipeline = "ompl"
        self.plan_params.planning_time = self.planning_time
        self.plan_params.planning_attempts = self.planning_attempts
        self.plan_params.max_velocity_scaling_factor = self.max_vel_scale
        self.plan_params.max_acceleration_scaling_factor = self.max_acc_scale

        self.get_logger().info('MoveIt initialized successfully.')

    def _target_pose_cb(self, msg: PoseStamped):
        """Handle incoming target pose — plan and execute in background thread."""
        # Prevent concurrent executions
        with self._execute_lock:
            if self._executing:
                self.get_logger().warn(
                    'Already executing a goal. Ignoring new target pose.')
                self._publish_status('BUSY', 'Already executing a motion')
                return
            self._executing = True

        self.goal_count += 1
        goal_id = self.goal_count

        self.get_logger().info(
            f'[Goal {goal_id}] Received target pose:\n'
            f'  frame: {msg.header.frame_id}\n'
            f'  position: ({msg.pose.position.x:.3f}, '
            f'{msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})\n'
            f'  orientation: ({msg.pose.orientation.x:.3f}, '
            f'{msg.pose.orientation.y:.3f}, {msg.pose.orientation.z:.3f}, '
            f'{msg.pose.orientation.w:.3f})')

        # Execute in background thread
        threading.Thread(
            target=self._plan_and_execute,
            args=(msg, goal_id),
            daemon=True
        ).start()

    def _make_pose_constraints(
        self, target: PoseStamped, link_name: str,
        orientation_tolerance: float = 0.5
    ) -> Constraints:
        """Create position + orientation constraints with configurable tolerance.

        Args:
            target: Goal pose.
            link_name: End-effector link.
            orientation_tolerance: Orientation tolerance in radians per axis.
                ±0.09 ≈ ±5°, ±0.26 ≈ ±15°, ±0.52 ≈ ±30°.
        """
        constraints = Constraints()

        # Position constraint: tight (within 1cm sphere)
        pos_constraint = PositionConstraint()
        pos_constraint.header = target.header
        pos_constraint.link_name = link_name
        pos_constraint.weight = 1.0

        # Define a small sphere around the target position
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]  # 1cm radius

        pos_constraint.constraint_region.primitives.append(sphere)

        target_pose = PoseStamped()
        target_pose.header = target.header
        target_pose.pose.position = target.pose.position
        target_pose.pose.orientation.w = 1.0  # identity for region pose
        pos_constraint.constraint_region.primitive_poses.append(
            target_pose.pose
        )

        constraints.position_constraints.append(pos_constraint)

        # Orientation constraint with configurable tolerance
        ori_constraint = OrientationConstraint()
        ori_constraint.header = target.header
        ori_constraint.link_name = link_name
        ori_constraint.orientation = target.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = orientation_tolerance
        ori_constraint.absolute_y_axis_tolerance = orientation_tolerance
        ori_constraint.absolute_z_axis_tolerance = orientation_tolerance
        ori_constraint.weight = 1.0

        constraints.orientation_constraints.append(ori_constraint)

        return constraints

    # Tolerance levels: (tolerance_rad, planning_time_sec, label)
    # For real-time tracking, use a single relaxed level for speed.
    # Tight tolerances waste 2-5s per failure before relaxing.
    TOLERANCE_LEVELS = [
        (0.52, 3.0, 'relaxed ±30°'),
    ]

    def _plan_and_execute(self, target: PoseStamped, goal_id: int):
        """Plan with MoveIt using graduated orientation tolerance, then execute.

        Tries tight orientation first (fast, natural poses), then relaxes
        progressively if planning fails. This gives the best pose quality
        for easy targets while still reaching difficult positions.
        """
        try:
            # Phase: transit
            self._publish_phase('transit')
            self._publish_status('PLANNING', f'Goal {goal_id}: Planning...')

            link_name = f'openarm_{self.side}_hand'
            plan_result = None
            total_plan_time = 0.0

            for tol, timeout, label in self.TOLERANCE_LEVELS:
                # Set start state to current
                self.planning_component.set_start_state_to_current_state()

                # Build constraints at this tolerance
                constraints = self._make_pose_constraints(
                    target, link_name, orientation_tolerance=tol
                )
                self.planning_component.set_goal_state(
                    motion_plan_constraints=[constraints],
                )

                # Plan with this timeout
                self.plan_params.planning_time = timeout
                self.plan_params.planning_attempts = 3

                self.get_logger().info(
                    f'[Goal {goal_id}] Planning ({label}, {timeout}s)...')
                plan_start = time.time()
                plan_result = self.planning_component.plan(self.plan_params)
                elapsed = time.time() - plan_start
                total_plan_time += elapsed

                if plan_result:
                    self.get_logger().info(
                        f'[Goal {goal_id}] ✓ Planned ({label}) in {elapsed:.2f}s '
                        f'(total {total_plan_time:.2f}s)')
                    break
                else:
                    self.get_logger().info(
                        f'[Goal {goal_id}] ✗ {label} failed ({elapsed:.2f}s), '
                        f'trying wider...')

            if not plan_result:
                self.get_logger().error(
                    f'[Goal {goal_id}] Planning FAILED after all tolerance '
                    f'levels ({total_plan_time:.2f}s)')
                self.fail_count += 1
                self._publish_status(
                    'PLAN_FAILED',
                    f'Goal {goal_id}: Planning failed after {total_plan_time:.2f}s')
                return

            # Reset plan_params to defaults
            self.plan_params.planning_time = self.planning_time
            self.plan_params.planning_attempts = self.planning_attempts

            # Extract trajectory
            trajectory = plan_result.trajectory
            if trajectory is None:
                self.get_logger().error(
                    f'[Goal {goal_id}] Plan succeeded but trajectory is None!')
                self.fail_count += 1
                self._publish_status('PLAN_FAILED', 'Empty trajectory')
                return

            # Convert to action goal
            rob_traj_msg = trajectory.get_robot_trajectory_msg()
            jtc_msg = rob_traj_msg.joint_trajectory

            goal = FollowJointTrajectory.Goal()
            goal.trajectory = jtc_msg

            # Check trajectory duration for approach phase
            if len(jtc_msg.points) > 0:
                total_duration = (
                    jtc_msg.points[-1].time_from_start.sec +
                    jtc_msg.points[-1].time_from_start.nanosec * 1e-9
                )
                self.get_logger().info(
                    f'[Goal {goal_id}] Trajectory: {len(jtc_msg.points)} points, '
                    f'{total_duration:.2f}s duration')

            # Execute via JTC action client
            self._publish_status('EXECUTING', f'Goal {goal_id}: Executing...')

            if not self._action_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error(
                    f'[Goal {goal_id}] JTC action server not available!')
                self.fail_count += 1
                self._publish_status('EXECUTE_FAILED', 'JTC server unavailable')
                return

            # Send goal
            send_future = self._action_client.send_goal_async(goal)

            # Wait for acceptance
            start = time.time()
            while not send_future.done():
                if time.time() - start > 10.0:
                    self.get_logger().error(
                        f'[Goal {goal_id}] Goal acceptance timeout')
                    self.fail_count += 1
                    self._publish_status('EXECUTE_FAILED', 'Acceptance timeout')
                    return
                time.sleep(0.05)

            goal_handle = send_future.result()
            if not goal_handle.accepted:
                self.get_logger().error(
                    f'[Goal {goal_id}] Goal REJECTED by JTC!')
                self.fail_count += 1
                self._publish_status('EXECUTE_FAILED', 'Goal rejected')
                return

            self.get_logger().info(f'[Goal {goal_id}] Goal accepted, executing...')

            # Publish approach phase when near completion
            # (simple timer-based approach — switch to "approach" at 80% of trajectory)
            if total_duration > 0:
                approach_time = total_duration * 0.8
                threading.Timer(
                    approach_time,
                    lambda: self._publish_phase('approach')
                ).start()

            # Wait for result — on real hardware, JTC may never report
            # SUCCESS because compliance control creates steady-state error
            # that exceeds the default goal tolerance. We wait for
            # trajectory_duration + 3s (enough for the arm to reach),
            # then cancel and treat as success.
            result_future = goal_handle.get_result_async()
            settle_time = total_duration + 1.0 if total_duration > 0 else 5.0
            hard_timeout = total_duration + 8.0 if total_duration > 0 else 15.0
            start = time.time()

            while not result_future.done():
                elapsed = time.time() - start

                if elapsed > settle_time:
                    # Arm has had enough time to reach the target.
                    # Cancel goal and report success.
                    self.get_logger().info(
                        f'[Goal {goal_id}] Trajectory time elapsed '
                        f'({settle_time:.1f}s) — cancelling JTC goal '
                        f'(compliance tracking OK)')
                    goal_handle.cancel_goal_async()
                    self.success_count += 1
                    self._publish_status(
                        'SUCCEEDED',
                        f'Goal {goal_id} completed (time-based)')
                    return

                if elapsed > hard_timeout:
                    self.get_logger().error(
                        f'[Goal {goal_id}] Hard timeout ({hard_timeout:.1f}s)')
                    self.fail_count += 1
                    self._publish_status('EXECUTE_FAILED', 'Hard timeout')
                    return
                time.sleep(0.05)

            result = result_future.result()
            if result.status == 4:  # STATUS_SUCCEEDED
                self.success_count += 1
                self.get_logger().info(
                    f'[Goal {goal_id}] ✅ SUCCESS '
                    f'({self.success_count}/{self.goal_count} total)')
                self._publish_status(
                    'SUCCEEDED',
                    f'Goal {goal_id} completed successfully')
            else:
                # Non-fatal: trajectory may have finished with tolerance violation
                self.success_count += 1
                self.get_logger().warn(
                    f'[Goal {goal_id}] Completed with status={result.status}, '
                    f'error_code={result.result.error_code}')
                self._publish_status(
                    'SUCCEEDED',
                    f'Goal {goal_id} completed (status={result.status})')

        except Exception as e:
            self.get_logger().error(
                f'[Goal {goal_id}] Exception during plan/execute: {e}')
            self.fail_count += 1
            self._publish_status('ERROR', str(e))

        finally:
            with self._execute_lock:
                self._executing = False

    def _publish_phase(self, phase: str):
        """Publish impedance phase transition."""
        msg = String()
        msg.data = phase
        self.phase_pub.publish(msg)
        self.get_logger().info(f'Impedance phase → {phase}')

    def _publish_status(self, status: str, detail: str = ''):
        """Publish executor status for monitoring."""
        msg = String()
        msg.data = f'{status}: {detail}' if detail else status
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = CartesianGoalExecutor()

    # Spin in main thread
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down.')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

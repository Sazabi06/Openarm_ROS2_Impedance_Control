#!/usr/bin/env python3
# Copyright 2026 OpenArm Contributors
# SPDX-License-Identifier: Apache-2.0
"""
Shadow-mode torque observer for OpenArm (bimanual-aware).

This node subscribes to /joint_states, computes gravity + friction + coriolis
feedforward torques using models ported from openarm_teleop, and publishes
them alongside the actual motor efforts for visual comparison in rqt_plot.
It does NOT send any commands to hardware.

Additionally, it separates the MIT controller's PD torque contribution from
the motor's reported effort, giving a "pure load torque" that can be directly
compared against the computed feedforward model.

Supports both single-arm and bimanual configurations via the
`arm_prefix` parameter (e.g. "" for single, "right_" or "left_").

The goal is to validate the feedforward model accuracy BEFORE
enabling variable impedance (low Kp) control.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from control_msgs.msg import JointTrajectoryControllerState

import PyKDL


# ---------------------------------------------------------------------------
# Minimal KDL tree builder from URDF string using urdf_parser_py + PyKDL
# This replaces the missing kdl_parser_py ROS package.
# ---------------------------------------------------------------------------
def _urdf_pose_to_kdl_frame(pose):
    """Convert a urdf_parser_py Pose to a KDL Frame."""
    if pose is None:
        return PyKDL.Frame()
    pos = pose.position
    rot = pose.rotation
    return PyKDL.Frame(
        PyKDL.Rotation.RPY(rot[0], rot[1], rot[2]),
        PyKDL.Vector(pos[0], pos[1], pos[2]),
    )


def _urdf_inertial_to_kdl(inertial):
    """Convert a urdf_parser_py Inertial to KDL RigidBodyInertia."""
    if inertial is None:
        return PyKDL.RigidBodyInertia()
    origin = inertial.origin
    if origin is not None:
        pos = origin.position
        rot = origin.rotation
        frame = PyKDL.Frame(
            PyKDL.Rotation.RPY(rot[0], rot[1], rot[2]),
            PyKDL.Vector(pos[0], pos[1], pos[2]),
        )
    else:
        frame = PyKDL.Frame()
    mass = inertial.mass if inertial.mass is not None else 0.0
    inertia = inertial.inertia
    if inertia is not None:
        rot_inertia = PyKDL.RotationalInertia(
            inertia.ixx, inertia.iyy, inertia.izz,
            inertia.ixy, inertia.ixz, inertia.iyz,
        )
    else:
        rot_inertia = PyKDL.RotationalInertia()
    return frame.M * PyKDL.RigidBodyInertia(
        mass, frame.p, rot_inertia
    )


def _urdf_joint_to_kdl_joint(joint):
    """Convert a urdf_parser_py Joint to a KDL Joint."""
    jtype = joint.type
    if jtype in ("revolute", "continuous"):
        axis = joint.axis if joint.axis is not None else [1, 0, 0]
        return PyKDL.Joint(
            joint.name,
            PyKDL.Vector(0, 0, 0),
            PyKDL.Vector(axis[0], axis[1], axis[2]),
            PyKDL.Joint.RotAxis,
        )
    elif jtype == "prismatic":
        axis = joint.axis if joint.axis is not None else [1, 0, 0]
        return PyKDL.Joint(
            joint.name,
            PyKDL.Vector(0, 0, 0),
            PyKDL.Vector(axis[0], axis[1], axis[2]),
            PyKDL.Joint.TransAxis,
        )
    else:
        return PyKDL.Joint(joint.name, PyKDL.Joint.Fixed)


def kdl_tree_from_urdf_string(urdf_string):
    """Build a KDL Tree from a URDF XML string.

    Returns (ok: bool, tree: PyKDL.Tree).
    """
    from urdf_parser_py.urdf import URDF
    robot = URDF.from_xml_string(urdf_string)
    tree = PyKDL.Tree(robot.get_root())

    def _add_children(parent_link_name):
        for joint in robot.joints:
            if joint.parent != parent_link_name:
                continue
            child_link_name = joint.child
            child_link = robot.link_map.get(child_link_name)

            kdl_joint = _urdf_joint_to_kdl_joint(joint)
            kdl_frame = _urdf_pose_to_kdl_frame(joint.origin)
            kdl_inertia = _urdf_inertial_to_kdl(
                child_link.inertial if child_link else None
            )
            kdl_segment = PyKDL.Segment(
                child_link_name, kdl_joint, kdl_frame, kdl_inertia
            )
            tree.addSegment(kdl_segment, parent_link_name)
            _add_children(child_link_name)

    _add_children(robot.get_root())
    return True, tree


# ---------------------------------------------------------------------------
# Torque Observer Node
# ---------------------------------------------------------------------------
class TorqueObserverNode(Node):
    """
    Observes joint states and computes gravity + friction + coriolis
    feedforward torques. Publishes both computed and actual torques for
    visual comparison.

    Also computes the MIT controller's PD torque contribution and subtracts
    it from the reported motor effort to produce a "pure load torque" for
    accurate model comparison.

    Supports bimanual via the `arm_prefix` parameter:
      - "" (empty):   single arm, joints = openarm_joint1..7
      - "right_":     bimanual right, joints = openarm_right_joint1..7
      - "left_":      bimanual left,  joints = openarm_left_joint1..7
    """

    ARM_DOF = 7

    def __init__(self):
        super().__init__("torque_observer")

        # ---- Declare parameters ----
        self.declare_parameter("arm_prefix", "right_")
        self.declare_parameter("friction.Fc", [0.0] * self.ARM_DOF)
        self.declare_parameter("friction.k", [0.0] * self.ARM_DOF)
        self.declare_parameter("friction.Fv", [0.0] * self.ARM_DOF)
        self.declare_parameter("friction.Fo", [0.0] * self.ARM_DOF)
        self.declare_parameter("root_link", "openarm_body_link0")
        self.declare_parameter("tip_link", "")  # auto-derived from arm_prefix
        self.declare_parameter("publish_rate", 50.0)

        # MIT controller gains for controller torque separation
        self.declare_parameter(
            "control_gains.kp",
            [70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0],
        )
        self.declare_parameter(
            "control_gains.kd",
            [2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5],
        )

        # ---- Load parameters ----
        self.arm_prefix = self.get_parameter("arm_prefix").value
        self.Fc = list(self.get_parameter("friction.Fc").value)
        self.k = list(self.get_parameter("friction.k").value)
        self.Fv = list(self.get_parameter("friction.Fv").value)
        self.Fo = list(self.get_parameter("friction.Fo").value)
        self.root_link = self.get_parameter("root_link").value
        tip_link_param = self.get_parameter("tip_link").value
        publish_rate = self.get_parameter("publish_rate").value
        self.kp = list(self.get_parameter("control_gains.kp").value)
        self.kd = list(self.get_parameter("control_gains.kd").value)

        # ---- Derive joint names and chain endpoints from arm_prefix ----
        if self.arm_prefix:
            # Bimanual mode: openarm_{prefix}joint1..7
            self.joint_names = [
                f"openarm_{self.arm_prefix}joint{i}" for i in range(1, 8)
            ]
            # Default tip: openarm_{prefix without trailing _}hand → openarm_right_hand
            prefix_clean = self.arm_prefix.rstrip("_")
            self.tip_link = tip_link_param or f"openarm_{prefix_clean}_hand"
            # Default root for bimanual
            if not self.root_link or self.root_link == "openarm_link0":
                self.root_link = "openarm_body_link0"
        else:
            # Single arm mode: openarm_joint1..7
            self.joint_names = [
                f"openarm_joint{i}" for i in range(1, 8)
            ]
            self.tip_link = tip_link_param or "openarm_hand"
            if not self.root_link:
                self.root_link = "openarm_link0"

        side = self.arm_prefix.rstrip("_") if self.arm_prefix else "single"
        self.get_logger().info(
            f"Mode: {side} arm | Joints: {self.joint_names[0]}..{self.joint_names[-1]}"
        )
        self.get_logger().info(
            f"KDL chain: {self.root_link} → {self.tip_link}"
        )
        self.get_logger().info(f"Friction Fc: {self.Fc}")
        self.get_logger().info(f"Control gains Kp: {self.kp}")
        self.get_logger().info(f"Control gains Kd: {self.kd}")
        if all(v == 0.0 for v in self.Fc):
            self.get_logger().warn(
                "All friction.Fc params are 0.0! Check that the YAML "
                "namespace matches the node name, or pass params directly."
            )

        # ---- Publishers (topic names include the arm side) ----
        ns = f"/torque_observer/{side}"
        self.pub_computed = self.create_publisher(
            Float64MultiArray, f"{ns}/computed_tau_ff", 10
        )
        self.pub_actual = self.create_publisher(
            Float64MultiArray, f"{ns}/actual_effort", 10
        )
        self.pub_gravity = self.create_publisher(
            Float64MultiArray, f"{ns}/gravity", 10
        )
        self.pub_friction = self.create_publisher(
            Float64MultiArray, f"{ns}/friction", 10
        )
        self.pub_coriolis = self.create_publisher(
            Float64MultiArray, f"{ns}/coriolis", 10
        )
        # New: controller torque separation
        self.pub_controller_torque = self.create_publisher(
            Float64MultiArray, f"{ns}/controller_torque", 10
        )
        self.pub_load_torque = self.create_publisher(
            Float64MultiArray, f"{ns}/load_torque", 10
        )

        # ---- State ----
        self.positions = [0.0] * self.ARM_DOF
        self.velocities = [0.0] * self.ARM_DOF
        self.efforts = [0.0] * self.ARM_DOF
        self.joint_state_received = False

        # Position commands from the trajectory controller (for controller torque calc)
        self.pos_commands = [0.0] * self.ARM_DOF
        self.vel_commands = [0.0] * self.ARM_DOF
        self.controller_state_received = False

        # ---- KDL dynamics solver (deferred until we get robot_description) ----
        self.kdl_solver = None
        self.kdl_chain = None

        # ---- Subscribe to robot_description to build KDL chain ----
        # robot_state_publisher uses TRANSIENT_LOCAL, so we must match it
        desc_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, "/robot_description",
            self._robot_description_cb, desc_qos
        )

        # ---- Subscribe to joint_states ----
        self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 10
        )

        # ---- Subscribe to trajectory controller state for position commands ----
        # Derive the controller state topic from arm_prefix:
        #   bimanual right → /right_joint_trajectory_controller/state
        #   bimanual left  → /left_joint_trajectory_controller/state
        #   single arm     → /joint_trajectory_controller/state
        if self.arm_prefix:
            ctrl_prefix = self.arm_prefix.rstrip("_")
            ctrl_state_topic = f"/{ctrl_prefix}_joint_trajectory_controller/controller_state"
        else:
            ctrl_state_topic = "/joint_trajectory_controller/controller_state"

        self.create_subscription(
            JointTrajectoryControllerState,
            ctrl_state_topic,
            self._controller_state_cb,
            10,
        )
        self.get_logger().info(
            f"Subscribing to controller state: {ctrl_state_topic}"
        )

        # ---- Timer for periodic computation and publishing ----
        period = 1.0 / publish_rate
        self.timer = self.create_timer(period, self._timer_cb)

        self.get_logger().info(
            "TorqueObserver started (shadow mode — read-only, no commands sent)"
        )

    # ------------------------------------------------------------------
    def _robot_description_cb(self, msg):
        """Build KDL chain from the robot_description URDF string."""
        if self.kdl_solver is not None:
            return  # already initialized

        urdf_string = msg.data
        if not urdf_string:
            self.get_logger().warn("Received empty robot_description")
            return

        ok, tree = kdl_tree_from_urdf_string(urdf_string)
        if not ok:
            self.get_logger().error("Failed to build KDL tree from URDF")
            return

        chain = tree.getChain(self.root_link, self.tip_link)
        if chain.getNrOfJoints() == 0:
            self.get_logger().error(
                f"KDL chain {self.root_link} → {self.tip_link} has 0 joints! "
                f"Check root_link / tip_link parameters."
            )
            return

        self.kdl_chain = chain
        gravity_vector = PyKDL.Vector(0.0, 0.0, -9.81)
        self.kdl_solver = PyKDL.ChainDynParam(chain, gravity_vector)

        # Log chain mass audit
        total_mass = 0.0
        for i in range(chain.getNrOfSegments()):
            seg = chain.getSegment(i)
            inertia = seg.getInertia()
            m = inertia.getMass()
            total_mass += m
            if m > 0.001:  # skip zero-mass fixed links
                cog = inertia.getCOG()
                self.get_logger().info(
                    f"  Segment '{seg.getName()}': "
                    f"mass={m:.4f} kg, CoG=({cog.x():.4f}, {cog.y():.4f}, {cog.z():.4f})"
                )
        self.get_logger().info(
            f"KDL chain ready: {chain.getNrOfJoints()} joints, "
            f"{chain.getNrOfSegments()} segments, "
            f"total mass={total_mass:.4f} kg "
            f"({self.root_link} → {self.tip_link})"
        )

    # ------------------------------------------------------------------
    def _joint_state_cb(self, msg: JointState):
        """Cache the latest joint states for our arm's joints."""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                if idx < len(msg.position):
                    self.positions[i] = msg.position[idx]
                if idx < len(msg.velocity):
                    self.velocities[i] = msg.velocity[idx]
                if idx < len(msg.effort):
                    self.efforts[i] = msg.effort[idx]
        self.joint_state_received = True

    # ------------------------------------------------------------------
    def _controller_state_cb(self, msg: JointTrajectoryControllerState):
        """Cache the position/velocity commands from the trajectory controller.

        The JointTrajectoryControllerState message has:
          - reference.positions: the desired (commanded) positions
          - reference.velocities: the desired velocities
          - output.positions: the actual output sent to HW

        We use 'reference' (the setpoint) to compute the MIT PD torque:
          tau_ctrl = Kp * (p_cmd - p_actual) + Kd * (v_cmd - v_actual)
        """
        # Map controller joint ordering to our joint ordering
        for i, name in enumerate(self.joint_names):
            if name in msg.joint_names:
                idx = msg.joint_names.index(name)
                if idx < len(msg.reference.positions):
                    self.pos_commands[i] = msg.reference.positions[idx]
                if (msg.reference.velocities and
                        idx < len(msg.reference.velocities)):
                    self.vel_commands[i] = msg.reference.velocities[idx]
                else:
                    self.vel_commands[i] = 0.0
        self.controller_state_received = True

    # ------------------------------------------------------------------
    def _compute_gravity(self) -> list:
        """Compute gravity torques using KDL ChainDynParam."""
        if self.kdl_solver is None:
            return [0.0] * self.ARM_DOF

        nj = self.kdl_chain.getNrOfJoints()
        q = PyKDL.JntArray(nj)
        for i in range(min(nj, self.ARM_DOF)):
            q[i] = self.positions[i]

        gravity_torques = PyKDL.JntArray(nj)
        self.kdl_solver.JntToGravity(q, gravity_torques)

        result = [0.0] * self.ARM_DOF
        for i in range(min(nj, self.ARM_DOF)):
            result[i] = gravity_torques[i]
        return result

    # ------------------------------------------------------------------
    def _compute_coriolis(self) -> list:
        """Compute Coriolis/centrifugal torques using KDL ChainDynParam.

        C(q, qdot) * qdot — the velocity-dependent coupling torques.
        At low speeds these are small, but including them gives a more
        accurate feedforward model for impedance control.
        """
        if self.kdl_solver is None:
            return [0.0] * self.ARM_DOF

        nj = self.kdl_chain.getNrOfJoints()
        q = PyKDL.JntArray(nj)
        qdot = PyKDL.JntArray(nj)
        for i in range(min(nj, self.ARM_DOF)):
            q[i] = self.positions[i]
            qdot[i] = self.velocities[i]

        coriolis_torques = PyKDL.JntArray(nj)
        self.kdl_solver.JntToCoriolis(q, qdot, coriolis_torques)

        result = [0.0] * self.ARM_DOF
        for i in range(min(nj, self.ARM_DOF)):
            result[i] = coriolis_torques[i]
        return result

    # ------------------------------------------------------------------
    def _compute_friction(self) -> list:
        """
        Compute friction torques using the tanh-based model from
        openarm_teleop/control.cpp::ComputeFriction.

        tau_f = amp * Fc * tanh(coef * k * dq) + Fv * dq + Fo

        where amp=1.0 and coef=0.1 (matching the C++ implementation).
        """
        amp = 1.0
        coef = 0.1
        friction = [0.0] * self.ARM_DOF
        for i in range(self.ARM_DOF):
            dq = self.velocities[i]
            friction[i] = (
                amp * self.Fc[i] * math.tanh(coef * self.k[i] * dq)
                + self.Fv[i] * dq
                + self.Fo[i]
            )
        return friction

    # ------------------------------------------------------------------
    def _compute_controller_torque(self) -> list:
        """Compute the MIT PD controller's torque contribution.

        The Damiao motor internally computes:
          tau_motor = Kp * (p_cmd - p_actual) + Kd * (v_cmd - v_actual) + tau_ff

        The motor reports tau_motor as get_torque() → actual_effort.
        We know Kp, Kd, p_actual, v_actual from joint_states, and p_cmd
        from the trajectory controller state topic.

        So: tau_ctrl = Kp * (p_cmd - p_actual) + Kd * (v_cmd - v_actual)
        And: load_torque = actual_effort - tau_ctrl
             (this should match our computed_tau_ff if the model is accurate)

        If we haven't received controller state yet, we assume p_cmd ≈ p_actual
        (small tracking error), which gives tau_ctrl ≈ Kd * (0 - v_actual).
        """
        controller_torque = [0.0] * self.ARM_DOF
        for i in range(self.ARM_DOF):
            if self.controller_state_received:
                pos_error = self.pos_commands[i] - self.positions[i]
                vel_error = self.vel_commands[i] - self.velocities[i]
            else:
                # Fallback: assume p_cmd ≈ p_actual, v_cmd ≈ 0
                pos_error = 0.0
                vel_error = -self.velocities[i]

            controller_torque[i] = (
                self.kp[i] * pos_error + self.kd[i] * vel_error
            )
        return controller_torque

    # ------------------------------------------------------------------
    def _timer_cb(self):
        """
        Periodic callback: compute feedforward torques and publish
        alongside actual motor effort for comparison.
        """
        if not self.joint_state_received:
            return

        gravity = self._compute_gravity()
        friction = self._compute_friction()
        coriolis = self._compute_coriolis()

        # Total feedforward: tau_ff = gravity + friction + coriolis
        # Gravity + friction matches bilateral_step() line 174.
        # Coriolis adds velocity-dependent coupling compensation
        # (small at low speeds, important for impedance control).
        tau_ff = [
            gravity[i] + friction[i] + coriolis[i]
            for i in range(self.ARM_DOF)
        ]

        # Controller torque separation:
        # actual_effort = tau_ctrl + tau_load
        # tau_load = actual_effort - tau_ctrl
        # If model is accurate: tau_load ≈ computed_tau_ff
        controller_torque = self._compute_controller_torque()
        load_torque = [
            self.efforts[i] - controller_torque[i]
            for i in range(self.ARM_DOF)
        ]

        # Publish computed tau_ff
        msg_computed = Float64MultiArray()
        msg_computed.data = tau_ff
        self.pub_computed.publish(msg_computed)

        # Publish actual motor effort (from joint_states)
        msg_actual = Float64MultiArray()
        msg_actual.data = list(self.efforts)
        self.pub_actual.publish(msg_actual)

        # Publish gravity, friction, and coriolis separately for debugging
        msg_grav = Float64MultiArray()
        msg_grav.data = gravity
        self.pub_gravity.publish(msg_grav)

        msg_fric = Float64MultiArray()
        msg_fric.data = friction
        self.pub_friction.publish(msg_fric)

        msg_cor = Float64MultiArray()
        msg_cor.data = coriolis
        self.pub_coriolis.publish(msg_cor)

        # Publish controller torque and load torque
        msg_ctrl = Float64MultiArray()
        msg_ctrl.data = controller_torque
        self.pub_controller_torque.publish(msg_ctrl)

        msg_load = Float64MultiArray()
        msg_load.data = load_torque
        self.pub_load_torque.publish(msg_load)


def main(args=None):
    rclpy.init(args=args)
    node = TorqueObserverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

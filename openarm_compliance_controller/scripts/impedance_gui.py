#!/usr/bin/env python3
# Copyright 2026 OpenArm Contributors
# SPDX-License-Identifier: Apache-2.0
"""
PyQt5 GUI for OpenArm Compliance Controller — Integrated Control Panel.

Features:
  - Controller activate/deactivate toggle
  - Per-joint Kp/Kd sliders with real-time numeric display
  - Live tau_ff readout from the controller
  - E-STOP button: resets to defaults + sends home trajectory
  - Presets: Full Stiff, Soft Wrist, Full Soft, Extra Stiff
  - Joint angle input (degrees) with Run/Home buttons
  - Joint limits read from URDF for safety
  - Log subwindow showing all operations

Usage:
  ros2 run openarm_compliance_controller impedance_gui.py
  ros2 run openarm_compliance_controller impedance_gui.py --side left
"""

import sys
import math
import argparse
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Float64MultiArray, String
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QSlider, QPushButton, QFrame, QGroupBox,
    QSizePolicy, QDoubleSpinBox, QTextEdit, QScrollArea
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont, QColor, QPalette, QTextCursor


# ─── Joint Configuration ─────────────────────────────────────────────
JOINT_CONFIG = [
    # (label, kp_min, kp_max, kp_default, kd_min, kd_max, kd_default)
    ("J1 — Shoulder Yaw",   15.0, 150.0, 70.0,   0.50, 5.0, 2.75),
    ("J2 — Shoulder Pitch",  15.0, 150.0, 70.0,   0.50, 5.0, 2.50),
    ("J3 — Shoulder Roll",   15.0, 150.0, 70.0,   0.40, 5.0, 2.00),
    ("J4 — Elbow",           12.0, 120.0, 60.0,   0.40, 5.0, 2.00),
    ("J5 — Wrist Roll",      3.0,  30.0,  10.0,   0.15, 2.0, 0.70),
    ("J6 — Wrist Yaw",       3.0,  30.0,  10.0,   0.12, 2.0, 0.60),
    ("J7 — Wrist Pitch",     3.0,  30.0,  10.0,   0.10, 2.0, 0.50),
]

# Fallback joint limits (rad) if URDF parsing fails
# These are the raw values from joint_limits.yaml with right_arm reflect=1 and offsets applied
FALLBACK_LIMITS_RAD = [
    (-1.396, 3.491),   # J1: no offset, reflect=1
    (-0.175, 3.316),   # J2: offset=+pi/2, reflect=1 → (-1.745+1.571, 1.745+1.571)
    (-1.571, 1.571),   # J3
    ( 0.000, 2.443),   # J4: reflect=1 (forced)
    (-1.571, 1.571),   # J5
    (-0.785, 0.785),   # J6
    (-1.571, 1.571),   # J7
]

PRESETS = {
    "🔒 Full Stiff (Default)": {
        "kp": [70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0],
        "kd": [2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5],
    },
    "🤝 Soft Wrist": {
        "kp": [70.0, 70.0, 70.0, 20.0, 3.0, 3.0, 3.0],
        "kd": [2.75, 2.5, 2.0, 0.8, 0.15, 0.12, 0.1],
    },
    "🪶 Full Soft (Min)": {
        "kp": [15.0, 15.0, 15.0, 12.0, 3.0, 3.0, 3.0],
        "kd": [0.5, 0.5, 0.4, 0.4, 0.15, 0.12, 0.1],
    },
    "💪 Extra Stiff": {
        "kp": [120.0, 120.0, 120.0, 100.0, 25.0, 25.0, 25.0],
        "kd": [4.0, 4.0, 4.0, 3.5, 1.5, 1.5, 1.5],
    },
}

DARK_STYLE = """
QMainWindow {
    background-color: #1a1d23;
}
QWidget {
    background-color: #1a1d23;
    color: #e0e0e0;
    font-family: 'Inter', 'Segoe UI', 'Roboto', sans-serif;
}
QLabel {
    color: #c8cdd3;
    font-size: 13px;
}
QGroupBox {
    border: 1px solid #2a2f38;
    border-radius: 8px;
    margin-top: 12px;
    padding: 12px 8px 8px 8px;
    background-color: #22262e;
    font-weight: bold;
    font-size: 14px;
    color: #8ab4f8;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 12px;
    padding: 0 6px;
}
QSlider::groove:horizontal {
    border: none;
    height: 6px;
    background: #3a3f4b;
    border-radius: 3px;
}
QSlider::handle:horizontal {
    background: #5b9cf5;
    border: 2px solid #7bb8ff;
    width: 16px;
    height: 16px;
    margin: -6px 0;
    border-radius: 9px;
}
QSlider::handle:horizontal:hover {
    background: #7bb8ff;
    border: 2px solid #a0d0ff;
}
QSlider::sub-page:horizontal {
    background: #3d6faa;
    border-radius: 3px;
}
QPushButton {
    background-color: #2a3040;
    color: #c8cdd3;
    border: 1px solid #3a4050;
    border-radius: 6px;
    padding: 8px 16px;
    font-size: 13px;
    font-weight: 500;
}
QPushButton:hover {
    background-color: #354060;
    border: 1px solid #5b9cf5;
}
QPushButton:pressed {
    background-color: #1a2030;
}
QPushButton:disabled {
    background-color: #1a1d23;
    color: #555;
    border: 1px solid #2a2f38;
}
QFrame#separator {
    background-color: #2a2f38;
    max-height: 1px;
}
QDoubleSpinBox {
    background-color: #22262e;
    color: #e0e0e0;
    border: 1px solid #3a4050;
    border-radius: 4px;
    padding: 4px 8px;
    font-size: 13px;
}
QDoubleSpinBox:disabled {
    background-color: #1a1d23;
    color: #555;
}
QTextEdit {
    background-color: #15171c;
    color: #a0a8b0;
    border: 1px solid #2a2f38;
    border-radius: 6px;
    padding: 6px;
    font-family: 'Consolas', 'Courier New', monospace;
    font-size: 12px;
}
"""

ESTOP_STYLE = """
QPushButton {
    background-color: #c0392b;
    color: white;
    border: 2px solid #e74c3c;
    border-radius: 8px;
    padding: 12px 32px;
    font-size: 18px;
    font-weight: bold;
    letter-spacing: 2px;
}
QPushButton:hover {
    background-color: #e74c3c;
    border: 2px solid #ff6b6b;
}
QPushButton:pressed {
    background-color: #922b21;
}
"""

TOGGLE_ACTIVE_STYLE = """
QPushButton {
    background-color: #1e6b3a;
    color: white;
    border: 2px solid #27ae60;
    border-radius: 6px;
    padding: 8px 20px;
    font-size: 14px;
    font-weight: bold;
}
QPushButton:hover {
    background-color: #27ae60;
}
"""

TOGGLE_INACTIVE_STYLE = """
QPushButton {
    background-color: #4a4a4a;
    color: #ccc;
    border: 2px solid #666;
    border-radius: 6px;
    padding: 8px 20px;
    font-size: 14px;
    font-weight: bold;
}
QPushButton:hover {
    background-color: #5a5a5a;
}
"""

RUN_BTN_STYLE = """
QPushButton {
    background-color: #2563a8;
    color: white;
    border: 2px solid #3a8fd4;
    border-radius: 6px;
    padding: 8px 20px;
    font-size: 14px;
    font-weight: bold;
}
QPushButton:hover {
    background-color: #3a8fd4;
}
QPushButton:disabled {
    background-color: #1a2030;
    color: #555;
    border: 2px solid #2a3040;
}
"""

HOME_BTN_STYLE = """
QPushButton {
    background-color: #7b5c2e;
    color: white;
    border: 2px solid #d4a33a;
    border-radius: 6px;
    padding: 8px 20px;
    font-size: 14px;
    font-weight: bold;
}
QPushButton:hover {
    background-color: #d4a33a;
}
"""


# ─── URDF Joint Limits Parser ────────────────────────────────────────
def parse_joint_limits_from_urdf(urdf_string, side):
    """Parse joint limits from URDF string using urdf_parser_py."""
    try:
        from urdf_parser_py.urdf import URDF
        robot = URDF.from_xml_string(urdf_string)
        limits = []
        for i in range(1, 8):
            joint_name = f"openarm_{side}_joint{i}"
            for j in robot.joints:
                if j.name == joint_name and j.limit is not None:
                    limits.append((j.limit.lower, j.limit.upper))
                    break
            else:
                # Joint not found, use fallback
                limits.append(FALLBACK_LIMITS_RAD[i - 1])
        return limits
    except Exception:
        return FALLBACK_LIMITS_RAD


# ─── ROS 2 Bridge Node ───────────────────────────────────────────────
class ImpedanceGuiNode(Node):
    """ROS 2 node for impedance GUI: pub/sub, services, actions."""

    def __init__(self, side: str):
        super().__init__(f"impedance_gui_{side}")
        self.side = side
        prefix = f"/{side}_compliance_controller"
        self.controller_name = f"{side}_compliance_controller"

        # Impedance params publisher
        self.pub = self.create_publisher(
            Float64MultiArray, f"{prefix}/impedance_params", 10
        )

        # tau_ff subscriber
        self.tau_ff = [0.0] * 7
        self.sub = self.create_subscription(
            Float64MultiArray, f"{prefix}/tau_ff", self._tau_ff_cb, 10
        )

        # Action client for JTC
        jtc_name = f"/{side}_joint_trajectory_controller"
        self.jtc_client = ActionClient(
            self, FollowJointTrajectory,
            f"{jtc_name}/follow_joint_trajectory"
        )

        self.joint_names = [
            f"openarm_{side}_joint{i}" for i in range(1, 8)
        ]

        # Gripper action client
        gripper_name = f"/{side}_gripper_controller"
        self.gripper_client = ActionClient(
            self, GripperCommand,
            f"{gripper_name}/gripper_cmd"
        )
        self.gripper_joint_name = f"openarm_{side}_finger_joint1"
        # Gripper limits: prismatic, 0.0 (closed) to 0.032m (open) — actual stroke
        self.gripper_min = 0.0
        self.gripper_max = 0.032
        self.gripper_position = 0.0  # track current command

        # Gripper stiffness publisher (ForwardCommandController)
        self.grip_stiffness_pub = self.create_publisher(
            Float64MultiArray,
            f"/{side}_gripper_stiffness_controller/commands", 10
        )
        self.grip_damping_pub = self.create_publisher(
            Float64MultiArray,
            f"/{side}_gripper_damping_controller/commands", 10
        )
        self.gripper_kp = 2.0  # current gripper Kp (matches hardware default)
        self.gripper_kd = 0.1  # current gripper Kd

        # Service client for controller_manager switch
        self.switch_client = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )

        # URDF subscription to read joint limits
        self.joint_limits_rad = list(FALLBACK_LIMITS_RAD)  # copy
        self.urdf_received = False
        self._urdf_sub = self.create_subscription(
            String, "/robot_description", self._urdf_cb,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        self.controller_active = True  # assume active on startup

    def _tau_ff_cb(self, msg: Float64MultiArray):
        if len(msg.data) == 7:
            self.tau_ff = list(msg.data)

    def _urdf_cb(self, msg: String):
        if not self.urdf_received:
            limits = parse_joint_limits_from_urdf(msg.data, self.side)
            if len(limits) == 7:
                self.joint_limits_rad = limits
                self.urdf_received = True
                self.get_logger().info(
                    f"Joint limits from URDF: {[(f'{math.degrees(lo):.1f}°', f'{math.degrees(hi):.1f}°') for lo,hi in limits]}"
                )

    def publish_impedance(self, kp_values: list, kd_values: list):
        msg = Float64MultiArray()
        msg.data = kp_values + kd_values
        self.pub.publish(msg)

    def send_target(self, positions_rad: list, duration_sec: float):
        """Send arm to target position via JTC."""
        if not self.jtc_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("JTC action server not available!")
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions = positions_rad
        pt.velocities = [0.0] * 7
        pt.time_from_start = Duration(sec=int(duration_sec),
                                       nanosec=int((duration_sec % 1) * 1e9))
        goal.trajectory.points = [pt]
        self.jtc_client.send_goal_async(goal)
        return True

    def send_gripper(self, position: float):
        """Send gripper to target position (0.0=closed, 0.032=open)."""
        position = max(self.gripper_min, min(self.gripper_max, position))
        if not self.gripper_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Gripper action server not available!")
            return False
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 10.0  # high limit, actual force controlled by MIT Kp
        self.gripper_client.send_goal_async(goal)
        self.gripper_position = position
        return True

    def set_gripper_stiffness(self, kp: float):
        """Set gripper motor stiffness via ForwardCommandController."""
        msg = Float64MultiArray()
        msg.data = [kp]
        self.grip_stiffness_pub.publish(msg)
        self.gripper_kp = kp
        self.get_logger().info(f"Gripper Kp set to {kp:.1f}")

    def set_gripper_damping(self, kd: float):
        """Set gripper motor damping via ForwardCommandController."""
        msg = Float64MultiArray()
        msg.data = [kd]
        self.grip_damping_pub.publish(msg)
        self.gripper_kd = kd
        self.get_logger().info(f"Gripper Kd set to {kd:.2f}")

    def set_gripper_impedance(self, kp: float, kd: float):
        """Set both gripper Kp and Kd."""
        self.set_gripper_stiffness(kp)
        self.set_gripper_damping(kd)

    def send_home(self):
        """Send arm to zero position."""
        return self.send_target([0.0] * 7, 3.0)

    def switch_controller(self, activate: bool):
        """Activate or deactivate the compliance controller."""
        if not self.switch_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("controller_manager switch service not available!")
            return False

        req = SwitchController.Request()
        req.strictness = SwitchController.Request.BEST_EFFORT
        if activate:
            req.activate_controllers = [self.controller_name]
            req.deactivate_controllers = []
        else:
            req.activate_controllers = []
            req.deactivate_controllers = [self.controller_name]

        future = self.switch_client.call_async(req)
        # Don't block — the GUI will check the result via timer
        self.controller_active = activate
        return True


# ─── Main Window ─────────────────────────────────────────────────────
class ImpedanceGui(QMainWindow):
    tau_ff_updated = pyqtSignal(list)
    log_signal = pyqtSignal(str)

    def __init__(self, ros_node: ImpedanceGuiNode):
        super().__init__()
        self.node = ros_node
        self.setWindowTitle(
            f"OpenArm Compliance Controller — {ros_node.side.capitalize()}")
        self.setMinimumSize(960, 860)

        # Sliders storage
        self.kp_sliders = []
        self.kd_sliders = []
        self.kp_labels = []
        self.kd_labels = []
        self.tau_labels = []

        # Joint angle spinboxes
        self.joint_spinboxes = []

        self._build_ui()
        self.setStyleSheet(DARK_STYLE)

        # Signals for thread-safe updates
        self.tau_ff_updated.connect(self._update_tau_labels)
        self.log_signal.connect(self._append_log)

        # Periodic tau_ff poll (10 Hz)
        self.tau_timer = QTimer(self)
        self.tau_timer.timeout.connect(self._poll_tau_ff)
        self.tau_timer.start(100)

        # Delayed joint limits update from URDF (check every 2s until received)
        self.limits_timer = QTimer(self)
        self.limits_timer.timeout.connect(self._update_joint_limits)
        self.limits_timer.start(2000)

        self._publish_current()
        self._log("GUI started, waiting for URDF joint limits...")

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setSpacing(6)
        main_layout.setContentsMargins(16, 10, 16, 10)

        # ── Header row: Title + E-STOP ──
        header = QHBoxLayout()
        title = QLabel(
            f"OpenArm Compliance Controller — {self.node.side.capitalize()} Arm")
        title.setFont(QFont("Inter", 18, QFont.Bold))
        title.setStyleSheet("color: #e0e0e0; padding: 4px;")
        header.addWidget(title)
        header.addStretch()

        self.estop_btn = QPushButton("⬛  E-STOP")
        self.estop_btn.setStyleSheet(ESTOP_STYLE)
        self.estop_btn.setFixedSize(200, 50)
        self.estop_btn.clicked.connect(self._on_estop)
        header.addWidget(self.estop_btn)
        main_layout.addLayout(header)

        # ── Separator ──
        main_layout.addWidget(self._make_separator())

        # ── Top panel: Controller toggle + Target position ──
        top_panel = QHBoxLayout()
        top_panel.setSpacing(12)

        # --- Controller toggle ---
        ctrl_group = QGroupBox("Controller")
        ctrl_layout = QVBoxLayout(ctrl_group)
        ctrl_layout.setSpacing(6)

        self.ctrl_status_label = QLabel("Status: 🟢 ACTIVE")
        self.ctrl_status_label.setStyleSheet(
            "color: #27ae60; font-size: 14px; font-weight: bold;")
        ctrl_layout.addWidget(self.ctrl_status_label)

        self.ctrl_toggle_btn = QPushButton("🔄 Deactivate")
        self.ctrl_toggle_btn.setStyleSheet(TOGGLE_ACTIVE_STYLE)
        self.ctrl_toggle_btn.setFixedHeight(40)
        self.ctrl_toggle_btn.clicked.connect(self._on_toggle_controller)
        ctrl_layout.addWidget(self.ctrl_toggle_btn)

        ctrl_group.setFixedWidth(220)
        top_panel.addWidget(ctrl_group)

        # --- Target position ---
        target_group = QGroupBox("Target Joint Angles")
        target_grid = QGridLayout(target_group)
        target_grid.setHorizontalSpacing(4)
        target_grid.setVerticalSpacing(6)

        # Each joint slot uses 3 columns: [label] [range] [spinbox]
        # 4 slots per row = 12 columns
        short_labels = ["J1", "J2", "J3", "J4", "J5", "J6", "J7"]
        for i in range(7):
            slot = i % 4
            row = i // 4
            col_base = slot * 3  # 3 cols per slot

            lbl = QLabel(short_labels[i])
            lbl.setStyleSheet("color: #8ab4f8; font-weight: bold; font-size: 12px;")
            lbl.setFixedWidth(22)
            target_grid.addWidget(lbl, row, col_base)

            lo_rad, hi_rad = FALLBACK_LIMITS_RAD[i]
            lo_deg = math.degrees(lo_rad)
            hi_deg = math.degrees(hi_rad)
            range_lbl = QLabel(f"{lo_deg:.0f}~{hi_deg:.0f}°")
            range_lbl.setStyleSheet("color: #666; font-size: 10px;")
            range_lbl.setFixedWidth(68)
            range_lbl.setAlignment(Qt.AlignCenter)
            target_grid.addWidget(range_lbl, row, col_base + 1)

            spin = QDoubleSpinBox()
            spin.setMinimum(lo_deg)
            spin.setMaximum(hi_deg)
            spin.setValue(0.0)
            spin.setDecimals(1)
            spin.setSingleStep(1.0)
            spin.setSuffix("°")
            spin.setFixedWidth(90)
            target_grid.addWidget(spin, row, col_base + 2)
            self.joint_spinboxes.append(spin)

        # Gripper (8th entry, row 1 slot 3 = cols 9,10,11)
        grip_lbl = QLabel("Grip")
        grip_lbl.setStyleSheet("color: #f5a623; font-weight: bold; font-size: 12px;")
        grip_lbl.setFixedWidth(28)
        target_grid.addWidget(grip_lbl, 1, 9)

        grip_range = QLabel("0~32mm")
        grip_range.setStyleSheet("color: #666; font-size: 10px;")
        grip_range.setFixedWidth(50)
        grip_range.setAlignment(Qt.AlignCenter)
        target_grid.addWidget(grip_range, 1, 10)

        self.gripper_spin = QDoubleSpinBox()
        self.gripper_spin.setMinimum(0.0)
        self.gripper_spin.setMaximum(32.0)
        self.gripper_spin.setValue(0.0)
        self.gripper_spin.setDecimals(1)
        self.gripper_spin.setSingleStep(1.0)
        self.gripper_spin.setSuffix(" mm")
        self.gripper_spin.setFixedWidth(90)
        self.gripper_spin.setStyleSheet(
            "QDoubleSpinBox { color: #f5a623; }"
        )
        target_grid.addWidget(self.gripper_spin, 1, 11)

        # Gripper stiffness toggle: G (Grip/firm) and S (Soft)
        GRIP_G_ON = ("QPushButton { background-color: #8b4513; color: #ffa500; "
                     "border: 2px solid #ffa500; border-radius: 5px; padding: 4px 10px; "
                     "font-weight: bold; font-size: 13px; }")
        GRIP_G_OFF = ("QPushButton { background-color: #2a2520; color: #886644; "
                      "border: 1px solid #554433; border-radius: 5px; padding: 4px 10px; "
                      "font-weight: bold; font-size: 13px; } "
                      "QPushButton:hover { background-color: #3a3530; }")
        GRIP_S_ON = ("QPushButton { background-color: #1a4a6a; color: #5bc0de; "
                     "border: 2px solid #5bc0de; border-radius: 5px; padding: 4px 10px; "
                     "font-weight: bold; font-size: 13px; }")
        GRIP_S_OFF = ("QPushButton { background-color: #1a2530; color: #446688; "
                      "border: 1px solid #334455; border-radius: 5px; padding: 4px 10px; "
                      "font-weight: bold; font-size: 13px; } "
                      "QPushButton:hover { background-color: #2a3540; }")

        self._grip_styles = {"g_on": GRIP_G_ON, "g_off": GRIP_G_OFF,
                             "s_on": GRIP_S_ON, "s_off": GRIP_S_OFF}
        self._grip_mode = "default"  # "default", "grip", "soft"

        self.grip_g_btn = QPushButton("G")
        self.grip_g_btn.setToolTip("Grip (firm)  Kp=5.0")
        self.grip_g_btn.setFixedSize(36, 28)
        self.grip_g_btn.setStyleSheet(GRIP_G_OFF)
        self.grip_g_btn.clicked.connect(self._on_grip_mode_g)
        target_grid.addWidget(self.grip_g_btn, 1, 12)

        self.grip_s_btn = QPushButton("S")
        self.grip_s_btn.setToolTip("Soft (gentle)  Kp=0.5")
        self.grip_s_btn.setFixedSize(36, 28)
        self.grip_s_btn.setStyleSheet(GRIP_S_OFF)
        self.grip_s_btn.clicked.connect(self._on_grip_mode_s)
        target_grid.addWidget(self.grip_s_btn, 1, 13)

        # Gripper debounce timer
        self.gripper_timer = QTimer(self)
        self.gripper_timer.setSingleShot(True)
        self.gripper_timer.setInterval(300)
        self.gripper_timer.timeout.connect(self._send_gripper_command)
        self.gripper_spin.valueChanged.connect(self._on_gripper_spin_changed)

        # Row 2: Time + Run + Home + Open + Close
        dur_lbl = QLabel("Time")
        dur_lbl.setStyleSheet("color: #c8cdd3; font-size: 12px;")
        dur_lbl.setFixedWidth(30)
        target_grid.addWidget(dur_lbl, 2, 0)

        self.duration_spin = QDoubleSpinBox()
        self.duration_spin.setMinimum(0.5)
        self.duration_spin.setMaximum(10.0)
        self.duration_spin.setValue(3.0)
        self.duration_spin.setDecimals(1)
        self.duration_spin.setSingleStep(0.5)
        self.duration_spin.setSuffix("s")
        self.duration_spin.setFixedWidth(90)
        target_grid.addWidget(self.duration_spin, 2, 1, 1, 2)

        # Run button
        self.run_btn = QPushButton("▶ Run")
        self.run_btn.setStyleSheet(RUN_BTN_STYLE)
        self.run_btn.setFixedHeight(36)
        self.run_btn.clicked.connect(self._on_run_trajectory)
        target_grid.addWidget(self.run_btn, 2, 3, 1, 2)

        # Home button
        self.home_btn = QPushButton("🏠 Home")
        self.home_btn.setStyleSheet(HOME_BTN_STYLE)
        self.home_btn.setFixedHeight(36)
        self.home_btn.clicked.connect(self._on_home)
        target_grid.addWidget(self.home_btn, 2, 5, 1, 2)

        # Gripper Open button
        self.grip_open_btn = QPushButton("✋ Open")
        self.grip_open_btn.setStyleSheet(
            "QPushButton { background-color: #2a5040; color: #50c878; "
            "border: 1px solid #50c878; border-radius: 5px; padding: 6px 12px; "
            "font-weight: bold; font-size: 12px; } "
            "QPushButton:hover { background-color: #3a6b55; }")
        self.grip_open_btn.setFixedHeight(36)
        self.grip_open_btn.clicked.connect(lambda: self._set_gripper_value(32.0))
        target_grid.addWidget(self.grip_open_btn, 2, 9, 1, 2)

        # Gripper Close button
        self.grip_close_btn = QPushButton("✊ Close")
        self.grip_close_btn.setStyleSheet(
            "QPushButton { background-color: #503030; color: #e74c3c; "
            "border: 1px solid #e74c3c; border-radius: 5px; padding: 6px 12px; "
            "font-weight: bold; font-size: 12px; } "
            "QPushButton:hover { background-color: #6b3a3a; }")
        self.grip_close_btn.setFixedHeight(36)
        self.grip_close_btn.clicked.connect(lambda: self._set_gripper_value(0.0))
        target_grid.addWidget(self.grip_close_btn, 2, 11)

        top_panel.addWidget(target_group)
        main_layout.addLayout(top_panel)

        # ── Separator ──
        main_layout.addWidget(self._make_separator())

        # ── Joint rows (scrollable) ──
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("QScrollArea { border: none; }")
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setSpacing(2)
        scroll_layout.setContentsMargins(0, 0, 0, 0)

        for i, (label, kp_min, kp_max, kp_def, kd_min, kd_max, kd_def) in enumerate(JOINT_CONFIG):
            group = QGroupBox(label)
            grid = QGridLayout(group)
            grid.setHorizontalSpacing(16)
            grid.setVerticalSpacing(2)

            # Kp slider
            grid.addWidget(QLabel("Kp"), 0, 0)
            kp_slider = QSlider(Qt.Horizontal)
            kp_slider.setMinimum(int(kp_min * 10))
            kp_slider.setMaximum(int(kp_max * 10))
            kp_slider.setValue(int(kp_def * 10))
            kp_slider.setTickInterval(int((kp_max - kp_min) * 10 / 5))
            kp_slider.valueChanged.connect(self._on_slider_changed)
            grid.addWidget(kp_slider, 0, 1)
            self.kp_sliders.append(kp_slider)

            kp_val_lbl = QLabel(f"{kp_def:.1f}")
            kp_val_lbl.setFixedWidth(55)
            kp_val_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            kp_val_lbl.setStyleSheet(
                "color: #5b9cf5; font-weight: bold; font-size: 14px;")
            grid.addWidget(kp_val_lbl, 0, 2)
            self.kp_labels.append(kp_val_lbl)

            kp_range = QLabel(f"[{kp_min:.0f} – {kp_max:.0f}]")
            kp_range.setStyleSheet("color: #666; font-size: 11px;")
            kp_range.setFixedWidth(80)
            grid.addWidget(kp_range, 0, 3)

            # Kd slider
            grid.addWidget(QLabel("Kd"), 1, 0)
            kd_slider = QSlider(Qt.Horizontal)
            kd_slider.setMinimum(int(kd_min * 100))
            kd_slider.setMaximum(int(kd_max * 100))
            kd_slider.setValue(int(kd_def * 100))
            kd_slider.setTickInterval(int((kd_max - kd_min) * 100 / 5))
            kd_slider.valueChanged.connect(self._on_slider_changed)
            grid.addWidget(kd_slider, 1, 1)
            self.kd_sliders.append(kd_slider)

            kd_val_lbl = QLabel(f"{kd_def:.2f}")
            kd_val_lbl.setFixedWidth(55)
            kd_val_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            kd_val_lbl.setStyleSheet(
                "color: #5b9cf5; font-weight: bold; font-size: 14px;")
            grid.addWidget(kd_val_lbl, 1, 2)
            self.kd_labels.append(kd_val_lbl)

            kd_range = QLabel(f"[{kd_min:.2f} – {kd_max:.1f}]")
            kd_range.setStyleSheet("color: #666; font-size: 11px;")
            kd_range.setFixedWidth(80)
            grid.addWidget(kd_range, 1, 3)

            # tau_ff readout
            tau_lbl = QLabel("τ: —")
            tau_lbl.setFixedWidth(100)
            tau_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            tau_lbl.setStyleSheet(
                "color: #50c878; font-size: 13px; font-weight: bold;")
            grid.addWidget(tau_lbl, 0, 4, 2, 1)
            self.tau_labels.append(tau_lbl)

            scroll_layout.addWidget(group)

        # ── Gripper Kp/Kd row (after J7) ──
        grip_group = QGroupBox("🤏 Gripper — DM4310")
        grip_grid = QGridLayout(grip_group)
        grip_grid.setHorizontalSpacing(16)
        grip_grid.setVerticalSpacing(2)

        # Gripper Kp slider: range 0.3 (safety min) to 10.0
        grip_grid.addWidget(QLabel("Kp"), 0, 0)
        self.grip_kp_slider = QSlider(Qt.Horizontal)
        self.grip_kp_slider.setMinimum(3)   # 0.3 * 10
        self.grip_kp_slider.setMaximum(100) # 10.0 * 10
        self.grip_kp_slider.setValue(20)    # 2.0 * 10 (default)
        self.grip_kp_slider.setTickInterval(10)
        self.grip_kp_slider.valueChanged.connect(self._on_grip_slider_changed)
        grip_grid.addWidget(self.grip_kp_slider, 0, 1)

        self.grip_kp_label = QLabel("2.0")
        self.grip_kp_label.setFixedWidth(55)
        self.grip_kp_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.grip_kp_label.setStyleSheet(
            "color: #f5a623; font-weight: bold; font-size: 14px;")
        grip_grid.addWidget(self.grip_kp_label, 0, 2)

        grip_kp_range = QLabel("[0.3 – 10.0]")
        grip_kp_range.setStyleSheet("color: #666; font-size: 11px;")
        grip_kp_range.setFixedWidth(80)
        grip_grid.addWidget(grip_kp_range, 0, 3)

        # Gripper Kd slider: range 0.05 to 1.0
        grip_grid.addWidget(QLabel("Kd"), 1, 0)
        self.grip_kd_slider = QSlider(Qt.Horizontal)
        self.grip_kd_slider.setMinimum(5)    # 0.05 * 100
        self.grip_kd_slider.setMaximum(100)  # 1.0 * 100
        self.grip_kd_slider.setValue(10)     # 0.1 * 100 (default)
        self.grip_kd_slider.setTickInterval(10)
        self.grip_kd_slider.valueChanged.connect(self._on_grip_slider_changed)
        grip_grid.addWidget(self.grip_kd_slider, 1, 1)

        self.grip_kd_label = QLabel("0.10")
        self.grip_kd_label.setFixedWidth(55)
        self.grip_kd_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.grip_kd_label.setStyleSheet(
            "color: #f5a623; font-weight: bold; font-size: 14px;")
        grip_grid.addWidget(self.grip_kd_label, 1, 2)

        grip_kd_range = QLabel("[0.05 – 1.0]")
        grip_kd_range.setStyleSheet("color: #666; font-size: 11px;")
        grip_kd_range.setFixedWidth(80)
        grip_grid.addWidget(grip_kd_range, 1, 3)

        scroll_layout.addWidget(grip_group)

        scroll.setWidget(scroll_content)
        main_layout.addWidget(scroll, stretch=1)


        # ── Presets ──
        preset_layout = QHBoxLayout()
        preset_layout.setSpacing(8)
        for name, values in PRESETS.items():
            btn = QPushButton(name)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            btn.clicked.connect(
                lambda checked, n=name: self._apply_preset(n))
            preset_layout.addWidget(btn)
        main_layout.addLayout(preset_layout)

        # ── Separator ──
        main_layout.addWidget(self._make_separator())

        # ── Log subwindow ──
        log_header = QLabel("📋 Activity Log")
        log_header.setStyleSheet(
            "color: #8ab4f8; font-size: 13px; font-weight: bold; padding: 2px;")
        main_layout.addWidget(log_header)

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFixedHeight(120)
        self.log_text.setPlaceholderText("Activity log will appear here...")
        main_layout.addWidget(self.log_text)

        # ── Status bar ──
        self.status_label = QLabel(
            f"  🟢 Connected to /{self.node.side}_compliance_controller  |  "
            f"Publishing ~/impedance_params  |  Monitoring ~/tau_ff")
        self.status_label.setStyleSheet(
            "background-color: #15171c; color: #888; "
            "padding: 6px; border-radius: 4px; font-size: 11px;")
        main_layout.addWidget(self.status_label)

    def _make_separator(self):
        sep = QFrame()
        sep.setObjectName("separator")
        sep.setFrameShape(QFrame.HLine)
        return sep

    # ── Log helpers ──

    def _log(self, message: str):
        """Thread-safe log message."""
        self.log_signal.emit(message)

    def _append_log(self, message: str):
        """Append message to log widget (must be called from GUI thread)."""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] {message}")
        # Auto-scroll to bottom
        cursor = self.log_text.textCursor()
        cursor.movePosition(QTextCursor.End)
        self.log_text.setTextCursor(cursor)
        # Trim to 200 lines
        doc = self.log_text.document()
        if doc.blockCount() > 200:
            cursor = QTextCursor(doc.firstBlock())
            cursor.select(QTextCursor.BlockUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()  # remove newline

    # ── URDF joint limits update ──

    def _update_joint_limits(self):
        """Update spinbox limits when URDF is received."""
        if self.node.urdf_received:
            for i in range(7):
                lo_rad, hi_rad = self.node.joint_limits_rad[i]
                lo_deg = math.degrees(lo_rad)
                hi_deg = math.degrees(hi_rad)
                self.joint_spinboxes[i].setMinimum(lo_deg)
                self.joint_spinboxes[i].setMaximum(hi_deg)
            self._log("✅ Joint limits loaded from URDF")
            for i in range(7):
                lo, hi = self.node.joint_limits_rad[i]
                self._log(f"   J{i+1}: {math.degrees(lo):.1f}° ~ {math.degrees(hi):.1f}°")
            self.limits_timer.stop()  # stop polling

    # ── Controller toggle ──

    def _on_toggle_controller(self):
        if self.node.controller_active:
            # Deactivate
            success = self.node.switch_controller(activate=False)
            if success:
                self.ctrl_status_label.setText("Status: ⚪ INACTIVE")
                self.ctrl_status_label.setStyleSheet(
                    "color: #888; font-size: 14px; font-weight: bold;")
                self.ctrl_toggle_btn.setText("▶ Activate")
                self.ctrl_toggle_btn.setStyleSheet(TOGGLE_INACTIVE_STYLE)
                # Disable sliders
                for s in self.kp_sliders + self.kd_sliders:
                    s.setEnabled(False)
                self._log("Controller deactivated — gains restored to defaults")
            else:
                self._log("⚠️ Failed to deactivate: service unavailable")
        else:
            # Activate
            success = self.node.switch_controller(activate=True)
            if success:
                self.ctrl_status_label.setText("Status: 🟢 ACTIVE")
                self.ctrl_status_label.setStyleSheet(
                    "color: #27ae60; font-size: 14px; font-weight: bold;")
                self.ctrl_toggle_btn.setText("🔄 Deactivate")
                self.ctrl_toggle_btn.setStyleSheet(TOGGLE_ACTIVE_STYLE)
                # Re-enable sliders
                for s in self.kp_sliders + self.kd_sliders:
                    s.setEnabled(True)
                self._log("Controller activated")
                # Re-publish current slider values
                self._publish_current()
            else:
                self._log("⚠️ Failed to activate: service unavailable")

    # ── Target position ──

    def _on_run_trajectory(self):
        positions_deg = [spin.value() for spin in self.joint_spinboxes]
        positions_rad = [math.radians(d) for d in positions_deg]
        duration = self.duration_spin.value()

        success = self.node.send_target(positions_rad, duration)
        if success:
            angles_str = ", ".join([f"J{i+1}={d:.1f}°" for i, d in enumerate(positions_deg)])
            self._log(f"▶ Trajectory sent: {angles_str}  ({duration:.1f}s)")
        else:
            self._log("⚠️ Trajectory failed: JTC action server unavailable")

    def _on_home(self):
        for spin in self.joint_spinboxes:
            spin.setValue(0.0)
        success = self.node.send_home()
        if success:
            self._log("🏠 Home trajectory sent (3.0s)")
        else:
            self._log("⚠️ Home failed: JTC action server unavailable")

    # ── Slider callbacks ──

    def _on_slider_changed(self, _value):
        for i in range(7):
            kp_val = self.kp_sliders[i].value() / 10.0
            kd_val = self.kd_sliders[i].value() / 100.0
            self.kp_labels[i].setText(f"{kp_val:.1f}")
            self.kd_labels[i].setText(f"{kd_val:.2f}")
        self._publish_current()

    def _publish_current(self):
        kp = [s.value() / 10.0 for s in self.kp_sliders]
        kd = [s.value() / 100.0 for s in self.kd_sliders]
        self.node.publish_impedance(kp, kd)

    def _apply_preset(self, name):
        preset = PRESETS[name]
        for i in range(7):
            self.kp_sliders[i].blockSignals(True)
            self.kd_sliders[i].blockSignals(True)
            self.kp_sliders[i].setValue(int(preset["kp"][i] * 10))
            self.kd_sliders[i].setValue(int(preset["kd"][i] * 100))
            self.kp_labels[i].setText(f"{preset['kp'][i]:.1f}")
            self.kd_labels[i].setText(f"{preset['kd'][i]:.2f}")
            self.kp_sliders[i].blockSignals(False)
            self.kd_sliders[i].blockSignals(False)
        self._publish_current()
        self._log(f"Preset applied: {name}")

    def _set_gripper_value(self, mm_value):
        """Set gripper spinbox to a value (mm) and send immediately."""
        self.gripper_spin.setValue(mm_value)
        self._send_gripper_command()

    def _on_gripper_spin_changed(self, value):
        """Handle gripper spinbox value change — debounced send."""
        self.gripper_timer.start()

    def _send_gripper_command(self):
        """Actually send the gripper command (debounced)."""
        self.gripper_timer.stop()
        position_mm = self.gripper_spin.value()
        position_m = position_mm / 1000.0
        pct = position_mm / 32.0 if position_mm > 0 else 0.0
        mode = self._grip_mode
        success = self.node.send_gripper(position_m)
        if success:
            self._log(f"🤏 Gripper → {position_mm:.1f}mm [{mode}] ({pct*100:.0f}% open)")
        else:
            self._log("⚠️ Gripper command failed: action server unavailable")

    def _on_grip_slider_changed(self, _value):
        """Handle gripper Kp/Kd slider changes."""
        kp = self.grip_kp_slider.value() / 10.0
        kd = self.grip_kd_slider.value() / 100.0
        self.grip_kp_label.setText(f"{kp:.1f}")
        self.grip_kd_label.setText(f"{kd:.2f}")
        self.node.set_gripper_impedance(kp, kd)
        # Update grip mode indicator
        if kp >= 4.0:
            self._grip_mode = "grip"
            self.grip_g_btn.setStyleSheet(self._grip_styles["g_on"])
            self.grip_s_btn.setStyleSheet(self._grip_styles["s_off"])
        elif kp <= 1.5:
            self._grip_mode = "soft"
            self.grip_s_btn.setStyleSheet(self._grip_styles["s_on"])
            self.grip_g_btn.setStyleSheet(self._grip_styles["g_off"])
        else:
            self._grip_mode = "default"
            self.grip_g_btn.setStyleSheet(self._grip_styles["g_off"])
            self.grip_s_btn.setStyleSheet(self._grip_styles["s_off"])

    def _on_grip_mode_g(self):
        """Toggle Grip (firm) mode — Kp=5.0, Kd=0.3."""
        if self._grip_mode == "grip":
            self._grip_mode = "default"
            self.grip_g_btn.setStyleSheet(self._grip_styles["g_off"])
            self.grip_kp_slider.setValue(20)  # Kp=2.0
            self.grip_kd_slider.setValue(10)  # Kd=0.1
            self._log("🤏 Gripper mode: Default (Kp=2.0)")
        else:
            self._grip_mode = "grip"
            self.grip_g_btn.setStyleSheet(self._grip_styles["g_on"])
            self.grip_s_btn.setStyleSheet(self._grip_styles["s_off"])
            self.grip_kp_slider.setValue(50)  # Kp=5.0
            self.grip_kd_slider.setValue(30)  # Kd=0.3
            self._log("🤏 Gripper mode: Grip (Kp=5.0, Kd=0.3) — firm hold")

    def _on_grip_mode_s(self):
        """Toggle Soft (gentle) mode — Kp=1.0, Kd=0.05."""
        if self._grip_mode == "soft":
            self._grip_mode = "default"
            self.grip_s_btn.setStyleSheet(self._grip_styles["s_off"])
            self.grip_kp_slider.setValue(20)  # Kp=2.0
            self.grip_kd_slider.setValue(10)  # Kd=0.1
            self._log("🤏 Gripper mode: Default (Kp=2.0)")
        else:
            self._grip_mode = "soft"
            self.grip_s_btn.setStyleSheet(self._grip_styles["s_on"])
            self.grip_g_btn.setStyleSheet(self._grip_styles["g_off"])
            self.grip_kp_slider.setValue(10)  # Kp=1.0
            self.grip_kd_slider.setValue(5)   # Kd=0.05
            self._log("🤏 Gripper mode: Soft (Kp=1.0, Kd=0.05) — gentle hold")

    def _on_estop(self):
        # 1. Reset to defaults (full stiff)
        self._apply_preset("🔒 Full Stiff (Default)")
        # 2. Close gripper
        self._set_gripper_value(0.0)
        # 3. Send home trajectory
        self.node.send_home()
        # 4. Visual feedback
        self.estop_btn.setText("⬛  RESET SENT")
        self.estop_btn.setStyleSheet(
            ESTOP_STYLE.replace("#c0392b", "#2c3e50").replace(
                "#e74c3c", "#34495e"))
        QTimer.singleShot(2000, self._reset_estop_button)
        self._log("🚨 E-STOP triggered → defaults restored + gripper closed + home sent")

    def _reset_estop_button(self):
        self.estop_btn.setText("⬛  E-STOP")
        self.estop_btn.setStyleSheet(ESTOP_STYLE)

    def _poll_tau_ff(self):
        self.tau_ff_updated.emit(self.node.tau_ff)

    def _update_tau_labels(self, tau_ff: list):
        for i in range(min(7, len(tau_ff))):
            val = tau_ff[i]
            color = ("#50c878" if abs(val) < 5.0
                     else "#f39c12" if abs(val) < 15.0
                     else "#e74c3c")
            self.tau_labels[i].setText(f"τ: {val:+.2f}")
            self.tau_labels[i].setStyleSheet(
                f"color: {color}; font-size: 13px; font-weight: bold;")


# ─── Main ─────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="OpenArm Impedance GUI")
    parser.add_argument("--side", default="right", choices=["left", "right"],
                        help="Which arm to control")
    args, _ = parser.parse_known_args()

    rclpy.init()
    ros_node = ImpedanceGuiNode(args.side)

    # Spin ROS in a background thread
    executor = SingleThreadedExecutor()
    executor.add_node(ros_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Qt app
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    # Set dark palette
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor("#1a1d23"))
    palette.setColor(QPalette.WindowText, QColor("#e0e0e0"))
    palette.setColor(QPalette.Base, QColor("#22262e"))
    palette.setColor(QPalette.AlternateBase, QColor("#2a2f38"))
    palette.setColor(QPalette.Text, QColor("#e0e0e0"))
    palette.setColor(QPalette.Button, QColor("#2a3040"))
    palette.setColor(QPalette.ButtonText, QColor("#c8cdd3"))
    palette.setColor(QPalette.Highlight, QColor("#5b9cf5"))
    app.setPalette(palette)

    gui = ImpedanceGui(ros_node)
    gui.show()

    exit_code = app.exec_()

    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()

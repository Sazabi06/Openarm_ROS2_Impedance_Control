#!/usr/bin/env python3
# Copyright 2026 OpenArm Contributors
# SPDX-License-Identifier: Apache-2.0
"""
PyQt5 GUI for OpenArm Compliance Controller impedance tuning.

Features:
  - Per-joint Kp/Kd sliders with real-time numeric display
  - Live tau_ff readout from the controller
  - E-STOP button: resets to defaults + sends home trajectory
  - Presets: Full Stiff, Soft Wrist, Full Soft
  - Auto-publishes impedance_params on any slider change

Usage:
  ros2 run openarm_compliance_controller impedance_gui.py
  ros2 run openarm_compliance_controller impedance_gui.py --side left
"""

import sys
import argparse
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float64MultiArray
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QSlider, QPushButton, QFrame, QGroupBox,
    QSizePolicy
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont, QColor, QPalette


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
QFrame#separator {
    background-color: #2a2f38;
    max-height: 1px;
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


# ─── ROS 2 Bridge Node ───────────────────────────────────────────────
class ImpedanceGuiNode(Node):
    """Lightweight ROS 2 node for publishing impedance params and subscribing to tau_ff."""

    def __init__(self, side: str):
        super().__init__(f"impedance_gui_{side}")
        self.side = side
        prefix = f"/{side}_compliance_controller"

        self.pub = self.create_publisher(
            Float64MultiArray, f"{prefix}/impedance_params", 10
        )
        self.tau_ff = [0.0] * 7
        self.sub = self.create_subscription(
            Float64MultiArray, f"{prefix}/tau_ff", self._tau_ff_cb, 10
        )

        # Action client for sending home trajectory
        jtc_name = f"/{side}_joint_trajectory_controller"
        self.jtc_client = ActionClient(
            self, FollowJointTrajectory,
            f"{jtc_name}/follow_joint_trajectory"
        )

        self.joint_names = [
            f"openarm_{side}_joint{i}" for i in range(1, 8)
        ]

    def _tau_ff_cb(self, msg: Float64MultiArray):
        if len(msg.data) == 7:
            self.tau_ff = list(msg.data)

    def publish_impedance(self, kp_values: list, kd_values: list):
        msg = Float64MultiArray()
        msg.data = kp_values + kd_values
        self.pub.publish(msg)

    def send_home(self):
        """Send arm to zero position via JTC."""
        if not self.jtc_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("JTC action server not available for home!")
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions = [0.0] * 7
        pt.velocities = [0.0] * 7
        pt.time_from_start = Duration(sec=3)
        goal.trajectory.points = [pt]
        self.jtc_client.send_goal_async(goal)
        self.get_logger().info("E-STOP: Sending home trajectory (3s)")


# ─── Main Window ─────────────────────────────────────────────────────
class ImpedanceGui(QMainWindow):
    tau_ff_updated = pyqtSignal(list)

    def __init__(self, ros_node: ImpedanceGuiNode):
        super().__init__()
        self.node = ros_node
        self.setWindowTitle(f"OpenArm Compliance Controller — {ros_node.side.capitalize()}")
        self.setMinimumSize(900, 700)

        # Sliders storage
        self.kp_sliders = []
        self.kd_sliders = []
        self.kp_labels = []
        self.kd_labels = []
        self.tau_labels = []

        self._build_ui()
        self.setStyleSheet(DARK_STYLE)

        # Signal for thread-safe tau_ff update
        self.tau_ff_updated.connect(self._update_tau_labels)

        # Periodic tau_ff poll (10 Hz)
        self.tau_timer = QTimer(self)
        self.tau_timer.timeout.connect(self._poll_tau_ff)
        self.tau_timer.start(100)

        self._publish_current()

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setSpacing(8)
        main_layout.setContentsMargins(16, 12, 16, 12)

        # ── Header ──
        header = QHBoxLayout()
        title = QLabel(f"OpenArm Compliance Controller — {self.node.side.capitalize()} Arm")
        title.setFont(QFont("Inter", 20, QFont.Bold))
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
        sep = QFrame()
        sep.setObjectName("separator")
        sep.setFrameShape(QFrame.HLine)
        main_layout.addWidget(sep)

        # ── Joint rows ──
        for i, (label, kp_min, kp_max, kp_def, kd_min, kd_max, kd_def) in enumerate(JOINT_CONFIG):
            group = QGroupBox(label)
            grid = QGridLayout(group)
            grid.setHorizontalSpacing(16)
            grid.setVerticalSpacing(4)

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
            kp_val_lbl.setStyleSheet("color: #5b9cf5; font-weight: bold; font-size: 14px;")
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
            kd_val_lbl.setStyleSheet("color: #5b9cf5; font-weight: bold; font-size: 14px;")
            grid.addWidget(kd_val_lbl, 1, 2)
            self.kd_labels.append(kd_val_lbl)

            kd_range = QLabel(f"[{kd_min:.2f} – {kd_max:.1f}]")
            kd_range.setStyleSheet("color: #666; font-size: 11px;")
            kd_range.setFixedWidth(80)
            grid.addWidget(kd_range, 1, 3)

            # tau_ff readout
            tau_lbl = QLabel("τ_ff: —")
            tau_lbl.setFixedWidth(100)
            tau_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            tau_lbl.setStyleSheet("color: #50c878; font-size: 13px; font-weight: bold;")
            grid.addWidget(tau_lbl, 0, 4, 2, 1)
            self.tau_labels.append(tau_lbl)

            main_layout.addWidget(group)

        # ── Presets ──
        preset_layout = QHBoxLayout()
        preset_layout.setSpacing(8)
        for name, values in PRESETS.items():
            btn = QPushButton(name)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            btn.clicked.connect(lambda checked, n=name: self._apply_preset(n))
            preset_layout.addWidget(btn)
        main_layout.addLayout(preset_layout)

        # ── Status bar ──
        self.status_label = QLabel(
            f"  🟢 Connected to /{self.node.side}_compliance_controller  |  "
            f"Publishing to ~/impedance_params  |  Monitoring ~/tau_ff"
        )
        self.status_label.setStyleSheet(
            "background-color: #15171c; color: #888; "
            "padding: 6px; border-radius: 4px; font-size: 11px;"
        )
        main_layout.addWidget(self.status_label)

    # ── Callbacks ──

    def _on_slider_changed(self, _value):
        # Update labels
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

    def _on_estop(self):
        # 1. Reset to defaults (full stiff)
        self._apply_preset("🔒 Full Stiff (Default)")
        # 2. Send home trajectory
        self.node.send_home()
        # 3. Visual feedback
        self.estop_btn.setText("⬛  RESET SENT")
        self.estop_btn.setStyleSheet(
            ESTOP_STYLE.replace("#c0392b", "#2c3e50").replace("#e74c3c", "#34495e")
        )
        QTimer.singleShot(2000, self._reset_estop_button)

    def _reset_estop_button(self):
        self.estop_btn.setText("⬛  E-STOP")
        self.estop_btn.setStyleSheet(ESTOP_STYLE)

    def _poll_tau_ff(self):
        self.tau_ff_updated.emit(self.node.tau_ff)

    def _update_tau_labels(self, tau_ff: list):
        for i in range(min(7, len(tau_ff))):
            val = tau_ff[i]
            color = "#50c878" if abs(val) < 5.0 else "#f39c12" if abs(val) < 15.0 else "#e74c3c"
            self.tau_labels[i].setText(f"τ: {val:+.2f}")
            self.tau_labels[i].setStyleSheet(
                f"color: {color}; font-size: 13px; font-weight: bold;"
            )


# ─── Main ─────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="OpenArm Impedance GUI")
    parser.add_argument("--side", default="right", choices=["left", "right"],
                        help="Which arm to control")
    # ROS 2 may pass extra args; ignore them
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

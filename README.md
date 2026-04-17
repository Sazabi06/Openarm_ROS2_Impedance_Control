# OpenArm Impedance Controller

**Variable impedance control for the [OpenArm V10](https://openarm.dev) bimanual robot — enabling safe, compliant human-robot interaction.**

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green)](LICENSE)

---

## What is Impedance Control?

Traditional position-controlled robots are **rigid** — they fight any external force with the full strength of their motors. This is dangerous for human-robot interaction and delicate manipulation tasks.

**Impedance control** makes the robot behave like a **spring-damper system**, where you can tune:
- **Stiffness (Kp)**: How strongly the robot resists displacement from its target position
- **Damping (Kd)**: How much the robot resists velocity (prevents oscillation)
- **Feedforward Torque (τ_ff)**: Pre-computed torque to compensate gravity and friction

```
τ_motor = Kp · (q_desired - q_actual) + Kd · (v_desired - v_actual) + τ_ff
           ↑ spring                       ↑ damper                    ↑ model-based compensation
```

### Why Does This Matter?

| Scenario | Position Control (Rigid) | Impedance Control (Compliant) |
|----------|-------------------------|-------------------------------|
| Human touches robot arm | Robot fights back with full force ⚠️ | Robot yields gently, then returns ✅ |
| Gripper contacts object off-center | Object gets pushed away 💥 | Wrist adapts to the object shape 🤝 |
| Arm hits unexpected obstacle | Motor stalls, possible damage ❌ | Arm absorbs impact safely ✅ |
| Gravity compensation | Kp must be high to prevent sag | τ_ff carries the load, Kp can be low 🪶 |

### The Key Insight: Feedforward Torque

Without feedforward compensation, reducing Kp causes the arm to **sag under gravity**. With τ_ff, the model-based controller pre-computes the torque needed to hold the arm against gravity and friction, so the PD gains (Kp/Kd) only need to handle **small perturbations**:

```
                  Without τ_ff                      With τ_ff
                ┌───────────┐                    ┌───────────┐
  Reduce Kp ──→ │ ARM SAGS! │           Reduce Kp ──→ │ ARM HOLDS │
                │ 💀        │                    │ + is soft  │
                └───────────┘                    └───────────┘
```

---

## Architecture

This repo contains three ROS 2 packages that work together:

```
┌─────────────────────────────────────────────────────────────────┐
│                    OpenArm Control Stack                        │
│                                                                 │
│  ┌──────────────────────┐    ┌──────────────────────────────┐   │
│  │ JointTrajectory      │    │ ComplianceController         │   │
│  │ Controller           │    │ (this repo)                  │   │
│  │                      │    │                              │   │
│  │ Writes:              │    │ Writes:                      │   │
│  │  • position          │    │  • effort (τ_ff)             │   │
│  │  • velocity          │    │  • stiffness (Kp)            │   │
│  │                      │    │  • damping (Kd)              │   │
│  └──────────┬───────────┘    └──────────────┬───────────────┘   │
│             │                               │                   │
│             └─────────────┬─────────────────┘                   │
│                           ▼                                     │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │              Hardware Interface (CAN-FD)                │    │
│  │   Combines into MIT frame: {Kp, Kd, q_des, v_des, τ_ff} │    │
│  │              → Damiao Actuators                         │    │
│  └─────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────┘
```

### Packages

| Package | Description |
|---------|-------------|
| **`openarm_compliance_controller`** | Main `ros2_control` plugin. Computes τ_ff using KDL dynamics (gravity + Coriolis + friction) and manages dynamic Kp/Kd with safety limits. Includes a PyQt5 tuning GUI. |
| **`openarm_torque_observer`** | Python-based torque observer for offline model validation. Compares KDL-predicted torque with measured motor current to calibrate scale factors. |
| **`openarm_hw_control`** | Legacy impedance controller prototype and thermal watchdog. |

---

## Safety Design

Three layers of protection prevent dangerous motor commands:

```
Layer 1: Controller (openarm_compliance_controller)
  ├── Rate limiting: ΔKp ≤ 2.0/cycle, ΔKd ≤ 0.1/cycle
  ├── Clamping: kp_min ≤ Kp ≤ kp_max, kd_min ≤ Kd ≤ kd_max
  └── Default-on-deactivate: restores high-stiffness defaults

Layer 2: Hardware Interface (openarm_hardware)
  ├── Safety floor: enforces kp_min / kd_min per joint
  └── MIT frame validation before CAN transmission

Layer 3: Motor Firmware (Damiao)
  └── Hardware limits: Kp ∈ [0, 500], Kd ∈ [0, 5]
```

### Gain Limits

| Joint Group | Kp Range | Kd Range | Default Kp | Default Kd |
|-------------|----------|----------|------------|------------|
| J1-J3 (Shoulder) | 15 – 150 | 0.4 – 5.0 | 70 | 2.0 – 2.75 |
| J4 (Elbow) | 12 – 120 | 0.4 – 5.0 | 60 | 2.0 |
| J5-J7 (Wrist) | 3 – 30 | 0.1 – 2.0 | 10 | 0.5 – 0.7 |
| Gripper (DM4310) | 0.3 – 10.0 | 0.05 – 1.0 | 2.0 | 0.1 |

---

## Dependencies

This repo depends on the [OpenArm ROS 2 stack](https://github.com/enactic/openarm_ros2):

- `controller_interface` / `hardware_interface` (ros2_control)
- `kdl_parser` / `orocos_kdl` (dynamics solver)
- `realtime_tools` (RT-safe publisher/buffer)
- `openarm_description` (URDF model)
- `openarm_hardware` (CAN-FD hardware interface)
- `PyQt5` (for the tuning GUI)

> **URDF Patch Required**: The `patches/` directory contains a modified `openarm.bimanual.ros2_control.xacro` that fixes a gripper interface mismatch. Copy this to `openarm_description/urdf/ros2_control/` before building.

---

## Installation

```bash
# 1. Clone into your workspace
cd ~/ros2_ws/src
git clone https://github.com/<your-username>/Openarm_Impedance_Controller.git impedance_control

# 2. Apply URDF patch (one-time)
cp impedance_control/patches/openarm.bimanual.ros2_control.xacro \
   core/openarm_description/urdf/ros2_control/

# 3. Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 4. Build
source /opt/ros/humble/setup.bash
colcon build --packages-select openarm_compliance_controller openarm_torque_observer --symlink-install

# 5. Source
source install/setup.bash
```

---

## Quick Start

### 1. CAN-FD Setup (Required After Every Reboot)

```bash
sudo ip link set can0 down 2>/dev/null
sudo ip link set can1 down 2>/dev/null
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 up type can bitrate 1000000 dbitrate 5000000 fd on
```

### 2. Launch the Robot

```bash
# Terminal 1: Bringup (real hardware)
ros2 launch openarm_bringup openarm.bimanual.launch.py use_fake_hardware:=false

# Terminal 2: Spawn controllers
ros2 run controller_manager spawner right_compliance_controller \
  -c /controller_manager \
  --param-file $(ros2 pkg prefix openarm_compliance_controller)/share/openarm_compliance_controller/config/compliance_controller.yaml

# Spawn gripper stiffness/damping controllers (enables runtime gripper Kp/Kd)
ros2 run controller_manager spawner right_gripper_stiffness_controller -c /controller_manager
ros2 run controller_manager spawner right_gripper_damping_controller -c /controller_manager
```

### 3. Launch the Tuning GUI

```bash
# Terminal 3:
ros2 run openarm_compliance_controller impedance_gui.py --side right
```

The GUI provides:
- **Per-joint Kp/Kd sliders** with real-time value display (J1-J7 + Gripper)
- **Live τ_ff readout** with color coding (green/yellow/red)
- **Gripper control** — position input (0-32mm), Open/Close buttons
- **Gripper impedance** — G (Grip/firm, Kp=5.0) and S (Soft/gentle, Kp=1.0) toggle buttons
- **E-STOP button** — resets gains to defaults + sends arm home
- **Preset buttons** — Full Stiff, Soft Wrist, Full Soft, Extra Stiff

### 4. Dynamic Stiffness Adjustment (Programmatic)

```bash
# Make wrist compliant (J5-J7 Kp → minimum)
ros2 topic pub --once /right_compliance_controller/impedance_params \
  std_msgs/msg/Float64MultiArray \
  "{data: [70, 70, 70, 60, 3, 3, 3,  2.75, 2.5, 2.0, 2.0, 0.15, 0.12, 0.1]}"

# Monitor actual gains (shows clamped values)
ros2 topic echo /right_compliance_controller/gains --once

# Monitor feedforward torque
ros2 topic echo /right_compliance_controller/tau_ff --once
```

---

## ROS 2 Interface

### Topics Published

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `~/tau_ff` | `Float64MultiArray` | 100 Hz | Feedforward torque [J1..J7] (gravity + friction + Coriolis) |
| `~/gains` | `Float64MultiArray` | 100 Hz | Actual clamped gains [Kp1..Kp7, Kd1..Kd7] |

### Topics Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `~/impedance_params` | `Float64MultiArray` | Target gains [Kp1..Kp7, Kd1..Kd7] (14 values) |
| `/{side}_gripper_stiffness_controller/commands` | `Float64MultiArray` | Gripper Kp [1 value] |
| `/{side}_gripper_damping_controller/commands` | `Float64MultiArray` | Gripper Kd [1 value] |

### Command Interfaces (Written to Hardware)

| Interface | Description |
|-----------|-------------|
| `<joint>/effort` | Feedforward torque τ_ff |
| `<joint>/stiffness` | Position gain Kp (arm J1-J7 + gripper) |
| `<joint>/damping` | Velocity gain Kd (arm J1-J7 + gripper) |

---

## Dynamics Model

The feedforward torque is computed per cycle (100 Hz) as:

```
τ_ff = scale_i · [ τ_gravity(q) + τ_coriolis(q, q̇) + τ_friction(q̇) ]
```

Where:
- **τ_gravity**: KDL recursive Newton-Euler gravity compensation
- **τ_coriolis**: KDL Coriolis/centrifugal forces
- **τ_friction**: Calibrated nonlinear model: `Fc·tanh(0.1·k·q̇) + Fv·q̇ + Fo`
- **scale_i**: Per-joint calibration factor (e.g., J2=0.96, J4=0.67) from hardware validation

### Calibrated Scale Factors

| Joint | Scale | Reason |
|-------|-------|--------|
| J1, J3, J5, J6, J7 | 1.00 | Model matches hardware |
| J2 | 0.96 | Model overpredicts by ~4% (planetary gear backlash) |
| J4 | 0.67 | Model overpredicts by ~33% (dual-stage planetary gear) |

---

## Testing

The full test plan (simulation + real hardware) is in:

📋 **[TEST.md](openarm_compliance_controller/TEST.md)**

It includes:
- 8 simulation tests with exact commands and expected outputs
- 8 real hardware tests with step-by-step instructions
- 3 demo scenarios (Gravity Compensation, Compliant Handshake, Variable Stiffness Pick-and-Place)
- Complete troubleshooting guide

---

## Use Case: VLA Integration

A Vision-Language-Action model can output stiffness profiles alongside joint trajectories:

```python
# VLA action output
action = {
    "joint_positions": [0.1, 0.5, -0.3, 1.2, 0.0, 0.0, 0.0],
    "kp_gains": [70, 70, 70, 20, 5, 5, 5],      # soft wrist for contact
    "kd_gains": [2.75, 2.5, 2.0, 0.8, 0.3, 0.3, 0.3]
}

# Publish to compliance controller
msg = Float64MultiArray()
msg.data = action["kp_gains"] + action["kd_gains"]
impedance_pub.publish(msg)
```

This enables the robot to:
- Approach targets with high stiffness (accurate positioning)
- Switch to low stiffness during contact (safe, adaptive grasping)
- Re-stiffen for transport (secure hold)

---

## Repository Structure

```
Openarm_Impedance_Controller/
├── README.md                          # This file
├── openarm_compliance_controller/     # Main compliance controller
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── openarm_compliance_controller.xml   # pluginlib descriptor
│   ├── config/
│   │   ├── compliance_controller.yaml      # Gains, limits, friction params
│   │   └── gripper_stiffness_controller.yaml  # Gripper Kp/Kd FCC config
│   ├── include/openarm_compliance_controller/
│   │   └── compliance_controller.hpp       # Controller header
│   ├── src/
│   │   └── compliance_controller.cpp       # KDL dynamics + impedance logic
│   ├── scripts/
│   │   ├── impedance_gui.py                # PyQt5 tuning GUI (arm + gripper)
│   │   └── motor_feedback_diagnostic.py    # Hardware diagnostic tool
│   ├── launch/
│   │   └── compliance.launch.py
│   ├── TEST.md                             # Full test plan
│   ├── IMPLEMENTATION_PLAN.md              # Roadmap
│   └── AGENT_TASKS.md                      # Multi-agent task delegation
├── openarm_torque_observer/           # Torque model validation
│   ├── openarm_torque_observer/
│   │   └── torque_observer_node.py         # KDL-based torque observer
│   ├── scripts/
│   │   └── audit_kdl_masses.py             # URDF mass auditing tool
│   └── config/
│       └── friction_params.yaml
├── openarm_hw_control/                # Legacy impedance controller
│   ├── src/
│   │   ├── impedance_controller.cpp
│   │   └── thermal_watchdog.cpp
│   └── include/openarm_hw_control/
│       ├── impedance_controller.hpp
│       └── low_pass_filter.hpp
└── patches/
    └── openarm.bimanual.ros2_control.xacro  # URDF fix for gripper interfaces
```

---

## License

Apache License 2.0 — See individual package files for details.

## Acknowledgments

- [OpenArm / Enactic](https://openarm.dev) for the open-source robot platform
- [KDL (Orocos)](https://www.orocos.org/kdl.html) for the dynamics solver
- [ros2_control](https://control.ros.org/) for the controller framework

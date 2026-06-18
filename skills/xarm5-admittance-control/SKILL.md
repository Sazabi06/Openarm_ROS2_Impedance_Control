---
name: xarm5-admittance-control
description: Admittance control for xArm5 using ros2_control, UFACTORY F/T sensor, profile management, and VLA integration roadmap.
version: 1.0.0
---
# xArm5 Admittance Control

This skill covers admittance control implementation for the UFACTORY xArm5 robot. The xArm5 uses harmonic drive actuators (position-controlled, non-backdrivable), making admittance control the ONLY viable compliance approach. Read this before attempting any force-aware manipulation.

## 1. Why Admittance, Not Impedance

### Hardware Architecture Comparison

| Factor | OpenArm V10 (QDD) | xArm5 (Harmonic Drive) |
|--------|-------------------|------------------------|
| Gear ratio | ~1:9 | ~1:100 |
| Backdrivable | ✅ Yes | ❌ No |
| Raw torque API | ✅ MIT mode τ_ff | ❌ Not exposed |
| Joint torque sensing | Motor current | Not user-accessible |
| Control interface | Torque/position/velocity | Position/velocity only |
| Friction | Low (~0.1-0.3 Nm) | High (~1-3 Nm static) |

**Impedance control** outputs torque → xArm5 cannot accept raw torque commands.
**Admittance control** reads force from sensor, outputs position → xArm5 handles this perfectly.

### The Control Equations

```
IMPEDANCE (OpenArm):    τ_cmd = τ_ff + Kp·(q_des - q) + Kd·(dq_des - dq)
  Input: position error → Output: torque

ADMITTANCE (xArm5):     M·ẍ + B·ẋ + K·(x - x_d) = F_ext
  Input: external force (F/T sensor) → Output: position adjustment Δx
  Solve: Δx = integrate(F_ext / (M·s² + B·s + K))
```

Both achieve the same user-facing behavior: the robot feels compliant when touched. The implementation path is fundamentally different.

## 2. xArm5 Hardware Specifications

| Spec | Value |
|------|-------|
| DOF | 5 (J1-J5) |
| Payload | 3 kg |
| Reach | 700 mm |
| Repeatability | ±0.1 mm |
| Max joint speed | 180°/s |
| Max TCP speed | 1000 mm/s |
| Communication | Ethernet TCP/IP to control box |
| Price | ~$5,800 |

### Joint Limits

| Joint | Range |
|-------|-------|
| J1 | ±360° |
| J2 | -118° to +120° |
| J3 | -225° to +11° |
| J4 | -97° to +180° |
| J5 | ±360° |

### xArm SDK Control Modes

| Mode | Name | Use Case |
|------|------|----------|
| 0 | Position | Default, point-to-point motion |
| 1 | Servo (external planner) | MoveIt trajectory execution |
| 2 | Free-drive / Manual | Firmware gravity comp, teach by demo |
| 4 | Joint velocity | Direct joint velocity commands |
| 5 | Cartesian velocity | Direct TCP velocity commands |

**Note:** No torque/effort mode exists for end users. Mode 2 (free-drive) provides firmware-level gravity compensation but is NOT user-configurable impedance control.

## 3. F/T Sensor Requirements

### UFACTORY 6-Axis Force Torque Sensor (~$1,200)

| Spec | Value |
|------|-------|
| Axes | 6 (Fx, Fy, Fz, Tx, Ty, Tz) |
| Force range | Fx/Fy: 150N, Fz: 200N |
| Torque range | 4 Nm |
| Force resolution | ~100 mN (Fx/Fy), ~150 mN (Fz) |
| Torque resolution | ~5 mNm |
| Data rate | 200 Hz |
| Overload | 150% rated |
| Interface | MODBUS via xArm control box |

### ROS 2 Topics (auto-published by xarm_api driver)

| Topic | Type | Content |
|-------|------|---------|
| `/xarm/uf_ftsensor_raw_states` | `WrenchStamped` | Raw sensor data |
| `/xarm/uf_ftsensor_ext_states` | `WrenchStamped` | Filtered + gravity-compensated |

### SDK Setup Sequence

```python
import xarm
arm = xarm.XArmAPI('192.168.1.xxx')
arm.set_ft_sensor_enable(1)           # Enable sensor
arm.set_ft_sensor_zero()              # Zero (tare) the sensor
# For firmware-level admittance (quick test):
arm.set_ft_sensor_admittance_parameters(M, B, K)
```

## 4. Two Implementation Paths

### Path A: SDK-Level Admittance (Quick Win — Day 1)

Uses xArm firmware's built-in admittance at ~1000Hz. No custom controller needed.

```python
# Firmware handles the admittance loop internally
arm.set_ft_sensor_enable(1)
arm.set_ft_sensor_zero()
# M=[mass], B=[damping], K=[stiffness] for 6 Cartesian axes
arm.set_ft_sensor_admittance_parameters(
    M=[2.0, 2.0, 2.0, 0.3, 0.3, 0.3],
    B=[20.0, 20.0, 20.0, 2.0, 2.0, 2.0],
    K=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # K=0 for free floating
)
```

**Pros:** Runs at firmware rate (~1000Hz), no ROS latency, simple.
**Cons:** No integration with MoveIt, no profile switching, no VLA pipeline.

### Path B: ros2_control Admittance Controller (Production — 1-2 weeks)

Uses the standard `ros2_control` `admittance_controller` package.

#### Install
```bash
sudo apt install ros-humble-admittance-controller
sudo apt install ros-humble-kinematics-interface-kdl
```

#### Configuration (admittance_controller.yaml)
```yaml
admittance_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5

    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    chainable_command_interfaces:
      - position
      - velocity

    kinematics:
      plugin_name: kinematics_interface_kdl/KinematicsInterfaceKDL
      plugin_package: kinematics_interface_kdl
      base: link_base
      tip: link_eef    # xArm5 end-effector link
      alpha: 0.0005

    ft_sensor:
      name: ft_sensor
      frame:
        id: link_eef
        external: false
      filter_coefficient: 0.01  # Low = more filtering (less noise)

    control:
      frame:
        id: link_eef
        external: false

    fixed_world_frame:
      frame:
        id: link_base
        external: false

    gravity_compensation:
      frame:
        id: link_eef
        external: false
      CoG:
        pos: [0.0, 0.0, 0.05]  # Adjust for gripper CoG
        force: 4.9  # gripper_mass * 9.81 (e.g., 0.5kg gripper)

    admittance:
      selected_axes:
        - true   # x
        - true   # y
        - true   # z
        - true   # rx
        - true   # ry
        - true   # rz

      # F = M*a + D*v + K*(x - x_d)
      # Start conservative, tune on real hardware
      mass:
        - 5.0    # x (kg)
        - 5.0    # y
        - 5.0    # z
        - 0.5    # rx (kg·m²)
        - 0.5    # ry
        - 0.5    # rz

      damping_ratio:  # zeta = D / (2*sqrt(M*K))
        - 1.0    # x (critically damped)
        - 1.0    # y
        - 1.0    # z
        - 1.0    # rx
        - 1.0    # ry
        - 1.0    # rz

      stiffness:
        - 200.0  # x (N/m)
        - 200.0  # y
        - 200.0  # z
        - 20.0   # rx (Nm/rad)
        - 20.0   # ry
        - 20.0   # rz
```

#### Controller Chain Architecture
```
JointTrajectoryController (optional, for planned motions)
        ↓ (chainable reference interfaces)
AdmittanceController (reads F/T, computes Δx)
        ↓ (position commands)
xArm Hardware Interface (sends to firmware)
```

## 5. Admittance Profiles

Adapt from OpenArm's impedance_profile_manager.py:

| Profile | M (kg) | B (N·s/m) | K (N/m) | Use Case |
|---------|--------|-----------|---------|----------|
| **stiff** | 2.0 | 100 | 500 | Fast transit, precise positioning |
| **medium** | 5.0 | 50 | 200 | Approach, general manipulation |
| **compliant** | 10.0 | 20 | 50 | Contact, insertion, surface following |
| **teach** | 15.0 | 10 | 0 | Human dragging (K=0 → no spring-back) |

**Key tuning insight from OpenArm experience:**
- Higher M = slower response to force (more "inertia")
- Higher B = more damping (less oscillation, more viscous feel)
- K=0 = pure damper (no position recovery) — ideal for teach mode
- Start with critical damping: ζ = B / (2·√(M·K)) = 1.0

## 6. Safety Considerations

| Safety Feature | Implementation |
|----------------|---------------|
| Force threshold | If \|F\| > 50N → emergency stop (xArm built-in) |
| Workspace limits | xArm firmware enforces joint limits |
| Collision detection | xArm built-in (sensitivity level 1-5) |
| Sensor failure | If F/T data stops → freeze position, alert |
| Velocity limit | Admittance output capped at 100mm/s |

**xArm5 advantage over OpenArm:** The firmware has built-in collision detection (current-based) and safety boundaries. Our OpenArm required manual kp_min/kd_min safety floors because QDD motors have no firmware safety net.

## 7. VLA Integration Roadmap

The admittance controller maps to CompliantVLA the same way our OpenArm impedance controller does:

```
Paper VLM (~1Hz)  → admittance_profile_manager.py (profile switching)
Paper VLA (~3Hz)  → vla_server.py (same architecture, different joint count)
Paper VIC (1000Hz)→ admittance_controller (ros2_control, or firmware-level)
```

### xArm5 Advantages for VLA
1. **Built-in teach mode** (Mode 2) — no custom code for data collection
2. **F/T sensor data** — can be included in VLA observations for richer training
3. **Smooth position control** — no jitter from raw torque commands
4. **5-DOF simplicity** — smaller action space, faster VLA training convergence

## 8. Bringup Quick Reference

```bash
# === SETUP (one-time) ===
mkdir -p ~/xarm_ws/src && cd ~/xarm_ws/src
git clone https://github.com/xArm-Developer/xarm_ros2.git --recursive -b humble
cd ~/xarm_ws/src && rosdep install --from-paths . --ignore-src --rosdistro humble -y
cd ~/xarm_ws && colcon build && source install/setup.bash

# === SIM TEST ===
ros2 launch xarm_moveit_config xarm5_moveit_fake.launch.py

# === REAL ROBOT ===
ros2 launch xarm_moveit_config xarm5_moveit_realmove.launch.py robot_ip:=192.168.1.xxx

# === MOVEIT SERVO (keyboard) ===
ros2 launch xarm_moveit_servo xarm_moveit_servo_realmove.launch.py robot_ip:=192.168.1.xxx dof:=5
ros2 run xarm_moveit_servo xarm_keyboard_input

# === VERIFY F/T SENSOR ===
ros2 topic echo /xarm/uf_ftsensor_ext_states
```

## 9. Key Differences from OpenArm Workflow

| Step | OpenArm | xArm5 |
|------|---------|-------|
| Hardware bringup | Manual CAN-FD socketcan setup | Plug Ethernet cable |
| Motor calibration | Motor offset calibration required | Factory calibrated |
| MoveIt config | Hand-assembled from XACRO | Pre-generated in repo |
| IK solver | TRAC-IK required (7-DOF redundancy) | KDL works fine (5-DOF) |
| Compliance controller | Custom C++ plugin (594 lines) | Off-the-shelf ros2_control package |
| Force sensing | Proprioceptive (τ_motor - τ_ff, ±0.5Nm) | Dedicated F/T sensor (±100mN) |
| Teach mode | Custom kp_min profile | Built-in Mode 2 |
| Safety | Manual kp_min/kd_min floors | Firmware collision detection |
| Dev time (compliance) | 4+ weeks | 1-2 weeks |

## 10. Common Pitfalls

| Pitfall | Solution |
|---------|----------|
| F/T sensor not zeroed → constant drift | Always call `set_ft_sensor_zero()` after mounting gripper/payload |
| Gravity comp wrong → arm drifts down | Set correct CoG and mass in `gravity_compensation` params |
| Oscillation during contact | Increase damping_ratio (try 1.5-2.0) or increase mass |
| Robot doesn't respond to push | Check `selected_axes` — all must be `true` for relevant axes |
| MoveIt + admittance conflict | Use controller chaining, not separate controllers |
| Mode 2 vs admittance confusion | Mode 2 is firmware-only free-drive; admittance uses F/T sensor for controlled compliance |

---
name: openarm-hardware
description: Hardware interface knowledge for the OpenArm V10 bimanual robot — DaMiao motor specs, CAN-FD configuration, motor offset calibration, safety floors, and common troubleshooting procedures.
version: 1.0.0
---
# OpenArm V10 Hardware Interface

This skill contains the essential hardware knowledge for working with the OpenArm V10 bimanual robot platform. Any code changes involving CAN communication, motor control, joint ordering, or hardware bringup must follow these conventions.

## 1. Robot Overview

```
Robot: OpenArm V10 Bimanual
  2 × 7-DOF arms + 2 × 1-DOF grippers = 16 actuated DOF
  Communication: CAN-FD (1 Mbps nominal, 5 Mbps data)
  Bus mapping: can0 → right arm, can1 → left arm
  Control mode: DaMiao MIT mode (position + velocity + kp + kd + torque_ff)
  Control rate: 100 Hz (compliance controller update loop)
```

## 2. Motor Distribution

| Joint | Motor Model | Torque Class | Notes |
|-------|-------------|-------------|-------|
| J1, J2 | DM8009 | High torque | Shoulder — heaviest load |
| J3, J4 | DM4340 | Medium torque | Elbow |
| J5, J6, J7 | DM4310 | Low torque | Wrist — fine manipulation |
| Gripper | DM4310 | Low torque | Prismatic joint, 0–0.032m range |

## 3. Joint Naming Convention

**CRITICAL: Always match joints by NAME, never by index.**

ROS 2 `/joint_states` publishes joints in **alphabetical order**, which does NOT match the J1-J7 physical order. The compliance controller and all VLA scripts must use name-based matching.

```
Right arm joints (physical order):
  openarm_right_joint1  (J1, shoulder yaw)
  openarm_right_joint2  (J2, shoulder pitch)
  openarm_right_joint3  (J3, shoulder roll)
  openarm_right_joint4  (J4, elbow pitch)
  openarm_right_joint5  (J5, wrist yaw)
  openarm_right_joint6  (J6, wrist pitch)
  openarm_right_joint7  (J7, wrist roll)
  openarm_right_finger_joint1  (gripper)

Left arm: same pattern with "left" instead of "right"
```

**Known bug pattern:** VLA datasets and inference scripts have been corrupted by index-based joint matching. This caused a $40 wasted fine-tuning run. ALWAYS verify joint ordering end-to-end before training.

## 4. CAN-FD Setup

```bash
# Bring up CAN interfaces (run before ROS 2 launch)
sudo ip link set can0 down 2>/dev/null
sudo ip link set can1 down 2>/dev/null
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 up type can bitrate 1000000 dbitrate 5000000 fd on

# Verify
ip -details link show can0 | head -3
# Expected: UP,LOWER_UP ... <FD>
```

**Note:** DaMiao motors do NOT broadcast spontaneously. `candump can0` will show NO output when idle — this is normal. Motors only respond when a controller sends commands.

## 5. System Bringup (Step-by-Step)

### Option A: Robot + Cameras (most common)
```bash
# Terminal 1: CAN + Robot + Cameras
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch openarm_vision camera_bringup.launch.py mode:=real use_rviz:=false
```

### Option B: Robot only (no cameras)
```bash
# Terminal 1: CAN + Robot
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch openarm_bringup openarm.bimanual.launch.py use_fake_hardware:=false
```

**WARNING:** Do NOT run both — `camera_bringup.launch.py` already includes the robot bringup. Running both causes duplicate controllers.

### After bringup: start compliance controller
```bash
# Terminal 2: Compliance controllers
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch openarm_compliance_controller compliance.launch.py side:=right &
sleep 2
ros2 launch openarm_compliance_controller compliance.launch.py side:=left
```

### After compliance: start impedance profile manager
```bash
# Terminal 3: Profile manager
python3 ~/ros2_ws/src/impedance_control/openarm_compliance_controller/scripts/impedance_profile_manager.py \
  --ros-args -p side:=right
```

## 6. Source Repositories

| Repository | URL |
|-----------|-----|
| OpenArm Robot Control | https://github.com/Sazabi06/Openarm-ROS2-robot-control |
| OpenArm Impedance | https://github.com/Sazabi06/Openarm_ROS2_Impedance_Control |
| OpenArm VLA | https://github.com/Sazabi06/Openarm_VLA |

## 7. Motor Offset Calibration

The left arm J1 motor has a known physical zero offset of **+1.149 rad** from the right arm's zero. This is corrected in the hardware interface XACRO:

```xml
<!-- In openarm.bimanual.ros2_control.xacro -->
<param name="motor_offset1">-1.148814</param>
```

If a new motor is replaced, recalibrate by:
1. Physically align the joint to its known zero position
2. Read the raw encoder value via `candump`
3. Compute the offset = -(raw_encoder_value)
4. Update the XACRO parameter

## 8. Safety Floor (kp_min / kd_min)

The hardware driver enforces minimum Kp/Kd values that prevent the arm from going fully limp. These are defined in the XACRO and must match the compliance controller config:

```yaml
# Hardware enforced minimums (v10_simple_hardware.cpp)
kp_min: [12.0, 12.0, 12.0, 3.0, 2.0, 2.0, 2.0]
kd_min: [0.5, 0.5, 0.4, 0.4, 0.15, 0.12, 0.1]
```

**Why this exists:** Without kp_min, setting Kp=0 would make the arm completely limp and it could fall and damage itself or the environment. The safety floor ensures there is always some holding torque.

## 9. Key Source Files

| File | Purpose |
|------|---------|
| `openarm_hardware/src/v10_simple_hardware.cpp` | CAN-FD communication, MIT mode read/write, kp_min/kd_min enforcement |
| `openarm_description/urdf/arm/openarm_macro.xacro` | 7-DOF kinematic chain, link inertias, joint limits |
| `openarm_description/urdf/ros2_control/openarm.bimanual.ros2_control.xacro` | Hardware interface definitions, motor offsets |
| `openarm_description/urdf/ee/openarm_hand_macro.xacro` | Gripper description (prismatic, 0–32mm) |

## 10. Common Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `CAN timeout` on bringup | CAN interface not UP, or PSU off | Check `ip link show can0`, verify power LED on PSU |
| Arm drifts slowly in teach mode | tau_ff_scale miscalibrated | Increase tau_ff_scale for the drifting joint by 0.05 increments |
| Left arm J1 offset from right | Physical motor zero mismatch | Apply motor_offset1 in XACRO (currently -1.148814) |
| `motor not found` for specific joint | CAN bus noise or loose connector | Reseat CAN USB adapter, try `sudo ip link set canX down && up` |
| Joint oscillation at high Kp | Kd too low for the chosen Kp | Increase Kd. Rule of thumb: Kd ≈ 2*sqrt(Kp)/30 |

## 11. Transferring to a New Robot Platform

If porting this system to a different robot:
1. Replace the XACRO files with your robot's URDF
2. Rewrite `v10_simple_hardware.cpp` for your motor protocol (EtherCAT, RS485, etc.)
3. Recalibrate tau_ff_scale and friction coefficients on real hardware
4. Update kp_min/kd_min based on your motor's torque limits
5. Keep the compliance_controller.cpp unchanged — it is hardware-agnostic

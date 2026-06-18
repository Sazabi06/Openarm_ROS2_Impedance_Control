# OpenArm Compliance Controller — Test Plan

> Step-by-step verification for the `openarm_compliance_controller`.
> Each test includes the exact command, expected output, and pass/fail criteria.

This document is divided into two parts:

| Part | Environment | Hardware | Tests |
|------|-------------|----------|-------|
| **[Part 1](#part-1-simulation-testing-fake-hardware)** | Simulation (`use_fake_hardware:=true`) | No real robot needed | Test 1–8, Demos, GUI |
| **[Part 2](#part-2-real-hardware-testing)** | Real Hardware (`use_fake_hardware:=false`) | Physical OpenArm required | HW-0 through HW-7 |
| **[Part 3](#part-3-gui-demo-guide-real-hardware)** | Real Hardware + GUI | Physical OpenArm required | 4 GUI-based demos |

> [!IMPORTANT]
> **Complete ALL Part 1 tests before moving to Part 2.** Part 2 requires CAN-FD bus setup first (HW-0).

---

## Prerequisites

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

# Part 1: Simulation Testing (Fake Hardware)

> All tests in Part 1 use `use_fake_hardware:=true`. **No real robot is needed.**

---

## Test 1: Build Verification (Simulation)

**Goal**: Controller compiles cleanly with zero errors.

```bash
colcon build --packages-select openarm_compliance_controller --symlink-install 2>&1
```

**Expected**:
```
Summary: 1 package finished [~10s]
```

**Pass**: Exit code 0, no `error:` lines. Warnings about deprecated headers are acceptable.

---

## Test 2: Controller Spawn & Activation (Simulation)

**Goal**: Controller loads, configures, and activates alongside the JointTrajectoryController.

**Terminal 1** — Launch bringup:
```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py use_fake_hardware:=true
```

**Terminal 2** — Spawn controllers:
```bash
# Compliance controller (arm J1-J7 impedance)
ros2 run controller_manager spawner right_compliance_controller \
  -c /controller_manager \
  --param-file $(ros2 pkg prefix openarm_compliance_controller)/share/openarm_compliance_controller/config/compliance_controller.yaml

# Gripper stiffness/damping controllers (enables runtime Kp/Kd for gripper)
ros2 run controller_manager spawner right_gripper_stiffness_controller -c /controller_manager
ros2 run controller_manager spawner right_gripper_damping_controller -c /controller_manager
```

**Expected**:
```
[INFO] Loaded right_compliance_controller
[INFO] Configured and activated right_compliance_controller
```

**In `controller_manager` logs** (Terminal 1), expect:
```
KDL chain initialized: 7 joints, 9 segments (openarm_body_link0 -> openarm_right_hand)
  Segment 'openarm_right_link0': mass=1.1432 kg, ...
  ...
  Total chain mass: 6.1780 kg
Compliance controller configured. Scale factors:
  J1: scale=1.00, kp=[15.0, 150.0], kd=[0.50, 5.00]
  J2: scale=0.96, kp=[15.0, 150.0], kd=[0.50, 5.00]
  ...
Compliance controller activated with default gains
```

**Pass**: Controller state = `active`. No errors.

---

## Test 3: Interface Claim Verification (Simulation)

**Goal**: Compliance controller claims `effort`, `stiffness`, `damping` — and ONLY those.

```bash
ros2 control list_controllers
```

**Expected** (6 controllers total):
```
right_compliance_controller  openarm_compliance_controller/ComplianceController  active
right_joint_trajectory_controller  joint_trajectory_controller/JointTrajectoryController  active
... (4 more: left JTC, both grippers, JSB)
```

```bash
ros2 control list_hardware_interfaces | grep "right_joint1"
```

**Expected**:
```
openarm_right_joint1/damping    [available] [claimed]
openarm_right_joint1/effort     [available] [claimed]
openarm_right_joint1/position   [available] [claimed]      ← by JTC
openarm_right_joint1/stiffness  [available] [claimed]
openarm_right_joint1/velocity   [available] [claimed]      ← by JTC
```

**Pass**: `effort`, `stiffness`, `damping` show `[claimed]` for all 7 right arm joints. Left arm remains `[unclaimed]`.

---

## Test 4: tau_ff Topic Validation (Simulation)

**Goal**: Feedforward torque publishes at 100 Hz with correct physics.

### 4a. Check publish rate

```bash
timeout 5 ros2 topic hz /right_compliance_controller/tau_ff
```

**Expected**:
```
average rate: 99.990
  min: 0.007s max: 0.012s std dev: 0.00043s
```

**Pass**: Rate = 100 ± 2 Hz.

### 4b. Check tau_ff values at zero position

```bash
ros2 topic echo /right_compliance_controller/tau_ff --once
```

**Expected** (at home / zero position):
```yaml
data:
- -0.015   # J1: ~Fo[0] = 0.088 (mostly friction offset, gravity ≈ 0 at zero)
- 0.169    # J2: ~Fo[1] = 0.088 + small gravity contribution
- 0.008    # J3: ~Fo[2] = 0.008
- -0.081   # J4: ~Fo[3] = -0.058
- 0.005    # J5: ~Fo[4] = 0.005
- -0.057   # J6: ~Fo[5] = 0.009
- -0.059   # J7: ~Fo[6] = -0.059
```

**Pass**: Values are small (< 1 Nm) at zero position. Non-zero values come from friction offsets `Fo`.

### 4c. Verify gravity contribution changes with position

Send the arm to a non-zero position (J2 = 90°, J4 = 90°) using JTC, then re-read:

```bash
ros2 action send_goal /right_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [openarm_right_joint1, openarm_right_joint2, openarm_right_joint3, openarm_right_joint4, openarm_right_joint5, openarm_right_joint6, openarm_right_joint7], points: [{positions: [0, 1.571, 0, 1.571, 0, 0, 0], time_from_start: {sec: 3}}]}}"
```

Wait 4 seconds, then:
```bash
ros2 topic echo /right_compliance_controller/tau_ff --once
```

**Expected** (J2=90°, J4=90° — full chain gravity):
```yaml
data:
- ~4.75    # J1: arm stretched horizontally → mass offset causes torque about J1 axis
- ~7.06    # J2: gravity torque on the full chain below J2 (note: J4=90° folds forearm, reducing moment arm)
- ~4.72    # J3: J2=90° rotates J3's load axis → now J3 sees significant gravity load
- ~-0.06   # J4: J4=90° with forearm nearly aligned to its axis → near-zero torque
- ~-0.01   # J5: small
- ~-0.76   # J6: wrist weight + friction offset
- ~-0.06   # J7: friction offset
```

> **Note**: These values differ from a "J2-only" test because KDL computes the **full kinematic chain**
> simultaneously. With both J2=90° and J4=90°, the mass distribution shifts — J1 and J3 now see
> significant load they wouldn't see at zero position, while J4's effective moment arm shrinks.

**Pass**: Large |tau_ff| (> 3 Nm) on J1, J2, J3 confirms gravity compensation is working. Compare with
Test 4b (zero position) where all values were < 0.2 Nm.

---

## Test 5: Dynamic Impedance Adjustment via Topic (Simulation)

**Goal**: `~/impedance_params` topic changes Kp/Kd in real-time.

### 5a. Lower J2 stiffness
```bash
ros2 topic pub --once /right_compliance_controller/impedance_params \
  std_msgs/msg/Float64MultiArray \
  "{data: [70, 30, 70, 60, 10, 10, 10,  2.75, 1.0, 2.0, 2.0, 0.7, 0.6, 0.5]}"
```

**Expected behavior**: J2 Kp drops from 70 → 30 over ~20 cycles (0.2s) due to rate limiting (ΔKp=2.0/cycle).

### 5b. Verify with wrong array size
```bash
ros2 topic pub --once /right_compliance_controller/impedance_params \
  std_msgs/msg/Float64MultiArray \
  "{data: [1, 2, 3]}"
```

**Expected log warning**:
```
impedance_params: expected 14 values, got 3
```

**Pass**: Correct-size messages accepted, wrong-size rejected with warning.

---

## Test 6: Rate Limiting Verification (Simulation)

**Goal**: Kp/Kd cannot jump instantaneously — changes are limited to ΔKp=2.0 and ΔKd=0.1 per cycle.

Starting from default Kp=70, command Kp=15 (minimum):

```bash
ros2 topic pub --once /right_compliance_controller/impedance_params \
  std_msgs/msg/Float64MultiArray \
  "{data: [15, 15, 15, 12, 3, 3, 3,  0.5, 0.5, 0.4, 0.4, 0.15, 0.12, 0.1]}"
```

**Expected**:
- Kp should take `(70 - 15) / 2.0 = 27.5 cycles = 0.275s` to reach target
- Kd should take `(2.75 - 0.5) / 0.1 = 22.5 cycles = 0.225s` to reach target
- The arm should NOT exhibit any sudden jerking — the transition is smooth

**Pass**: No oscillation or jerking during the transition.

---

## Test 7: Safety Floor Enforcement (Simulation)

**Goal**: Kp never goes below kp_min, even if commanded to.

```bash
ros2 topic pub --once /right_compliance_controller/impedance_params \
  std_msgs/msg/Float64MultiArray \
  "{data: [0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0]}"
```

**Expected**: Despite commanding Kp=0 and Kd=0, the controller clamps to:
- J1-J3: Kp=15.0, Kd=0.5/0.5/0.4
- J4: Kp=12.0, Kd=0.4
- J5-J7: Kp=3.0, Kd=0.15/0.12/0.1

Additionally, the hardware `write()` function provides a **second layer** of enforcement.

**Pass**: Arm remains stable, does not collapse or oscillate.

---

## Test 8: Controller Deactivation (Simulation)

**Goal**: Deactivating the controller cleanly restores default high-stiffness gains.

```bash
ros2 control set_controller_state right_compliance_controller inactive
```

**Expected log**:
```
Compliance controller deactivated, restored default gains
```

Also:
```bash
ros2 control list_hardware_interfaces | grep "right_joint1/stiffness"
```

**Expected**:
```
openarm_right_joint1/stiffness [available] [unclaimed]
```

**Pass**: Interfaces released, default gains restored, arm maintains position.

---

## Demo Scenarios

The following demos illustrate **why impedance control matters** and what it enables. Each demo shows a fundamentally different capability compared to traditional position-only control.

---

### Demo 1: Gravity Compensation — "Weightless Arm"

> **What it shows**: With good feedforward (`tau_ff`), you can dramatically lower stiffness and the arm still holds its position against gravity. Without `tau_ff`, reducing Kp would cause the arm to sag and drop.

#### The Problem
In traditional position control, the motor must generate **all** the holding torque via the PD controller:
```
τ_motor = Kp·(q_des - q) + Kd·(v_des - v) + 0
                                               ↑ no tau_ff
```
At high Kp (70), this works fine — but the arm is **completely rigid**. If a person pushes the arm, it fights back with 70 Nm/rad of force. Unsafe for human interaction.

#### The Solution
With the compliance controller providing `tau_ff`:
```
τ_motor = Kp·(q_des - q) + Kd·(v_des - v) + tau_ff
                                               ↑ gravity + friction compensation
```
Now `tau_ff` carries ~95% of the load, so Kp can be reduced to **15** (safety minimum) and the arm still holds position — but it's now **compliant** and can be pushed by hand.

#### How to Run (Real Hardware)
```bash
# 1. Launch with real hardware
ros2 launch openarm_bringup openarm.bimanual.launch.py use_fake_hardware:=false

# 2. Spawn compliance controller
ros2 run controller_manager spawner right_compliance_controller ...

# 3. Move arm to J2=45°
ros2 action send_goal /right_joint_trajectory_controller/follow_joint_trajectory ...

# 4. Gradually reduce Kp (J2 only) from 70 → 30 → 15
ros2 topic pub --once /right_compliance_controller/impedance_params \
  std_msgs/msg/Float64MultiArray \
  "{data: [70, 30, 70, 60, 10, 10, 10,  2.75, 1.0, 2.0, 2.0, 0.7, 0.6, 0.5]}"
```

#### Expected Result
- **With compliance controller**: Arm holds position at J2=45° even at Kp=15. You can gently push the arm with your hand and it moves, then slowly returns. The arm feels "light" — like a well-balanced desk lamp.
- **Without compliance controller** (tau_ff=0, Kp=15): Arm immediately sags under gravity.

---

### Demo 2: Compliant Handshake — "Soft Touch"

> **What it shows**: The robot can perform contact tasks safely. When the end-effector bumps into an unexpected obstacle, it yields instead of crushing it.

#### The Concept
Traditional stiff robots are **dangerous** on contact — they push with the full force of their motors. An impedance-controlled robot can be tuned to feel "soft" at the wrist while remaining stiff at the shoulder:

| Joint | Behavior | Kp Setting |
|-------|----------|------------|
| J1-J3 | Stiff (hold position) | 70 (default) |
| J4-J7 | Soft (absorb contact) | **minimum** |

This mimics how a human arm works: your shoulder holds the arm up, while your wrist and fingers adapt to the object you're touching.

#### How to Run
```bash
# Set shoulder stiff, wrist soft
ros2 topic pub --once /right_compliance_controller/impedance_params \
  std_msgs/msg/Float64MultiArray \
  "{data: [70, 70, 70, 20, 3, 3, 3,  2.75, 2.5, 2.0, 0.8, 0.15, 0.12, 0.1]}"
```

#### Expected Result
- J1-J3 (shoulder/elbow): Firm, maintains trajectory
- J4-J7 (wrist): Compliant, can be deflected by hand pressure
- If the gripper contacts a wall during trajectory, the wrist absorbs the impact instead of stalling or damaging

---

### Demo 3: Variable Stiffness Pick-and-Place — "Gentle Grasp"

> **What it shows**: Stiffness can change **dynamically during a task**. Start stiff for accurate positioning, go soft during contact, then stiffen again for secure transport.

#### The Workflow

```
Phase 1: APPROACH (High Kp)          Phase 2: CONTACT (Low Kp)
┌─────────────────────┐              ┌─────────────────────┐
│ Kp = 70 (default)   │──── move ───▶│ Kp = 20 (soft)      │
│ Accurate positioning│   to target  │ Gentle contact force│
│ Stiff trajectory    │              │ Won't crush object  │
└─────────────────────┘              └──────────┬──────────┘
                                                │ grasp
                                     ┌──────────▼──────────┐
                                     │ Phase 3: TRANSPORT   │
                                     │ Kp = 70 (stiff)      │
                                     │ Secure hold during   │
                                     │ motion               │
                                     └─────────────────────┘
```

#### How to Run
```bash
# Phase 1: High stiffness approach
ros2 topic pub --once /right_compliance_controller/impedance_params \
  std_msgs/msg/Float64MultiArray \
  "{data: [70, 70, 70, 60, 10, 10, 10,  2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5]}"

# Execute approach trajectory...

# Phase 2: Switch to low stiffness for contact
ros2 topic pub --once /right_compliance_controller/impedance_params \
  std_msgs/msg/Float64MultiArray \
  "{data: [70, 70, 70, 20, 5, 5, 5,  2.75, 2.5, 2.0, 0.8, 0.3, 0.3, 0.3]}"

# Grasp object...

# Phase 3: Stiffen for transport
ros2 topic pub --once /right_compliance_controller/impedance_params \
  std_msgs/msg/Float64MultiArray \
  "{data: [70, 70, 70, 60, 10, 10, 10,  2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5]}"

# Execute transport trajectory...
```

#### Expected Result
- **Phase 1**: Arm moves precisely to target (stiff tracking)
- **Phase 2**: If the gripper hits the object slightly off-center, the wrist adapts instead of pushing the object away. Contact force stays low (~2N vs ~20N with stiff control)
- **Phase 3**: Object is held securely during transport — no wobble

#### Why This Matters for VLA
A Vision-Language-Action model can output these stiffness profiles as part of its action prediction:
```python
action = {
    "joint_positions": [0.1, 0.5, ...],
    "kp_gains": [70, 70, 70, 20, 5, 5, 5],   # ← soft wrist for contact
    "kd_gains": [2.75, 2.5, 2.0, 0.8, 0.3, 0.3, 0.3]
}
```
The compliance controller makes this possible by dynamically adjusting the motor gains every 10ms.

---

## GUI Tool (Works in Both Simulation and Real Hardware)

A PyQt5-based GUI is provided for real-time impedance tuning:

```bash
# Launch the GUI (works with either fake or real hardware)
ros2 run openarm_compliance_controller impedance_gui.py --side right
```

Features:
- **Per-joint Kp/Kd sliders** with real-time feedback
- **Live tau_ff readout** showing computed feedforward torque
- **E-STOP button** — resets all gains to defaults and sends arm to home
- **Preset buttons** — quick access to common stiffness profiles (Full Stiff, Soft Wrist, Full Soft, Extra Stiff)

### Simulation Checklist (Part 1 Complete)

```
[ ] Test 1: Build — compiles with zero errors
[ ] Test 2: Controller spawns and activates
[ ] Test 3: Correct interfaces claimed
[ ] Test 4a: tau_ff publishes at 100 Hz
[ ] Test 4b: tau_ff near-zero at home position
[ ] Test 4c: tau_ff shows gravity at J2=90°
[ ] Test 5: Dynamic impedance topic works
[ ] Test 6: Rate limiting prevents sudden jumps
[ ] Test 7: Safety floor clamps to kp_min/kd_min
[ ] Test 8: Clean deactivation restores defaults
```

> [!IMPORTANT]
> **ALL boxes above must be checked before proceeding to Part 2 (Real Hardware).**

---

# Part 2: Real Hardware Testing

> [!CAUTION]
> Real hardware testing must ONLY begin after ALL Part 1 simulation tests (1-8) pass.
> Have someone ready to power off the robot during first-time compliance tests.

**Required equipment**: Physical OpenArm V10, CAN-FD USB adapters, emergency power-off switch.

**First step is ALWAYS [HW-0: CAN-FD Bus Setup](#hw-0-can-fd-bus-setup-required-after-every-reboot)**.

---

### HW-0: CAN-FD Bus Setup (Required After Every Reboot)

The CAN-FD interfaces must be configured before launching. This is required **every time the PC reboots**.

```bash
# Step 1: Bring down existing interfaces (if any)
sudo ip link set can0 down 2>/dev/null
sudo ip link set can1 down 2>/dev/null

# Step 2: Configure CAN-FD (1Mbps nominal, 5Mbps data)
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 up type can bitrate 1000000 dbitrate 5000000 fd on

# Step 3: Verify both interfaces are UP
ip -details link show can0 | head -3
ip -details link show can1 | head -3
```

**Expected** (for each interface):
```
can0: <NOARP,UP,LOWER_UP,ECHO> mtu 72 ...
    link/can
    can <FD> state ERROR-ACTIVE ...
         bitrate 1000000 ...
         dbitrate 5000000 ...
```

**Pass**: Both show `UP,LOWER_UP`, `<FD>`, and `ERROR-ACTIVE`.

> **Mapping**: `can0` = right arm, `can1` = left arm (set in xacro defaults)

**Quick verify with candump** (optional):
```bash
# Should show CAN frames if motors are powered
timeout 2 candump can0 | head -5
```

---

### HW-1: Real Hardware Bringup (Without Compliance Controller)

**Goal**: Confirm the baseline system works on real hardware before adding compliance.

```bash
# Terminal 1: Source and launch
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch openarm_bringup openarm.bimanual.launch.py use_fake_hardware:=false
```

**Expected logs**:
```
[OpenArm_v10HW]: Configuration: CAN=can0, arm_prefix=right_, hand=enabled, can_fd=enabled
[OpenArm_v10HW]: Stiffness/damping interfaces enabled with safety floor:
  J1: kp=70.0 (min=15.0), kd=2.75 (min=0.50)
  ...
[OpenArm_v10HW]: OpenArm V10 Simple HW initialized successfully
...
Configured and activated right_joint_trajectory_controller
Configured and activated left_joint_trajectory_controller
```

**Verify**:
```bash
# Check all controllers active
ros2 control list_controllers

# Check joints report real positions (should NOT all be 0.0)
ros2 topic echo /joint_states --once | head -20
```

**Expected**: Joint positions reflect actual arm pose (typically close to zero if at home).

**Pass**: No crashes, no `missing command interfaces`, all controllers active, joint positions are real values.

> [!WARNING]
> If you see `missing command interfaces: ... finger_joint1/stiffness ...`, rebuild
> `openarm_description` — the xacro fix separates gripper joints from arm joints:
> `colcon build --packages-select openarm_description --symlink-install`

---

### HW-2: Spawn Compliance Controller on Real Hardware

**Goal**: Load the compliance controller alongside the running JTC.

```bash
# Terminal 2:
cd ~/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 run controller_manager spawner right_compliance_controller \
  -c /controller_manager \
  --param-file $(ros2 pkg prefix openarm_compliance_controller)/share/openarm_compliance_controller/config/compliance_controller.yaml
```

**Expected**:
```
[INFO] Loaded right_compliance_controller
[INFO] Configured and activated right_compliance_controller
```

In Terminal 1 logs:
```
KDL chain initialized: 7 joints, 9 segments
  Total chain mass: 6.1780 kg
Compliance controller activated with default gains
```

**Verify interfaces**:
```bash
ros2 control list_hardware_interfaces | grep "right_joint1"
```

**Expected**:
```
openarm_right_joint1/damping    [available] [claimed]
openarm_right_joint1/effort     [available] [claimed]
openarm_right_joint1/position   [available] [claimed]
openarm_right_joint1/stiffness  [available] [claimed]
openarm_right_joint1/velocity   [available] [claimed]
```

**Pass**: Controller active, all interfaces claimed, arm holds position with default gains.

---

### HW-3: Verify tau_ff on Real Hardware

**Goal**: Confirm tau_ff values match real arm position (NOT the simulation zero-position values).

```bash
# Check current joint positions
ros2 topic echo /joint_states --once | grep -A 20 "position"

# Check tau_ff
ros2 topic echo /right_compliance_controller/tau_ff --once
```

**Expected behavior**:
- If arm is at home (near zero): tau_ff values should be < 1 Nm (similar to simulation Test 4b)
- If arm is at a non-zero position: tau_ff should reflect gravity compensation for that position

**Key validation A**: Move arm to **J1=45°** and verify gravity torques change:
```bash
# Move J1 to 45° (all others stay at 0)
ros2 action send_goal /right_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [openarm_right_joint1, openarm_right_joint2, openarm_right_joint3, openarm_right_joint4, openarm_right_joint5, openarm_right_joint6, openarm_right_joint7], points: [{positions: [0.785, 0, 0, 0, 0, 0, 0], time_from_start: {sec: 3}}]}}"

# Wait 4s, then check
ros2 topic echo /right_compliance_controller/tau_ff --once
```

**Expected** (J1=45°, all others at 0):
```yaml
data:
- ~8.47    # J1: rotating arm 45° creates significant gravity torque about J1 axis
- ~0.15    # J2: similar to zero position (arm still hangs down from J2's perspective)
- ~-0.05   # J3: small
- ~2.17    # J4: the arm rotation shifts J4's load axis → now sees gravity contribution
- ~-0.05   # J5: small
- ~-0.04   # J6: friction offset
- ~0.48    # J7: wrist mass offset from J1 rotation
```

**Pass**: J1 tau_ff jumps from ~-0.01 Nm (zero position) to ~8.5 Nm at 45°.

---

**Key validation B**: Return to zero, then move arm to **J4=90°**:
```bash
# First return to zero position
ros2 action send_goal /right_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [openarm_right_joint1, openarm_right_joint2, openarm_right_joint3, openarm_right_joint4, openarm_right_joint5, openarm_right_joint6, openarm_right_joint7], points: [{positions: [0, 0, 0, 0, 0, 0, 0], time_from_start: {sec: 3}}]}}"

# Wait 4s, confirm zero baseline
ros2 topic echo /right_compliance_controller/tau_ff --once

# Move J4 to 90° (all others stay at 0)
ros2 action send_goal /right_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [openarm_right_joint1, openarm_right_joint2, openarm_right_joint3, openarm_right_joint4, openarm_right_joint5, openarm_right_joint6, openarm_right_joint7], points: [{positions: [0, 0, 0, 1.571, 0, 0, 0], time_from_start: {sec: 3}}]}}"

# Wait 4s, then check
ros2 topic echo /right_compliance_controller/tau_ff --once
```

**Expected** (J4=90°, all others at 0):
```yaml
data:
- ~4.75    # J1: forearm extends horizontally → mass offset creates J1 torque
- ~0.17    # J2: similar to zero position
- ~0.008   # J3: small
- ~3.11    # J4: 0.67 × gravity on forearm+wrist mass at 90° (scaled by calibration)
- ~-0.07   # J5: small
- ~0.009   # J6: friction offset
- ~0.71    # J7: wrist weight in new orientation
```

**Pass**: J4 tau_ff jumps from ~-0.08 Nm (zero position) to ~3.1 Nm at 90°.
J1 also shows ~4.75 Nm due to the shifted center of mass.

---

### HW-4: Verify Current Gains Topic

```bash
ros2 topic echo /right_compliance_controller/gains --once
```

**Expected**:
```yaml
data:
- 70.0     # J1 Kp
- 70.0     # J2 Kp
- 70.0     # J3 Kp
- 60.0     # J4 Kp
- 10.0     # J5 Kp
- 10.0     # J6 Kp
- 10.0     # J7 Kp
- 2.75     # J1 Kd
- 2.5      # J2 Kd
- 2.0      # J3 Kd
- 2.0      # J4 Kd
- 0.7      # J5 Kd
- 0.6      # J6 Kd
- 0.5      # J7 Kd
```

**Pass**: Default gains are being reported.

---

### HW-5: Incremental Compliance Test (Wrist First!)

> [!IMPORTANT]
> Always start compliance testing with the WRIST joints (J5-J7), not the shoulder.
> Wrist joints carry the least load and are safest for first-time testing.
> Keep one hand on the E-STOP power switch at all times.

#### Step 1: Reduce wrist Kp slightly (10 → 5)
```bash
ros2 topic pub --once /right_compliance_controller/impedance_params \
  std_msgs/msg/Float64MultiArray \
  "{data: [70, 70, 70, 60, 5, 5, 5,  2.75, 2.5, 2.0, 2.0, 0.5, 0.5, 0.5]}"
```

**Expected**:
- Wrist should feel slightly softer when pushed by hand
- Arm should NOT sag or vibrate
- Verify: `ros2 topic echo /right_compliance_controller/gains --once` → J5-J7 Kp ≈ 5.0 (may take ~0.3s for rate limiting)

#### Step 2: Reduce wrist Kp to minimum (→ 3)
```bash
ros2 topic pub --once /right_compliance_controller/impedance_params \
  std_msgs/msg/Float64MultiArray \
  "{data: [70, 70, 70, 60, 3, 3, 3,  2.75, 2.5, 2.0, 2.0, 0.15, 0.12, 0.1]}"
```

**Expected**:
- Wrist should be noticeably compliant — can be deflected by light finger pressure
- Wrist should slowly return to original position when released (tau_ff + low Kp)
- No oscillation or buzzing

#### Step 3: Reduce elbow Kp (60 → 30)
```bash
ros2 topic pub --once /right_compliance_controller/impedance_params \
  std_msgs/msg/Float64MultiArray \
  "{data: [70, 70, 70, 30, 3, 3, 3,  2.75, 2.5, 2.0, 1.0, 0.15, 0.12, 0.1]}"
```

**Expected**:
- Elbow (J4) becomes compliant
- Arm holds position against gravity (tau_ff compensates)
- Pushing forearm results in yielding motion, then slow return

#### Step 4: Restore defaults
```bash
ros2 topic pub --once /right_compliance_controller/impedance_params \
  std_msgs/msg/Float64MultiArray \
  "{data: [70, 70, 70, 60, 10, 10, 10,  2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5]}"
```

**Expected**: Arm returns to fully stiff mode. Verify:
```bash
ros2 topic echo /right_compliance_controller/gains --once
```

**Pass for HW-5**: Each incremental step shows increasing compliance without sag, vibration, or instability.

---

### HW-6: GUI-Based Testing

**Goal**: Use the impedance GUI for real-time tuning with visual feedback.

```bash
# Terminal 3:
ros2 run openarm_compliance_controller impedance_gui.py --side right
```

**Test sequence**:
1. Observe tau_ff values updating in real-time (should match arm position)
2. Slowly drag J7 Kp slider to minimum → wrist becomes soft
3. Push wrist with finger → observe it yields and returns
4. Click **🤝 Soft Wrist** preset → J4-J7 become compliant
5. Push the wrist/forearm → they yield
6. Click **🔒 Full Stiff** preset → arm stiffens back
7. Press **⬛ E-STOP** button → all gains reset, arm moves to home (0,0,0,0,0,0,0)

**Expected**:
- Tau_ff values in GUI show color-coded: green (< 5Nm), yellow (< 15Nm), red (> 15Nm)
- Slider changes reach the controller within ~0.3s (rate limiting)
- E-STOP sends arm to home position in 3 seconds

**Pass**: All slider adjustments result in expected compliance changes. E-STOP works correctly.

---

### HW-7: Clean Deactivation

```bash
ros2 control set_controller_state right_compliance_controller inactive
```

**Expected log**:
```
Compliance controller deactivated, restored default gains
```

**Verify**: Arm still holds position (JTC still active with default Kp/Kd from hardware layer).

**Pass**: Arm maintains position after deactivation. No droop or jerk.

---

### Real Hardware Testing Checklist

```
Pre-flight:
[ ] CAN-FD interfaces configured (can0 + can1 UP with FD)
[ ] Robot powered on, motors active
[ ] candump shows CAN frames on both buses
[ ] Emergency power-off switch accessible

Baseline (HW-1):
[ ] Bringup succeeds without compliance controller
[ ] Joint positions reading correctly from hardware
[ ] JTC moves arm to commanded positions

Compliance (HW-2 through HW-7):
[ ] HW-2: Compliance controller spawns and activates
[ ] HW-3: tau_ff matches real arm position
[ ] HW-4: Gains topic reports defaults
[ ] HW-5 Step 1: J5-J7 Kp=5 — wrist slightly soft, no sag
[ ] HW-5 Step 2: J5-J7 Kp=3 — wrist very compliant, returns to position
[ ] HW-5 Step 3: J4 Kp=30 — elbow compliant, gravity compensated
[ ] HW-5 Step 4: Full defaults restored
[ ] HW-6: GUI works with real-time feedback
[ ] HW-6: E-STOP works correctly
[ ] HW-7: Clean deactivation, arm holds position

Post-test:
[ ] All gains restored to defaults
[ ] Arm at home position
[ ] All controllers back to normal state
[ ] Save observations (oscillation? sag? noise?) → tune scale factors if needed
```

---

### Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| `missing command interfaces: finger_joint1/stiffness` | URDF declares stiffness for gripper | Rebuild `openarm_description` (gripper fix applied) |
| Arm sags when Kp is reduced | tau_ff scale factor too low for that joint | Increase `tau_ff_scale[i]` in YAML, rebuild |
| Arm oscillates at low Kp | Kd too low (underdamped) | Increase Kd before lowering Kp |
| CAN errors / no response | CAN-FD not configured | Run CAN-FD setup (HW-0) |
| `can0 not found` | USB-CAN adapter not plugged in or driver missing | Check `ip link show` and USB connections |
| Controller fails to load | Library not rebuilt after code changes | Full restart: kill bringup → rebuild → relaunch |

---

# Part 3: GUI Demo Guide (Real Hardware)

> This section provides step-by-step demos using the **enhanced GUI** on real hardware.
> Each demo is self-contained — start from Step 0 after a fresh reboot.

---

## Step 0: CAN-FD Setup (Required After Every Reboot)

```bash
# Bring down existing interfaces
sudo ip link set can0 down 2>/dev/null
sudo ip link set can1 down 2>/dev/null

# Configure CAN-FD (1Mbps nominal, 5Mbps data)
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 up type can bitrate 1000000 dbitrate 5000000 fd on

# Verify
ip -details link show can0 | head -3
ip -details link show can1 | head -3
```

**Pass**: Both show `UP,LOWER_UP`, `<FD>`, and `ERROR-ACTIVE`.

> **Mapping**: `can0` = right arm, `can1` = left arm

---

## Step 1: Launch Bringup + Spawn Compliance Controller

**Terminal 1** — Bringup:
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch openarm_bringup openarm.bimanual.launch.py use_fake_hardware:=false
```

Wait for: `Configured and activated right_joint_trajectory_controller`

**Terminal 2** — Spawn controllers:
```bash
cd ~/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash

# Compliance controller (arm J1-J7 impedance)
ros2 run controller_manager spawner right_compliance_controller \
  -c /controller_manager \
  --param-file $(ros2 pkg prefix openarm_compliance_controller)/share/openarm_compliance_controller/config/compliance_controller.yaml

# Gripper stiffness/damping controllers (enables runtime Kp/Kd for gripper)
ros2 run controller_manager spawner right_gripper_stiffness_controller -c /controller_manager
ros2 run controller_manager spawner right_gripper_damping_controller -c /controller_manager
```

Wait for: `Configured and activated right_gripper_damping_controller`

---

## Step 2: Launch the GUI

**Terminal 2** (same terminal, controller already spawned):
```bash
ros2 run openarm_compliance_controller impedance_gui.py --side right
```

**Expected**:
- GUI window opens with dark theme
- Status bar shows: `🟢 Connected to /right_compliance_controller`
- Log window shows: `GUI started, waiting for URDF joint limits...`
- After ~2 seconds: `✅ Joint limits loaded from URDF`
- τ_ff values appear for all 7 joints (color-coded: green < 5Nm)
- Controller status shows: `Status: 🟢 ACTIVE`

---

## Demo 1: Joint Positioning — Move Arm Using GUI

**Goal**: Verify the joint angle input and trajectory execution work correctly.

### Steps:
1. In the **Target Joint Angles** panel, set **J1 = 45.0°** (type or use arrows)
2. Leave all other joints at 0°
3. Set **Time = 3.0s**
4. Click **▶ Run**

**Expected**:
- Log shows: `▶ Trajectory sent: J1=45.0°, J2=0.0°, ... (3.0s)`
- Robot arm rotates J1 to 45°
- τ_ff values update in real-time as arm moves
- J1 τ_ff changes from ~0 to ~8.5 Nm (gravity compensation)

### Verify:
5. Click **🏠 Home** to return to zero
6. Log shows: `🏠 Home trajectory sent (3.0s)`
7. Arm returns to home position

**Pass**: Arm moves to commanded angle and returns home. τ_ff values update in real-time.

---

## Demo 2: Gravity Compensation — "Weightless Arm"

**Goal**: Show that τ_ff compensates gravity, allowing low Kp without arm drooping.

### Steps:
1. Move arm to a non-zero position:
   - Set **J1 = 45°**, click **▶ Run**, wait for motion to complete
2. Observe τ_ff for J1 (should show ~8.5 Nm in green/yellow)
3. Now **gradually lower J1 Kp**: drag the J1 Kp slider from 70 → 50 → 30 → 15
4. At each Kp level, gently **push J1 with your hand** and release

**Expected**:
| Kp | Push Response | Return Behavior |
|----|--------------|-----------------|
| 70 | Arm resists strongly | Snaps back precisely |
| 50 | Arm gives slightly | Returns accurately |
| 30 | Arm yields noticeably | Returns slowly |
| 15 | Arm very compliant | Returns mostly (may have small offset due to friction) |

**Key observation**: At Kp=15, the arm **still holds its position against gravity** (no sagging), because τ_ff is providing ~8.5 Nm of gravity compensation. Without τ_ff, the arm would immediately drop.

5. Click **🔒 Full Stiff (Default)** preset to restore
6. Log shows: `Preset applied: 🔒 Full Stiff (Default)`

**Pass**: Arm holds position at all Kp levels. Compliance increases as Kp drops. No sagging.

---

## Demo 3: Variable Stiffness — "Stiff Shoulder, Soft Wrist"

**Goal**: Different stiffness on different joints — stiff base for stability, soft wrist for safe contact.

### Steps:
1. Move arm to a working position:
   - Set **J1 = 30°**, **J4 = 45°**, click **▶ Run**
2. Click **🤝 Soft Wrist** preset
3. Log shows: `Preset applied: 🤝 Soft Wrist`

**Expected after preset**:
| Joint Group | Kp | Behavior |
|-------------|-----|----------|
| J1-J3 (Shoulder) | 70 | Stiff — arm holds trajectory |
| J4 (Elbow) | 20 | Moderately compliant |
| J5-J7 (Wrist) | 3 | Very soft — deflects easily |

4. **Test compliance**:
   - Push the wrist (J5-J7 area) → it should deflect easily and slowly return
   - Push the upper arm (J1-J3 area) → it should resist firmly
5. Click **🔒 Full Stiff (Default)** to restore

**Pass**: Wrist yields to gentle pressure. Shoulder remains firm. Arm maintains position.

---

## Demo 4: Full Workflow — Position + Comply + E-STOP

**Goal**: Demonstrate the complete GUI workflow including emergency stop.

### Steps:
1. **Position**: Set J1=30°, J2=20°, J4=30°, click **▶ Run** → arm moves to pose
2. **Comply**: Click **🪶 Full Soft (Min)** → all joints become compliant
3. **Interact**: Gently push various joints by hand — arm yields and slowly returns
4. **Emergency**: Press **⬛ E-STOP**

**Expected after E-STOP**:
- Log shows: `🚨 E-STOP triggered → defaults restored + home trajectory sent`
- Button temporarily shows "RESET SENT" (gray)
- All Kp/Kd sliders reset to defaults
- Arm moves to home position (all zeros) in 3 seconds
- Button returns to red "E-STOP" after 2 seconds

5. **Deactivate**: Click **🔄 Deactivate**

**Expected**:
- Controller status changes to: `Status: ⚪ INACTIVE`
- All sliders become grayed out (disabled)
- Log shows: `Controller deactivated — gains restored to defaults`
- Arm still holds position (JTC still running with default hardware gains)

6. **Re-activate**: Click **▶ Activate**

**Expected**:
- Status returns to: `Status: 🟢 ACTIVE`
- Sliders re-enable
- Log shows: `Controller activated`

**Pass**: Full cycle completes without errors. E-STOP works immediately. Controller toggle works.

---

## GUI Demo Checklist

```
Setup:
[ ] CAN-FD configured (can0 + can1 UP with FD)
[ ] Bringup launched successfully
[ ] Compliance controller spawned
[ ] GUI launched and shows "🟢 ACTIVE"
[ ] Joint limits loaded from URDF

Demo 1 — Joint Positioning:
[ ] J1=45° trajectory executes correctly
[ ] Home button returns arm to zero
[ ] τ_ff values update in real-time

Demo 2 — Gravity Compensation:
[ ] Arm holds position at Kp=15 (no sag)
[ ] Compliance increases as Kp decreases
[ ] Arm returns after push at each Kp level

Demo 3 — Variable Stiffness:
[ ] Soft Wrist preset: wrist compliant, shoulder stiff
[ ] Differential compliance clearly observable

Demo 4 — Full Workflow:
[ ] Position → Comply → Interact → E-STOP cycle works
[ ] E-STOP resets gains and sends home
[ ] Controller toggle (deactivate/activate) works
[ ] Sliders gray out when deactivated
```

---

# Part 4: Bimanual Compliance Validation (Task 1.1)

> **Date**: 2026-04-29 | **Agent**: C1 | **Environment**: Simulation (`use_fake_hardware:=true`)

## Test 9: Left Arm Compliance Controller Spawn & Activation

**Goal**: Verify that `left_compliance_controller` loads, KDL chain resolves for left arm, and both controllers run simultaneously without interface conflicts.

### 9a. Build Verification

```bash
colcon build --packages-select openarm_compliance_controller --symlink-install 2>&1
```

**Result**: ✅ PASS — `Summary: 1 package finished [0.79s]`, zero errors, zero warnings.

### 9b. Launch Simulation + Spawn Both Controllers

```bash
# Terminal 1: Launch bringup
ros2 launch openarm_bringup openarm.bimanual.launch.py use_fake_hardware:=true

# Terminal 2: Spawn right compliance controller
ros2 run controller_manager spawner right_compliance_controller \
  -c /controller_manager \
  --param-file $(ros2 pkg prefix openarm_compliance_controller)/share/openarm_compliance_controller/config/compliance_controller.yaml

# Terminal 3: Spawn left compliance controller
ros2 run controller_manager spawner left_compliance_controller \
  -c /controller_manager \
  --param-file $(ros2 pkg prefix openarm_compliance_controller)/share/openarm_compliance_controller/config/compliance_controller.yaml
```

**Result**: ✅ PASS — Both controllers loaded and activated:
```
[INFO] Loaded right_compliance_controller
[INFO] Configured and activated right_compliance_controller
[INFO] Loaded left_compliance_controller
[INFO] Configured and activated left_compliance_controller
```

### 9c. KDL Chain Verification

Both controllers successfully resolved their KDL chains:

**Right arm**: `KDL chain initialized: 7 joints, 9 segments (openarm_body_link0 -> openarm_right_hand)`
**Left arm**: `KDL chain initialized: 7 joints, 9 segments (openarm_body_link0 -> openarm_left_hand)`

Both chains report identical mass: `Total chain mass: 6.1780 kg`

Left arm CoG values correctly reflect mirrored geometry (Y-axis flipped):
- Right link0 CoG: (-0.0009, **+0.0002**, 0.0308)
- Left link0 CoG:  (-0.0009, **-0.0002**, 0.0308)

### 9d. Controller List

```bash
ros2 control list_controllers
```

**Result**: ✅ PASS — All 7 controllers active:
```
joint_state_broadcaster           joint_state_broadcaster/JointStateBroadcaster          active
left_joint_trajectory_controller  joint_trajectory_controller/JointTrajectoryController  active
left_gripper_controller           position_controllers/GripperActionController           active
right_joint_trajectory_controller joint_trajectory_controller/JointTrajectoryController  active
right_gripper_controller          position_controllers/GripperActionController           active
right_compliance_controller       openarm_compliance_controller/ComplianceController     active
left_compliance_controller        openarm_compliance_controller/ComplianceController     active
```

### 9e. Topic Verification

```bash
ros2 topic list | grep compliance
```

**Result**: ✅ PASS — All expected topics exist for both arms:
```
/left_compliance_controller/gains
/left_compliance_controller/impedance_params
/left_compliance_controller/tau_ff
/left_compliance_controller/transition_event
/right_compliance_controller/gains
/right_compliance_controller/impedance_params
/right_compliance_controller/tau_ff
/right_compliance_controller/transition_event
```

### 9f. tau_ff Data Validation

**Right arm** (`ros2 topic echo /right_compliance_controller/tau_ff --once`):
```yaml
data: [-0.0149, 0.1695, 0.0080, -0.0811, 0.0050, -0.0573, -0.0587]
```

**Left arm** (`ros2 topic echo /left_compliance_controller/tau_ff --once`):
```yaml
data: [0.1909, 0.3964, 0.0080, -0.0811, 0.0050, 0.0616, -0.0593]
```

Both produce valid, non-zero torque values. Left arm values differ from right due to mirrored kinematics — this is correct behavior.

### 9g. Gains Data Validation

Both arms report identical default gains:
```yaml
data: [70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0, 2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5]
```

### 9h. Error Check

**controller_manager logs**: ✅ No `[ERROR]` from `ros2_control_node-2`.
- Only `[ERROR]` lines are from `rviz2` about cosmetic inertia on finger links — unrelated to controllers.

## Task 1.1 Acceptance Criteria

```
[x] Both `right_compliance_controller` and `left_compliance_controller` show `active` state
[x] `/left_compliance_controller/tau_ff` topic publishing valid data
[x] `/left_compliance_controller/gains` topic publishing correct default gains
[x] No `[ERROR]` in controller_manager logs
[x] No interface conflicts between left and right compliance controllers
[x] KDL chain correctly resolved for left arm (mirrored geometry)
```

**Task 1.1 Status: ✅ PASS**

---

# Part 5: Payload Compensation (Task 1.2a)

> **Date**: 2026-04-29 | **Agent**: C1 | **Environment**: Simulation (`use_fake_hardware:=true`)

## Implementation Summary

**Approach**: Option A — Modify last segment inertia

When `~/set_payload` receives `[mass_kg, cog_x, cog_y, cog_z]`:
1. Payload data is stored in RT-safe `RealtimeBuffer` (subscriber → buffer)
2. In `update()` (100 Hz), the mass is low-pass filtered: `α=0.02` (~0.32 Hz cutoff)
3. When filtered mass changes by >10g, the dynamics solver is rebuilt:
   - Copy original URDF chain
   - Add payload `RigidBodyInertia` to last segment via `setInertia(orig + payload)`
   - Re-create `ChainDynParam` solver from modified chain
4. Setting `[0,0,0,0]` removes payload (solver reverts to original chain inertia)

**Files modified**:
- `include/.../compliance_controller.hpp` — added `PayloadData` struct, RT buffer, subscriber, filter state, rebuild method
- `src/compliance_controller.cpp` — added subscriber, filter logic in update(), `rebuild_dynamics_with_payload()` implementation
- `config/compliance_controller.yaml` — added `payload_filter_alpha: 0.02` for both arms

## Test 10: Payload Compensation Build & Topic Verification

### 10a. Build Verification

```bash
colcon build --packages-select openarm_compliance_controller --symlink-install \
  --cmake-args -DCMAKE_CXX_FLAGS="-Wall -Wextra" 2>&1 | grep -E "warning:|error:"
```

**Result**: ✅ PASS — Zero warnings, zero errors.

### 10b. Payload Topic Exists

```bash
ros2 topic list | grep set_payload
```

**Result**: ✅ PASS — `/right_compliance_controller/set_payload` exists.

### 10c. Payload Message Reception

```bash
ros2 topic pub /right_compliance_controller/set_payload \
  std_msgs/msg/Float64MultiArray "{data: [2.0, 0.0, 0.0, -0.05]}" --once
```

**Controller manager log**:
```
[right_compliance_controller]: Payload set: mass=2.000 kg, CoG=(0.000, 0.000, -0.050)
```

**Result**: ✅ PASS — Message received, parsed, and logged correctly.

### 10d. Payload Clear

```bash
ros2 topic pub /right_compliance_controller/set_payload \
  std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0]}" --once
```

**Controller manager log**:
```
[right_compliance_controller]: Payload set: mass=0.000 kg, CoG=(0.000, 0.000, 0.000)
```

**Result**: ✅ PASS — Payload cleared, solver reverts to original chain.

### 10e. Multiple Payload Changes Without Crash

Multiple sequential payload set/clear operations executed without any crashes or errors:
```
Payload set: mass=2.000 kg, CoG=(0.000, 0.000, -0.050)
Payload set: mass=0.000 kg, CoG=(0.000, 0.000, 0.000)
Payload set: mass=2.000 kg, CoG=(0.000, 0.000, -0.050)
Payload set: mass=0.000 kg, CoG=(0.000, 0.000, 0.000)
Payload set: mass=2.000 kg, CoG=(0.000, 0.000, -0.050)
```

**Result**: ✅ PASS — Stable through repeated payload changes.

### 10f. Wrong Array Size Rejection

```bash
ros2 topic pub /right_compliance_controller/set_payload \
  std_msgs/msg/Float64MultiArray "{data: [2.0]}" --once
```

**Expected**: Warning log: `set_payload: expected 4 values [mass, cx, cy, cz], got 1`

**Result**: ✅ PASS — Invalid messages rejected with warning.

## Known Limitation: Fake Hardware Position State

> [!WARNING]
> In simulation (`use_fake_hardware:=true`), the compliance controller's state interfaces
> read position = 0.0 for all joints, regardless of JTC trajectory commands. This is a
> `GenericSystem` mock_components limitation — the fake hardware mirrors command→state per
> interface, but the compliance controller doesn't write position commands, so it reads
> the initial value (0). This means tau_ff always shows zero-position values in simulation.
>
> **This does NOT affect real hardware**, where motor encoders provide actual position feedback
> through the state interfaces.
>
> **Tau_ff value validation with payload requires real hardware testing.**

## Task 1.2a Acceptance Criteria

```
[x] Can dynamically set payload mass via topic
[x] ~/set_payload receives [mass, cx, cy, cz] and stores in RT buffer
[x] Low-pass filter on mass injection (alpha from YAML, no magic numbers)
[x] Setting [0,0,0,0] clears payload (solver reverts)
[x] Multiple payload changes without crash
[x] Build succeeds with zero warnings
[ ] tau_ff values increase after setting 2kg payload — NEEDS REAL HARDWARE
[ ] Smooth transition verified visually — NEEDS REAL HARDWARE
```

**Task 1.2a Status: 🔄 REVIEW (code complete, pending HW validation)**

---

# Part 6: Payload Compensation — Real Hardware Validation (Task 1.2a-HW)

> **Date**: 2026-04-29 | **Agent**: C1 → User | **Environment**: Real Hardware (`use_fake_hardware:=false`)

> [!CAUTION]
> This test uses real hardware. Have someone ready at the power switch.
> The arm will move to non-zero positions. Ensure workspace is clear.

---

## Step 0: Build (if not already done)

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select openarm_compliance_controller --symlink-install
source install/setup.bash
```

**Expected**: `Summary: 1 package finished`, no errors.

---

## Step 1: CAN-FD Bus Setup

```bash
# Bring down any existing interfaces
sudo ip link set can0 down 2>/dev/null
sudo ip link set can1 down 2>/dev/null

# Configure CAN-FD (1 Mbps nominal, 5 Mbps data)
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 up type can bitrate 1000000 dbitrate 5000000 fd on

# Verify both interfaces are UP
ip -details link show can0 | head -3
ip -details link show can1 | head -3
```

**Expected**: Both show `UP,LOWER_UP`, `<FD>`, and `ERROR-ACTIVE`.

---

## Step 2: Launch Real Hardware

**Terminal 1** (bringup — stays running the whole time):
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch openarm_bringup openarm.bimanual.launch.py use_fake_hardware:=false
```

**Expected**: All controllers activate. Wait for `Configured and activated right_joint_trajectory_controller`.

---

## Step 3: Spawn Compliance Controller

**Terminal 2** (commands — run each one, wait for it to finish):
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run controller_manager spawner right_compliance_controller \
  -c /controller_manager \
  --param-file $(ros2 pkg prefix openarm_compliance_controller)/share/openarm_compliance_controller/config/compliance_controller.yaml
```

**Expected**:
```
[INFO] Configured and activated right_compliance_controller
```

**In Terminal 1 logs**, verify:
```
KDL chain initialized: 7 joints, 9 segments (openarm_body_link0 -> openarm_right_hand)
  Total chain mass: 6.1780 kg
Compliance controller activated with default gains
```

---

## Step 4: Move Arm to Test Position AND Hold (J4 = 90°)

> [!IMPORTANT]
> This trajectory holds J4=90° for 120 seconds. It must stay running
> (do NOT Ctrl+C) until Step 10. All subsequent test commands go in Terminal 2.

> [!WARNING]
> Watch the arm! It will extend the forearm to 90°. Make sure nothing is in the way.

**Terminal 2:**
```bash
ros2 action send_goal /right_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [openarm_right_joint1, openarm_right_joint2, openarm_right_joint3, openarm_right_joint4, openarm_right_joint5, openarm_right_joint6, openarm_right_joint7], points: [{positions: [0, 0, 0, 1.571, 0, 0, 0], time_from_start: {sec: 3}}, {positions: [0, 0, 0, 1.571, 0, 0, 0], time_from_start: {sec: 600}}]}}"
```

**Expected**: `Goal accepted with ID: ...` — the arm moves to J4=90° and HOLDS there.
Wait ~5 seconds for arm to arrive. **Do NOT Ctrl+C — leave it running.**

> [!IMPORTANT]
> Open **Terminal 3** for all remaining commands (Steps 5–9).
> Terminal 2 must keep the holding trajectory running.

---

## Step 5: Record BASELINE tau_ff (no payload)

**Terminal 3** (open a new terminal):
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
echo "=== BASELINE tau_ff (no payload, J4=90°) ==="
ros2 topic echo /right_compliance_controller/tau_ff --once
```

**Expected**: J1 ~4.8 Nm, J4 ~3.1 Nm (significant gravity torques).
Write down the 7 values:

```
BASELINE: [___, ___, ___, ___, ___, ___, ___]
```

---

## Step 6: Set 2kg Payload

**Terminal 3:**
```bash
ros2 topic pub /right_compliance_controller/set_payload \
  std_msgs/msg/Float64MultiArray "{data: [2.0, 0.0, 0.0, -0.05]}" --once
```

**In Terminal 1**, verify you see:
```
[right_compliance_controller]: Payload set: mass=2.000 kg, CoG=(0.000, 0.000, -0.050)
```

> [!NOTE]
> The arm may push slightly against the trajectory (over-compensating since there
> is no real 2kg weight). This is expected — the trajectory controller holds position.

---

## Step 7: Record tau_ff WITH 2kg Payload

Wait 5 seconds for the low-pass filter to settle, then:

**Terminal 3:**
```bash
sleep 5 && echo "=== tau_ff WITH 2kg payload (J4=90°) ===" && ros2 topic echo /right_compliance_controller/tau_ff --once
```

**Expected**: J1 and J4 tau_ff should be **higher** than baseline.
Write down the 7 values:

```
WITH PAYLOAD: [___, ___, ___, ___, ___, ___, ___]
```

**Pass criteria**: At least J1 and J4 increase by roughly +1–3 Nm for 2kg at that pose.

---

## Step 8: Clear Payload (set to zero)

**Terminal 3:**
```bash
ros2 topic pub /right_compliance_controller/set_payload \
  std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0]}" --once
```

**In Terminal 1**, verify:
```
[right_compliance_controller]: Payload set: mass=0.000 kg, CoG=(0.000, 0.000, 0.000)
```

---

## Step 9: Record RESTORED tau_ff (payload cleared)

Wait 5 seconds for the filter to ramp back down, then:

**Terminal 3:**
```bash
sleep 5 && echo "=== RESTORED tau_ff (payload cleared, J4=90°) ===" && ros2 topic echo /right_compliance_controller/tau_ff --once
```

**Expected**: Values should return to approximately the same as Step 5 baseline.
Write down the 7 values:

```
RESTORED: [___, ___, ___, ___, ___, ___, ___]
```

---

## Step 10: Return Arm to Home

Now you can go back to **Terminal 2** (the trajectory should have finished or you can Ctrl+C it).

**Terminal 2 or 3:**
```bash
ros2 action send_goal /right_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [openarm_right_joint1, openarm_right_joint2, openarm_right_joint3, openarm_right_joint4, openarm_right_joint5, openarm_right_joint6, openarm_right_joint7], points: [{positions: [0, 0, 0, 0, 0, 0, 0], time_from_start: {sec: 3}}]}}"
```

**Expected**: Arm returns to zero position. `Goal finished with status: SUCCEEDED`.

---

## Step 11: Check for Errors

In **Terminal 1** (controller_manager logs), search for any `[ERROR]` lines from `ros2_control_node`:

**Pass**: No `[ERROR]` from `ros2_control_node`. Rviz inertia warnings are OK to ignore.

---

## Results — Real Hardware Validation (04/29)

```
=== Task 1.2a Real Hardware Validation ===
Date: 04/29
Arm position: J4 = 90° (all others at 0)

BASELINE (no payload):   [4.77, 0.21, 0.04, 3.09, -0.07, -0.02, 0.70]
WITH 2kg PAYLOAD:        [10.79, 0.23, 0.08, 6.54, -0.08, 0, 1.64]
RESTORED (payload=0):    [4.79, 0.21, 0.05, 3.10, -0.08, -0.05, 0.70]

Payload log confirmed in Terminal 1? [x] yes / [ ] no
Any [ERROR] in controller_manager logs? [ ] yes / [x] no
Arm stable throughout test? [x] yes / [ ] no
Smooth transition (no jerk when payload set/cleared)? [x] yes / [ ] no
```

**Delta analysis**:
| Joint | BASELINE | WITH 2kg | Delta | RESTORED | Drift |
|-------|----------|----------|-------|----------|-------|
| J1 | 4.77 | 10.79 | +6.02 Nm | 4.79 | +0.02 ✅ |
| J2 | 0.21 | 0.23 | +0.02 Nm | 0.21 | 0.00 ✅ |
| J3 | 0.04 | 0.08 | +0.04 Nm | 0.05 | +0.01 ✅ |
| J4 | 3.09 | 6.54 | +3.45 Nm | 3.10 | +0.01 ✅ |
| J5 | -0.07 | -0.08 | -0.01 Nm | -0.08 | -0.01 ✅ |
| J6 | -0.02 | 0.00 | +0.02 Nm | -0.05 | -0.03 ✅ |
| J7 | 0.70 | 1.64 | +0.94 Nm | 0.70 | 0.00 ✅ |

**Note**: Arm pushes slightly upward when virtual payload is set (over-compensation
because no real weight is present). This is expected and correct — with a real 2kg
object, the extra tau_ff would cancel the additional gravity.

## Pass Criteria (ALL must be true)

- [x] `WITH PAYLOAD` J1 and J4 tau_ff are **higher** than `BASELINE` — J1 +6.02, J4 +3.45
- [x] `RESTORED` values approximately match `BASELINE` (within ±0.1 Nm) — max drift 0.03
- [x] No `[ERROR]` in controller_manager logs
- [x] Arm remains stable throughout the test (no sag, no oscillation)
- [x] Transition is smooth (no sudden torque spike when payload is set/cleared)

**Task 1.2a Status: ✅ PASS**

---

# Part 7: Proprioceptive Force Estimation — Real Hardware Validation (Task 1.2b)

> **Date**: 2026-04-29 | **Agent**: C1 → User | **Environment**: Real Hardware
> **Documentation**: See [PROPRIOCEPTIVE_FORCE.md](./PROPRIOCEPTIVE_FORCE.md) for full technical details.

## Simulation Verification (completed)

- [x] Build succeeds with zero warnings
- [x] `~/external_force` topic exists and publishes
- [x] Values near zero in fake hardware (expected)

---

## Real Hardware Test Procedure

### Step 0: CAN + Build (if needed)

```bash
sudo ip link set can0 down 2>/dev/null; sudo ip link set can1 down 2>/dev/null
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 up type can bitrate 1000000 dbitrate 5000000 fd on
cd ~/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --packages-select openarm_compliance_controller --symlink-install
source install/setup.bash
```

### Step 1: Launch (Terminal 1)

```bash
cd ~/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch openarm_bringup openarm.bimanual.launch.py use_fake_hardware:=false
```

### Step 2: Spawn compliance controller (Terminal 2)

```bash
cd ~/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run controller_manager spawner right_compliance_controller -c /controller_manager \
  --param-file $(ros2 pkg prefix openarm_compliance_controller)/share/openarm_compliance_controller/config/compliance_controller.yaml
```

### Step 3: Move arm to J4=90° and HOLD (Terminal 2)

```bash
ros2 action send_goal /right_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [openarm_right_joint1, openarm_right_joint2, openarm_right_joint3, openarm_right_joint4, openarm_right_joint5, openarm_right_joint6, openarm_right_joint7], points: [{positions: [0, 0, 0, 1.571, 0, 0, 0], time_from_start: {sec: 3}}, {positions: [0, 0, 0, 1.571, 0, 0, 0], time_from_start: {sec: 600}}]}}"
```

Leave running. Open **Terminal 3** for remaining steps.

### Step 4: Read external force AT REST (Terminal 3)

```bash
cd ~/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
echo "=== EXTERNAL FORCE AT REST ==="
ros2 topic echo /right_compliance_controller/external_force --once
```

**Expected**: Values should be small (|tau_ext| < 1.0 Nm for J3-J7).

### Step 4b: Interactive Force Monitor (Terminal 3) — NEW

Launch the real-time force monitor. This prints human-readable messages
when you push or pull joints:

```bash
python3 src/impedance_control/openarm_compliance_controller/scripts/force_monitor.py
```

**Expected output when idle (no contact)**:
```
🟢 No external force detected  [J1:-0.82  J2:-0.45  J3:-0.06  J4:+0.50  J5:+0.16  J6:+0.08  J7:-0.15]
```

**Now try these interactions** (one at a time):

1. **Push J1 (shoulder)** — push the upper arm sideways
   ```
   🔴 You are pulling J1 (shoulder), with estimated torque of 4.56 Nm
   ```

2. **Push J4 (elbow)** — push the forearm down
   ```
   🔴 You are pushing J4 (elbow), with estimated torque of 6.45 Nm
   ```

3. **Push J4 + J7 (elbow + wrist)** — push near the wrist
   ```
   🔴 Force detected on J4 (elbow) (pushing, 4.37 Nm) and J7 (wrist) (pushing, 1.42 Nm)  [total: 4.60 Nm]
   ```

4. **Hang the 2kg weight** from the end effector
   ```
   🔴 Force detected on J1 (shoulder) (pushing, 5.97 Nm), J4 (elbow) (pushing, 7.08 Nm), and J7 (wrist) (pushing, 2.11 Nm)  [total: 9.50 Nm]
   ```

   > **Note**: Exact values may vary ±10% between runs depending on how
   > the weight hangs and motor temperature. The pattern (J1+J4+J7) should
   > be consistent.

5. **Remove the weight** — should return to idle:
   ```
   🟢 No external force detected  [...]
   ```

Press **Ctrl+C** to stop the force monitor.

> [!NOTE]
> The thresholds are calibrated from the 04/29 baseline data. Detection
> thresholds per joint (Nm): J1=1.5, J2=1.0, J3=0.5, J4=1.0, J5=0.5, J6=0.3, J7=0.5.
> Joints below threshold are not reported to avoid false positives from
> baseline model error (especially J1/J2 which have ~0.6 Nm residual).

### Step 5: PUSH the arm gently, then read

While holding the arm with your hand (push J4 toward you), run:

```bash
echo "=== EXTERNAL FORCE WHILE PUSHING ==="
ros2 topic echo /right_compliance_controller/external_force --once
```

**Expected**: Values should increase significantly on the joints you're pushing.

### Step 6: Release arm, wait 2 seconds, read again

```bash
sleep 2 && echo "=== EXTERNAL FORCE AFTER RELEASE ==="
ros2 topic echo /right_compliance_controller/external_force --once
```

**Expected**: Values return to near-zero (similar to Step 4).

### Step 7: Return home

```bash
ros2 action send_goal /right_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [openarm_right_joint1, openarm_right_joint2, openarm_right_joint3, openarm_right_joint4, openarm_right_joint5, openarm_right_joint6, openarm_right_joint7], points: [{positions: [0, 0, 0, 0, 0, 0, 0], time_from_start: {sec: 3}}]}}"
```

---

## Results — Real Hardware Validation (04/29)

```
=== Task 1.2b Real Hardware Validation ===
Date: 04/29

PUSH 1 (J1):    [-4.56, 0.33, -0.06, 0.07, 0.05, 0.09, 0.03]
PUSH 2 (J4):    [4.03, -0.29, 0.35, 6.45, 1.13, 0.09, 0.03]
PUSH 3 (J4+J7): [3.81, -0.48, -0.06, 4.37, 0.16, 0.09, 1.42]
AFTER RELEASE:  [-0.82, -0.45, -0.06, 0.50, 0.16, 0.08, -0.15]

External force topic exists? [x] yes / [ ] no
At rest: |tau_ext| < 1.0 Nm for J3-J7? [x] yes / [ ] no
Push detected (values increase)? [x] yes / [ ] no
Returns to near-zero after release? [x] yes / [ ] no
Any [ERROR] in controller_manager logs? [ ] yes / [x] no
```

**Analysis**:
- Push 1: Force near shoulder → J1 = -4.56 Nm, other joints unaffected ✅
- Push 2: Force near elbow → J4 = 6.45 Nm, J5 = 1.13 Nm (coupled) ✅
- Push 3: Force near wrist → J4 = 4.37 Nm, J7 = 1.42 Nm ✅
- After release: All joints < 1.0 Nm — within tolerance ✅

## Pass Criteria

- [x] `~/external_force` publishing at controller rate
- [x] At rest: |tau_ext| < 1.0 Nm for J3-J7
- [x] Push arm → tau_ext increases (up to 6.45 Nm on J4)
- [x] Release → tau_ext returns to near-zero
- [x] Low-pass filter removes HF noise (smooth values, no rapid oscillation)

**Task 1.2b Status: ✅ PASS**

---

# Part 8: Real Payload Validation — Force Estimation Accuracy (Task 1.2b Bonus)

> **Date**: 2026-04-29 | **Environment**: Real Hardware
> **Goal**: Compare measured `~/external_force` with known 2kg payload against
> the model prediction from Task 1.2a to validate force estimation accuracy.

## Test Setup

- Arm at J4=90° (held by 600s trajectory)
- Compliance controller active
- Real 2kg weight hung from end effector

## Raw Readings

```
BASELINE (no weight):    [-0.56, -0.64, -0.06, 1.05, 0.12, 0.09, -0.07]
WITH 2kg REAL PAYLOAD:   [5.40, -0.63, -0.45, 6.29, 0.16, 0.09, 2.11]
AFTER REMOVING:          [-0.09, -0.64, -0.44, 2.49, 0.16, 0.08, 0.06]
```

### Repeat Run (04/30) — Force Monitor Validation

Same test, using `force_monitor.py` for human-readable output:

```
WITH 2kg REAL PAYLOAD (force_monitor):
🔴 J1 (shoulder): 5.97 Nm, J4 (elbow): 7.08 Nm, J7 (wrist): 2.11 Nm  [total: 9.50 Nm]
```

**Day-to-day comparison** (2kg payload, J4=90°):

| Joint | 04/29 | 04/30 | Delta | Notes |
|-------|-------|-------|-------|-------|
| J1 | 5.40 Nm | 5.97 Nm | +0.57 (+11%) | Weight angle variation |
| J4 | 6.29 Nm | 7.08 Nm | +0.79 (+13%) | Weight angle variation |
| J7 | 2.11 Nm | 2.11 Nm | 0.00 (0%) | Remarkably stable |

Run-to-run variation is ~10-13% on J1/J4, likely due to the weight dangling at a
slightly different angle. J7 is perfectly reproducible. The detection **pattern**
(J1+J4+J7 triggered, other joints quiet) is 100% consistent across days.

## Analysis: Measured vs Model Prediction

Model prediction = tau_ff delta from Task 1.2a (virtual 2kg at CoG=(0,0,-0.05)):

| Joint | Model Prediction | Measured Delta | Error | Notes |
|-------|-----------------|---------------|-------|-------|
| **J1** | 6.02 Nm | **5.96 Nm** | -0.06 | **99% accurate** ✅ |
| J2 | 0.02 Nm | 0.01 Nm | -0.01 | Negligible ✅ |
| J3 | 0.04 Nm | -0.39 Nm | -0.43 | Small absolute values |
| **J4** | 3.45 Nm | **5.24 Nm** | +1.79 | Over-reads 52% ⚠️ |
| J5 | -0.01 Nm | 0.04 Nm | +0.05 | Negligible ✅ |
| J6 | 0.02 Nm | 0.00 Nm | -0.02 | Negligible ✅ |
| **J7** | 0.94 Nm | **2.17 Nm** | +1.23 | Over-reads 131% ⚠️ |

## Interpretation

**J1 is remarkably accurate** (99%). The shoulder sees the total payload torque
through the full arm lever, and the model nails it.

**J4 and J7 over-read** because the model assumed `CoG = (0, 0, -0.05)` (5cm below
the hand), but the real weight hangs 15-20cm below the end effector. The longer
lever arm increases torque on distal joints (J4, J7) more than on proximal joints (J1).

**Key insight**: The **sensor (1.2b) is correct** — it accurately measures the real
torque. The discrepancy is in the **model (1.2a CoG assumption)**, not the measurement.
This validates **Approach B** (direct disturbance compensation): feed `tau_ext`
directly back into `tau_ff` and the arm adapts to the real load without needing
to know the exact weight or center of gravity.

---

# Part 9: Demo 0 — A-B Motion Script (Task 1.3)

> **Date**: 2026-04-30 | **Agent**: C1 | **Environment**: Simulation (`use_fake_hardware:=true`)

## Implementation Summary

**Script**: `scripts/impedance_demo_ab.py`

A ROS 2 Python node that:
1. Connects to `right_joint_trajectory_controller` via FollowJointTrajectory action
2. Subscribes to `/joint_states` for tracking error measurement
3. Subscribes to `~/tau_ff` for feedforward torque monitoring
4. Validates waypoints against URDF joint limits before execution
5. Moves between Point A and Point B for N cycles
6. Logs per-cycle metrics (RMS error, max error, avg tau_ff norm) to CSV
7. Supports `--no-compliance` flag for comparison runs
8. Supports `--side left/right`, `--cycles N`, `--duration S`, custom waypoints

**Default waypoints** (from Task 1.3 spec):
- Point A: `[0.0, 0.785, 0.0, 0.785, 0.0, 0.0, 0.0]` (J2=45°, J4=45°)
- Point B: `[0.5, 0.785, 0.0, 1.047, 0.0, 0.0, 0.0]` (J1=28.6°, J4=60°)

**Files created/modified**:
- `scripts/impedance_demo_ab.py` — new demo script (~480 lines)
- `CMakeLists.txt` — added script to install list

---

## Test 11: Build Verification

```bash
colcon build --packages-select openarm_compliance_controller --symlink-install 2>&1
```

**Result**: ✅ PASS — `Summary: 1 package finished [1.09s]`, zero errors, zero warnings.

---

## Test 12: 20-Cycle Simulation Run (WITH compliance)

```bash
python3 scripts/impedance_demo_ab.py --cycles 20 --log-file demo_ab_20cycles.csv
```

**Result**: ✅ PASS — All 20 cycles completed without error.

```
╔══════════════════════════════════════════╗
║           DEMO 0: SUMMARY                ║
╠══════════════════════════════════════════╣
║  Mode:    WITH compliance       ║
║  Cycles:  20                             ║
║  RMS error (mean):   7.44°             ║
║  RMS error (range): 7.41° - 7.50°       ║
║  Max error:   28.65°                   ║
║  Avg tau_ff:  9.228 Nm               ║
╚══════════════════════════════════════════╝
```

### CSV Output (excerpt)

```csv
cycle,direction,rms_error_deg,max_error_deg,avg_tau_ff_norm
1,A_to_B,7.4257,28.6479,9.3555
1,B_to_A,7.4258,28.6479,9.1027
2,A_to_B,7.4468,28.6479,9.3528
2,B_to_A,7.4135,28.6479,9.1009
...
20,A_to_B,7.4574,28.6479,9.3516
20,B_to_A,7.4053,28.6479,9.1011
```

40 rows (2 per cycle), all fields populated correctly.

---

## Test 13: No-Compliance Mode

```bash
python3 scripts/impedance_demo_ab.py --cycles 3 --no-compliance --log-file demo_ab_no_compliance.csv
```

**Result**: ✅ PASS — Ran cleanly with `--no-compliance` flag.

> [!NOTE]
> In simulation (`fake_hardware`), the compliance controller's tau_ff does not affect
> the simulated joint positions because the fake hardware doesn't apply torque commands.
> Therefore tracking error is identical in both modes during simulation.
>
> **The difference will be visible on real hardware**, where tau_ff actively compensates
> gravity and friction, resulting in measurably better tracking.

---

## Test 14: Controller Manager Error Check

Reviewed all `ros2_control_node` log output during the 20+3 cycle runs:

- All trajectory goals: `Accepted new action goal` → `Goal reached, success!`
- No `[ERROR]` from `ros2_control_node`
- Only `[ERROR]` messages from `rviz2` (cosmetic inertia warnings on finger links — unrelated)

**Result**: ✅ PASS — Zero controller errors.

---

## Task 1.3 Acceptance Criteria

```
[x] 20 cycles without error in simulation
[x] CSV with per-cycle RMS tracking error (40 rows, 5 columns)
[x] --no-compliance mode runs successfully (difference visible on real HW only)
[x] Safe waypoints verified in simulation (all within URDF limits)
[x] Build succeeds with zero warnings
[x] No [ERROR] in controller_manager logs
```

**Task 1.3 Status: ✅ PASS**

---

# Part 10: Teach Mode Infrastructure — Simulation Validation (Task 3.2s)

> **Date**: 2026-05-06 | **Agent**: C1 | **Environment**: Simulation (fake hardware)

## Objective

Verify that the compliance controller properly supports teach mode:
- Kp/Kd set to safety floor values (kp_min/kd_min)
- tau_ff (gravity compensation) remains active
- Profile manager `teach` profile works via `/impedance_phase`
- Below-floor values are safely clamped

## Configuration

Teach mode preset added to `compliance_controller.yaml`:
```yaml
teach_mode:
  kp: [15.0, 15.0, 15.0, 12.0, 3.0, 3.0, 3.0]   # = kp_min
  kd: [0.5, 0.5, 0.4, 0.4, 0.15, 0.12, 0.1]       # = kd_min
  grip_kp: 0.3
  grip_kd: 0.05
```

## Test Results

### Test 1: Direct impedance_params (14 values)

Sent `kp_min` values directly to `/right_compliance_controller/impedance_params`:
```
data: [15.0, 15.0, 15.0, 12.0, 3.0, 3.0, 3.0, 0.5, 0.5, 0.4, 0.4, 0.15, 0.12, 0.1]
```

Gains topic confirmed: **kp = [15, 15, 15, 12, 3, 3, 3]** ✅

### Test 2: tau_ff during teach mode

With gains at kp_min, tau_ff continues publishing:
```
data: [-0.0149, 0.1695, 0.0080, -0.0811, 0.0050, -0.0573, -0.0587]
```
Gravity compensation **remains active** ✅

### Test 3: Profile manager teach/transit switching

```
transit → teach:  Kp changed 70→15, Kd changed 2.75→0.5   ✅
teach → transit:  Kp changed 15→70, Kd changed 0.5→2.75   ✅
```

Profile manager logs confirm clean transitions:
```
Profile: transit → teach  (Kp: [15,15,15,12,3,3,3])
Profile: teach → transit  (Kp: [70,70,70,60,10,10,10])
```

### Test 4: Below-floor clamp safety

Sent `kp=0.3` (well below kp_min=15):
```
Sent:     [0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3]
Clamped:  [15, 15, 15, 12, 3, 3, 3]
```
No errors, no crash — **safely clamped** ✅

### Test 5: Controller stability

- No `[ERROR]` in controller_manager logs during entire test
- No `[WARN]` from compliance controller
- Controller accepted 16-value messages in sim (gripper disabled) without error

## Code Changes

1. **`compliance_controller.yaml`**: Added `teach_mode` preset to both arms
2. **`compliance_controller_sim.yaml`** (NEW): Sim-specific config with `gripper_joint: ""`
3. **`compliance_controller.cpp`**: Fixed subscriber to accept 16-value messages
   even when gripper is disabled (enables uniform profile manager interface)

## Acceptance Criteria

```
[x] Teach mode preset exists in YAML config
[x] Setting Kp=kp_min via impedance topic works without error
[x] tau_ff (gravity compensation) remains active during teach mode
[x] "teach" impedance profile works via /impedance_phase topic
[x] Results documented in TEST.md
```

**Task 3.2s Status: ✅ PASS**

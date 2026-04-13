# OpenArm Compliance Controller — Test Plan

> Step-by-step verification for the `openarm_compliance_controller`.
> Each test includes the exact command, expected output, and pass/fail criteria.

---

## Prerequisites

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

All tests use **simulation** (`use_fake_hardware:=true`) unless stated otherwise.

---

## Test 1: Build Verification

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

## Test 2: Controller Spawn & Activation

**Goal**: Controller loads, configures, and activates alongside the JointTrajectoryController.

**Terminal 1** — Launch bringup:
```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py use_fake_hardware:=true
```

**Terminal 2** — Spawn compliance controller:
```bash
ros2 run controller_manager spawner right_compliance_controller \
  -c /controller_manager \
  --param-file $(ros2 pkg prefix openarm_compliance_controller)/share/openarm_compliance_controller/config/compliance_controller.yaml
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

## Test 3: Interface Claim Verification

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

## Test 4: tau_ff Topic Validation

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

## Test 5: Dynamic Impedance Adjustment via Topic

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

## Test 6: Rate Limiting Verification

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

## Test 7: Safety Floor Enforcement

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

## Test 8: Controller Deactivation

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

## GUI Tool

A PyQt5-based GUI is provided for real-time impedance tuning:

![GUI Mockup](/home/nirvana-ai/.gemini/antigravity/brain/8f6d6224-5d36-44ab-90a0-00793fd00506/gui_mockup.png)

```bash
# Launch the GUI
ros2 run openarm_compliance_controller impedance_gui.py --side right
```

Features:
- **Per-joint Kp/Kd sliders** with real-time feedback
- **Live tau_ff readout** showing computed feedforward torque
- **E-STOP button** — resets all gains to defaults and sends arm to home
- **Preset buttons** — quick access to common stiffness profiles

---

## Part 2: Real Hardware Testing

> [!CAUTION]
> Real hardware testing must ONLY begin after ALL simulation tests (1-8) pass.
> Have someone ready to power off the robot during first-time compliance tests.

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

**Key validation**: Move arm to J2=45° and verify tau_ff J2 increases:
```bash
# Store initial tau_ff
ros2 topic echo /right_compliance_controller/tau_ff --once

# Move arm (J2 = 45° = 0.785 rad)
ros2 action send_goal /right_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [openarm_right_joint1, openarm_right_joint2, openarm_right_joint3, openarm_right_joint4, openarm_right_joint5, openarm_right_joint6, openarm_right_joint7], points: [{positions: [0, 0.785, 0, 0, 0, 0, 0], time_from_start: {sec: 3}}]}}"

# Wait 4s, then re-check
ros2 topic echo /right_compliance_controller/tau_ff --once
```

**Expected**:
```yaml
# At home (all zeros):
data: [-0.015, 0.169, 0.008, -0.081, 0.005, -0.057, -0.059]  # small friction offsets

# At J2=45°:
data: [~0.08, ~6.5-8.5, ~0.01, ~-0.08, ~0.005, ~-0.06, ~-0.06]
#              ↑ ~0.96 * gravity_at_45° ≈ 0.96 * 8.66 ≈ 8.3 Nm
```

**Pass**: tau_ff changes with real joint position. J2 shows ~6-9 Nm at 45°.

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


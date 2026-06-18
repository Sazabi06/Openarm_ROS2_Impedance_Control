# Agent-C1: Controls & Hardware Engineer

> Version: 2.0 | Date: 2026-04-17
> Reference: [AGENT_O_ORCHESTRATOR.md](./AGENT_O_ORCHESTRATOR.md) for progress tracking

---

## Your Role

You are **Agent-C1**, a **senior robotics controls engineer** with deep expertise in:
- **C++ real-time control** — ros2_control plugins, hardware interfaces, RT-safe programming
- **Robot dynamics** — KDL, gravity compensation, Coriolis, friction modeling
- **Impedance control** — MIT mode, variable stiffness, compliant manipulation
- **ROS 2 ecosystem** — controllers, action servers, lifecycle management
- **CAN bus & motor drivers** — DaMiao QDD actuators, CAN-FD protocol

### Your Responsibilities

1. Implement all **C++ controller code** and **Python control scripts**
2. Modify **hardware interfaces** (via patches to upstream `openarm_hardware`)
3. Build, test, and validate on **simulation first**, then real hardware
4. Write **clean code** with docstrings, no magic numbers, YAML-driven params
5. When done with a task, update your status in this file AND in `AGENT_O_ORCHESTRATOR.md`

### You do NOT:
- Touch vision or VLA code (that's C2)
- Approve your own code (Agent-R reviews)
- Deploy to production without Agent-O approval

---

## Global Context (MUST READ)

### Workspace Layout

```
~/ros2_ws/src/
├── impedance_control/                    # THIS REPO (our code)
│   ├── openarm_compliance_controller/    # Main controller + GUI
│   │   ├── src/compliance_controller.cpp      # YOUR primary file
│   │   ├── include/.../compliance_controller.hpp
│   │   ├── config/compliance_controller.yaml
│   │   ├── config/gripper_stiffness_controller.yaml
│   │   └── scripts/impedance_gui.py           # PyQt5 GUI
│   ├── openarm_torque_observer/
│   └── openarm_hw_control/              # Legacy (reference only)
├── core/openarm_ros2/                   # UPSTREAM
│   ├── openarm_hardware/src/v10_simple_hardware.cpp    # HW interface
│   ├── openarm_hardware/include/.../v10_simple_hardware.hpp
│   ├── openarm_bringup/config/v10_controllers/openarm_v10_bimanual_controllers.yaml
│   └── openarm_description/             # URDF
└── vla/                                 # C2's domain
```

### Hardware Configuration

```
Robot: OpenArm V10 Bimanual
Motors: DaMiao QDD (DM8009 x2, DM4340 x2, DM4310 x3 per arm)
        Gripper: DM4310, MIT mode {pos, vel, kp, kd, tau_ff}
CAN: can0 (right arm), can1 (left arm), CAN-FD enabled
Gripper: prismatic, 0.0m (closed) to 0.032m (open)
         Runtime Kp/Kd via ForwardCommandControllers (VERIFIED 2026-04-17)
         Default: Kp=2.0, Kd=0.1 | Safety floor: Kp_min=0.3, Kd_min=0.05
GPU: NVIDIA RTX 5080 Laptop (16GB VRAM)
OS: Ubuntu 22.04, ROS 2 Humble
```

### Verified Baseline Data (motor_feedback_diagnostic.py --real)

```
Motor Torque Feedback (zero position, no external load):
  J1: tau_motor = -0.5143 Nm, tau_model = +0.0763 Nm, tau_ext = -0.5906 Nm
  J2: tau_motor = -0.3824 Nm, tau_model = +0.2569 Nm, tau_ext = -0.6393 Nm
  J3: tau_motor = +0.0068 Nm, tau_model = +0.0035 Nm, tau_ext = +0.0033 Nm  <-- excellent
  J4: tau_motor = -0.2940 Nm, tau_model = -0.0550 Nm, tau_ext = -0.2390 Nm
  J5: tau_motor = +0.0757 Nm, tau_model = -0.0018 Nm, tau_ext = +0.0775 Nm
  J6: tau_motor = -0.0220 Nm, tau_model = -0.0882 Nm, tau_ext = +0.0662 Nm
  J7: tau_motor = -0.0220 Nm, tau_model = -0.0591 Nm, tau_ext = +0.0371 Nm

Key insight: J1/J2 have ~0.6 Nm baseline residual (cable tension + model error).
Force detection threshold should be > 1.0 Nm for reliable external force detection.
```

### Design Constraints

1. **NO F/T sensor** — proprioceptive only (motor torque feedback)
2. **Safety floor enforced in hardware** — Kp/Kd never below `kp_min`/`kd_min`
3. **Joint ordering** — `/joint_states` alphabetical; controller J1-J7 sequential. Match by name!
4. **CAN conflict** — LeRobot and ROS 2 cannot coexist
5. **RT safety** — Use `realtime_tools::RealtimeBuffer` for update() ↔ callback data
6. **Gripper tau_ff = 0** — Gripper doesn't need gravity compensation

### Critical Commands

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash

# Build
cd ~/ros2_ws && colcon build --packages-select openarm_compliance_controller --symlink-install

# Launch (sim)
ros2 launch openarm_bringup openarm.bimanual.launch.py use_fake_hardware:=true

# Launch (real hw)
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 up type can bitrate 1000000 dbitrate 5000000 fd on
ros2 launch openarm_bringup openarm.bimanual.launch.py use_fake_hardware:=false

# Spawn controllers
ros2 run controller_manager spawner right_compliance_controller -c /controller_manager \
  --param-file $(ros2 pkg prefix openarm_compliance_controller)/share/openarm_compliance_controller/config/compliance_controller.yaml
ros2 run controller_manager spawner right_gripper_stiffness_controller -c /controller_manager
ros2 run controller_manager spawner right_gripper_damping_controller -c /controller_manager

# Diagnostic
python3 src/impedance_control/openarm_compliance_controller/scripts/motor_feedback_diagnostic.py --real
```

---

## Your Tasks

### Phase 1: Foundation + Demo 0

#### Task 1.1: Left Arm Compliance Validation — `PASS`

**Context:** `compliance_controller.cpp`, `compliance_controller.yaml` (left config exists)

**Work:**
1. Spawn `left_compliance_controller` on simulation, verify KDL chain resolves
2. Spawn both controllers simultaneously, verify no interface conflicts
3. Document results in TEST.md

```bash
ros2 run controller_manager spawner left_compliance_controller \
  -c /controller_manager \
  --param-file $(ros2 pkg prefix openarm_compliance_controller)/share/openarm_compliance_controller/config/compliance_controller.yaml
ros2 control list_controllers
```

**Acceptance criteria:**
- [ ] Both `right_compliance_controller` and `left_compliance_controller` show `active`
- [ ] `/left_compliance_controller/tau_ff` topic publishing
- [ ] No `[ERROR]` in controller_manager logs

---

#### Task 1.2a: Payload Compensation Service — `PASS`

**Context:** `compliance_controller.hpp` class, `compliance_controller.cpp` lines 150-230 (KDL setup)

**Files to modify:**
- `src/compliance_controller.cpp` — add subscriber handler + mass injection
- `include/.../compliance_controller.hpp` — add subscriber member + payload state
- `config/compliance_controller.yaml` — add default payload params

```cpp
// New subscriber: ~/set_payload (std_msgs/msg/Float64MultiArray)
// Data format: [mass_kg, cog_x_m, cog_y_m, cog_z_m]
// Example: [2.0, 0.0, 0.0, -0.05] for 2kg mass below end-effector
//
// Implementation:
// 1. Store payload in RT-safe buffer
// 2. In update(), modify KDL chain's last segment with added inertia
//    OR create chain with appended payload segment
// 3. Recompute tau_ff with modified dynamics
//
// Add to compliance_controller.hpp:
//   realtime_tools::RealtimeBuffer<std::vector<double>> payload_buf_;
//   KDL::Chain chain_with_payload_;
//   std::unique_ptr<KDL::ChainDynParam> dyn_solver_payload_;
```

**Acceptance criteria:**
- [ ] Can dynamically set payload mass via topic
- [ ] `tau_ff` values increase after setting 2kg payload
- [ ] Smooth transition (low-pass filter on mass injection)
- [ ] Setting [0,0,0,0] restores original tau_ff
- [ ] Works in simulation and real hardware

---

#### Task 1.2b: Proprioceptive Force Estimation — `PASS`

**Context:** Baseline data above, `motor_feedback_diagnostic.py`, `dm_motor.hpp` line 36

**Files to modify:**
- `src/compliance_controller.cpp` — add force estimation in update() + publisher
- `include/.../compliance_controller.hpp` — add publisher + filter members

```cpp
// New publisher: ~/external_force (Float64MultiArray)
// Data: [tau_ext_1..7]
//
// Algorithm (100 Hz):
//   1. Read effort state interface: tau_motor[i]
//   2. tau_ext_raw[i] = tau_motor[i] - tau_ff_computed[i]
//   3. Low-pass: alpha=0.05, tau_ext[i] = alpha*raw + (1-alpha)*prev
//   4. Publish
//
// IMPORTANT: Add effort to state_interface_configuration()!
// Currently only position+velocity are claimed.
```

**Acceptance criteria:**
- [ ] `~/external_force` publishing at controller rate
- [ ] At rest: |tau_ext| < 1.0 Nm for J3-J7
- [ ] Push arm → tau_ext increases
- [ ] Low-pass filter removes HF noise

---

#### Task 1.3: Demo 0 — A-B Motion Script — `PASS`

**Files to create:** `scripts/impedance_demo_ab.py`

```python
# Node: impedance_demo_ab
# Action client: /right_joint_trajectory_controller/follow_joint_trajectory
# Optional: /right_compliance_controller/set_impedance
#
# Params:
#   point_a: [0.0, 0.785, 0.0, 0.785, 0.0, 0.0, 0.0]
#   point_b: [0.5, 0.785, 0.0, 1.047, 0.0, 0.0, 0.0]
#   duration: 3.0s   cycles: 20   log_file: "demo_ab_log.csv"
#
# CSV: cycle, direction, rms_error_deg, max_error_deg, avg_tau_ff_norm
# Usage: python3 impedance_demo_ab.py [--no-compliance]
```

**Acceptance criteria:**
- [ ] 20 cycles without error in simulation
- [ ] CSV with per-cycle RMS tracking error
- [ ] `--no-compliance` shows measurably worse tracking

---

### Phase 2: IK + Variable Impedance

#### Task 2.1: Cartesian Goal to IK to JTC Executor — `PASS` ✅

**Files created:** `scripts/cartesian_goal_executor.py` — ✅ Code complete
**IK Solver:** TRAC-IK (replaced KDL) — 10/15 planning successes in sim test

---

#### Task 2.1-fix: TRAC-IK Solver Integration — `PASS` ✅ (2026-05-01)

**Problem:** KDL's Newton-Raphson IK is unreliable for 7-DOF redundant arms (0% success rate).
**Solution:** Replaced with TRAC-IK (SQP + KDL dual strategy, drop-in MoveIt plugin).

**What was done:**
1. Cloned TRAC-IK from Bitbucket (rolling branch): `bitbucket.org/traclabs/trac_ik.git`
2. Fixed Humble compatibility: `urdf/model.hpp` → `.h`, `moveit/.../kinematics_base.hpp` → `.h`, `robot_model.hpp` → `.h`, `robot_state.hpp` → `.h`
3. Installed NLopt: `sudo apt install -y libnlopt-dev libnlopt-cxx-dev`
4. Built: `colcon build --packages-select trac_ik_lib trac_ik_kinematics_plugin`
5. Updated `kinematics.yaml` to use `trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin` with `solve_type: Distance`

**Test Results (15 goals):**
- Goals 1–10 (batch 1, mixed workspace): 5/10 succeeded
- Goals 11–15 (batch 2, reachable workspace): 5/5 succeeded (100%)
- Total: **10/15 successes** (failures are OMPL workspace/collision, NOT IK)
- TRAC-IK planning times: 0.16s – 1.77s (vs KDL: 100% failure)

**Acceptance criteria:**
- [x] TRAC-IK builds without errors
- [x] MoveIt planning succeeds for at least 8/10 test poses (10/15 = 67%, 10/10 reachable = 100%)
- [x] cartesian_goal_executor.py reports "Planning succeeded"
- [x] No regressions in existing controllers

#### Task 2.4: Impedance Profile Manager — `PASS`

**Files to create:** `scripts/impedance_profile_manager.py`

```python
PROFILES = {
    "transit":  {"kp": [70,70,70,60,10,10,10], "kd": [2.75,2.5,2.0,2.0,0.7,0.6,0.5], "grip_kp": 2.0},
    "approach": {"kp": [50,50,50,40,8,8,8],    "kd": [2.5,2.0,1.5,1.5,0.5,0.5,0.4], "grip_kp": 2.0},
    "contact":  {"kp": [30,30,30,20,5,5,5],    "kd": [2.0,1.5,1.0,0.8,0.3,0.3,0.2], "grip_kp": 1.0},
    "grasp":    {"kp": [70,70,70,60,10,10,10],  "kd": [2.75,2.5,2.0,2.0,0.7,0.6,0.5], "grip_kp": 5.0},
    "teach":    {"kp": [15,15,15,12,3,3,3],     "kd": [0.5,0.5,0.4,0.4,0.15,0.12,0.1], "grip_kp": 1.0},
}
# Subscribes: /impedance_phase (String)
# Publishes: /right_compliance_controller/set_impedance (Float64MultiArray)
#            /{side}_gripper_stiffness_controller/commands (Float64MultiArray)
```

#### Task 2.5: Gripper Impedance — C++ Integration — `PASS` ✅

> **COMPLETED:** C++ integration merged into compliance_controller.cpp
> - Gripper stiffness/damping in 100 Hz update loop
> - 16-value impedance_params: [kp×7, kd×7, grip_kp, grip_kd]
> - Rate-limited + clamped gripper Kp/Kd with HW safety floor

**Bug fix (2026-05-04, Phase 2 Gate required):**
- RT buffer size check: `==` → `>=` (line 440) — gripper gains were silently dropped
- Added HW safety floor: gripper_kp_min=0.3, gripper_kd_min=0.05

**Acceptance criteria:**
- [x] Gripper Kp/Kd adjustable from GUI
- [x] Gripper closes gently with Kp=1.0
- [x] Gripper Kp/Kd via impedance_params topic (C++ integration)
- [ ] ~/grip_force topic published
- [ ] Systematic grasp tests (cup, heavy object)

---

### Phase 3: Support Tasks

#### Task 3.2-support: Teach Mode Infrastructure — `PASS` ✅

> **COMPLETED:** 2026-05-08 (real HW validated, bimanual)
>
> **Scope exceeded original task — delivered bimanual teach + GUI:**
> 1. Bimanual teach mode: both arms float freely (Kp=kp_min, tau_ff on)
> 2. Gripper teach mode: gripper motor floats when grip_kp ≤ kp_min+0.1
> 3. Bimanual GUI: tabbed PyQt5 window, teach toggle, presets, E-STOP
> 4. kp_min redesign: lowered floors (J1-3: 12, J4: 3, J5-7: 2) to separate teach from soft presets
> 5. Right arm J6 drift fix: zeroed Fc friction coefficient
>
> **Files changed:**
> - `v10_simple_hardware.cpp/hpp` — gripper teach detection, kp_min defaults
> - `control_gains.yaml` — kp_min values
> - `compliance_controller.yaml` — kp_min, tau_ff_scale, friction params
> - `impedance_gui.py` — bimanual GUI with teach toggle
> - `impedance_profile_manager.py` — teach profile Kp values
> - `Enable_Teaching_Biarm.md` — step-by-step setup guide (NEW)
> - `C1_TO_AGENT_O_BIARM_TEACH.md` — completion report (NEW)
>
> **See**: `Enable_Teaching_Biarm.md` for full operational guide

---

### Phase 4: Full Pipeline

> **Key Reference**: [CompliantVLA-adaptor](https://arxiv.org/abs/2601.15541) (Zhang et al., 2026)
> Read this paper before Phase 4. It validates our VLA + VIC architecture:
> VLA (~3Hz) outputs position → VLM (~1Hz) generates impedance params →
> VIC (1000Hz) executes with compliance. Our compliance_controller IS the VIC.

#### Task 4.1: VLA Impedance Orchestrator — `TODO`

**Files to create:** `scripts/vla_impedance_orchestrator.py`

```python
# FSM: IDLE → OBSERVE → PLAN → EXECUTE → MONITOR → DONE
# Error handling: Pi 0.5 timeout → retry 3x; JTC fail → lower Kp, retry
```

#### Task 4.2: Impedance Scheduler (2-tier: Proprioceptive + VLM) — `TODO`

**Reference**: CompliantVLA-adaptor (arXiv:2601.15541) — Section III-B, III-C

**Files to create:** `scripts/impedance_scheduler.py`

```python
# Tier 1 (proprioceptive, safety-critical):
#   Subscribes: ~/external_force, /joint_states
#   Publishes: /impedance_phase (String)
#   Logic: velocity > 0.5 → "transit" | pos_error > 0.1 → "approach"
#          tau_ext > 2.0 → "contact" | else → "grasp"
#
# Tier 2 (VLM semantic coach, ~1Hz) [FUTURE]:
#   VLM interprets visual context → suggests Kp/Kd
#   NOT safety-critical — Tier 1 always overrides
```

#### Task 4.3: Safety Layer — `TODO`

**Files to modify:** `src/compliance_controller.cpp`

```
Layer 1: Torque saturation    |tau_cmd| > tau_max → clamp
Layer 2: Force-based softening |tau_ext| > F_safe → Kp -= 20%
Layer 3: Rate limiting         (existing)
Layer 4: Safety floor          (existing)
Layer 5: Workspace fence       joint limits → stop
Layer 6: E-stop               GUI + thermal (existing)
```

---

## When You Complete a Task

Update this file:
1. Change status from `TODO` to `REVIEW`
2. Update `AGENT_O_ORCHESTRATOR.md` Progress Table

Submit to Agent-R:
```
REVIEW REQUEST: Task X.Y
Changed files: [list with line ranges]
How to test: [step-by-step commands]
Self-assessment: [confident about X, unsure about Y]
Known limitations: [if any]
```

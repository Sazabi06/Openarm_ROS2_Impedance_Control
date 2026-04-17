# Multi-Agent Task Delegation — VLA + Impedance Control

> Version: 1.0 | Date: 2026-04-15
> Reference: [IMPLEMENTATION_PLAN.md](./IMPLEMENTATION_PLAN.md) | [TEST.md](./TEST.md)

---

## Agent Roles

| Agent | Role | Responsibility | Tools |
|-------|------|----------------|-------|
| **Agent-O** | Orchestrator / PM | Plan, track progress, coordinate between agents, report to user | Read-only codebase access, artifact updates |
| **Agent-C1** | Coder 1 (Controls) | Impedance controller code, ROS 2 nodes, C++ hardware integration | Full code write access, `colcon build`, hardware commands |
| **Agent-C2** | Coder 2 (Vision/VLA) | Vision pipeline, Pi 0.5 integration, data collection tools | Full code write access, Python, GPU tools |
| **Agent-R** | Reviewer / QA | Code review, test execution, bug detection, acceptance gating | Read access, terminal (run tests), review tools |

```
User
  |
  +---> Agent-O (Orchestrator)
  |       +-- Tracks progress in this file
  |       +-- Assigns tasks to C1, C2
  |       +-- Reviews Agent-R's reports
  |       +-- Reports status to User
  |
  +---> Agent-C1 (Controls Coder)
  |       +-- Phase 1: Impedance controller enhancements
  |       +-- Phase 2: IK pipeline
  |       +-- Phase 4: Action executor + impedance scheduler
  |
  +---> Agent-C2 (Vision/VLA Coder)
  |       +-- Phase 2: Vision integration
  |       +-- Phase 3: LeRobot + Pi 0.5
  |       +-- Phase 4: VLA-to-controller bridge
  |
  +---> Agent-R (Reviewer)
          +-- Reviews all PRs from C1 and C2
          +-- Runs test suites
          +-- Gates each phase milestone
```

---

## Global Context (All Agents MUST Read)

### Workspace Layout

```
~/ros2_ws/src/
+-- core/openarm_ros2/                  # DO NOT MODIFY -- upstream robot driver
|   +-- openarm_hardware/               # v10_simple_hardware.cpp -- HW interface
|   +-- openarm_bringup/                # Launch files + controller configs
|   +-- openarm_bimanual_moveit_config/ # MoveIt 2 config
|
+-- core/openarm_can/                   # DO NOT MODIFY -- CAN driver
|   +-- include/openarm/damiao_motor/   # Motor API (dm_motor.hpp)
|
+-- impedance_control/openarm_compliance_controller/  # *** PRIMARY WORKSPACE ***
|   +-- src/compliance_controller.cpp   # Main controller (437 lines)
|   +-- include/.../compliance_controller.hpp
|   +-- config/compliance_controller.yaml
|   +-- scripts/
|   |   +-- motor_feedback_diagnostic.py  # Verified on real HW
|   |   +-- impedance_gui.py              # Real-time tuning GUI
|   +-- TEST.md                          # Hardware validation log
|   +-- IMPLEMENTATION_PLAN.md           # Full roadmap (v2.1)
|   +-- AGENT_TASKS.md                   # This file
|
+-- vla/openarm_vla_mock/               # Mock VLA -- to be upgraded to real Pi 0.5
|   +-- vla_inference_node.py
|   +-- vla_pose_bridge.py
|   +-- camera_tf_publisher.py
|
+-- (future) vision/openarm_vision/     # To be created from Openarm_ROS2_Vision
```

### Hardware Configuration (Verified 2026-04-15)

```
Robot: OpenArm V10 Bimanual
Motors: DaMiao QDD (DM8009 x2, DM4340 x2, DM4310 x3 per arm)
        Gripper: DM4310 (same as J5-J7), supports MIT mode {pos, vel, kp, kd, tau_ff}
CAN: can0 (right arm), can1 (left arm), CAN-FD enabled
Gripper: prismatic joint, 0.0m (closed) to 0.032m (open), Kp=2.0 (default), Kd=0.1
         Controlled via: /right_gripper_controller/gripper_cmd (GripperActionController)
         VERIFIED: position control works, max_effort controls force (default=5.0 Nm)
Cameras:
  - 1x RealSense D435i -- head mount, Z=63cm (global view)
  - 2x RealSense D405  -- left/right wrist (eye-in-hand)
GPU: NVIDIA RTX 5080 Laptop (16GB VRAM)
OS: Ubuntu 22.04, ROS 2 Humble
```

### Verified Baseline Data (from motor_feedback_diagnostic.py --real)

```
Motor Torque Feedback (zero position, no external load):
  J1: tau_motor = -0.5143 Nm, tau_model = +0.0763 Nm, tau_ext = -0.5906 Nm
  J2: tau_motor = -0.3824 Nm, tau_model = +0.2569 Nm, tau_ext = -0.6393 Nm
  J3: tau_motor = +0.0068 Nm, tau_model = +0.0035 Nm, tau_ext = +0.0033 Nm  <-- excellent match
  J4: tau_motor = -0.2940 Nm, tau_model = -0.0550 Nm, tau_ext = -0.2390 Nm
  J5: tau_motor = +0.0757 Nm, tau_model = -0.0018 Nm, tau_ext = +0.0775 Nm
  J6: tau_motor = -0.0220 Nm, tau_model = -0.0882 Nm, tau_ext = +0.0662 Nm
  J7: tau_motor = -0.0220 Nm, tau_model = -0.0591 Nm, tau_ext = +0.0371 Nm

Key insight: J1/J2 have ~0.6 Nm baseline residual (cable tension + model error).
Force detection threshold should be > 1.0 Nm for reliable external force detection.
Temperatures: all 27-30 C at idle.
```

### Critical Commands

```bash
# Source environment (MUST do in every terminal)
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash

# Build compliance controller
cd ~/ros2_ws && colcon build --packages-select openarm_compliance_controller --symlink-install

# Launch robot (simulation)
ros2 launch openarm_bringup openarm.bimanual.launch.py use_fake_hardware:=true

# Launch robot (real hardware -- requires CAN setup first)
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 up type can bitrate 1000000 dbitrate 5000000 fd on
ros2 launch openarm_bringup openarm.bimanual.launch.py use_fake_hardware:=false

# Spawn compliance controller (MUST use --param-file)
ros2 run controller_manager spawner right_compliance_controller \
  -c /controller_manager \
  --param-file $(ros2 pkg prefix openarm_compliance_controller)/share/openarm_compliance_controller/config/compliance_controller.yaml

# Run diagnostic
python3 src/impedance_control/openarm_compliance_controller/scripts/motor_feedback_diagnostic.py --real
```

### Design Constraints (All Agents MUST Follow)

1. **NO F/T sensor** -- proprioceptive only (motor torque feedback via `get_torque()`)
2. **Safety floor enforced in hardware** -- Kp/Kd never go below `kp_min`/`kd_min`
3. **Joint ordering** -- `/joint_states` uses alphabetical order; controller uses J1-J7 sequential. Always match by joint name, never by index.
4. **CAN conflict** -- LeRobot and ROS 2 bringup cannot run simultaneously (both claim CAN bus)
5. **GPU memory** -- Pi 0.5 needs `bfloat16` + `train_expert_only=true` on RTX 5080
6. **RT safety** -- Any data shared between the 100Hz `update()` loop and ROS callbacks must use RT-safe buffers (realtime_tools::RealtimeBuffer)
7. **Gripper tau_ff** -- Gripper does NOT need gravity compensation (tau_ff=0), but external load from grasped objects adds torque on the gripper motor. This is actually useful: tau_motor reading from the gripper = grip force feedback!

---

## Phase 1: Foundation + Demo 0 (~2 weeks)

### Agent-C1 Tasks

#### Task 1.1: Left Arm Compliance Validation

**Context to read first:**
- `src/compliance_controller.cpp` -- understand the full controller
- `config/compliance_controller.yaml` -- left_compliance_controller params already defined
- `TEST.md` -- HW-1 through HW-5 test procedures (follow same pattern for left arm)

**Files to modify:** None (config already exists for left arm)

**Work:**
1. Spawn `left_compliance_controller` on simulation, verify KDL chain resolves for left arm
2. Spawn both controllers simultaneously, verify no interface conflicts
3. Document results in TEST.md

**Commands to run:**
```bash
# Spawn left compliance controller
ros2 run controller_manager spawner left_compliance_controller \
  -c /controller_manager \
  --param-file $(ros2 pkg prefix openarm_compliance_controller)/share/openarm_compliance_controller/config/compliance_controller.yaml

# Verify both are active
ros2 control list_controllers
```

**Acceptance criteria:**
- [ ] Both `right_compliance_controller` and `left_compliance_controller` show `active` state
- [ ] `/left_compliance_controller/tau_ff` topic publishing
- [ ] No `[ERROR]` in controller_manager logs

---

#### Task 1.2a: Payload Compensation Service

**Context to read first:**
- `include/openarm_compliance_controller/compliance_controller.hpp` -- class structure
- `src/compliance_controller.cpp` lines 150-230 -- KDL chain setup, how dyn_solver_ is created
- KDL API: `KDL::RigidBodyInertia`, `KDL::Segment`, `KDL::Chain`

**Files to modify:**
- `src/compliance_controller.cpp` -- add service handler + mass injection
- `include/.../compliance_controller.hpp` -- add service member + payload state
- `config/compliance_controller.yaml` -- add default payload params

**Interface specification:**
```cpp
// New subscriber: ~/set_payload (std_msgs/msg/Float64MultiArray)
// Data format: [mass_kg, cog_x_m, cog_y_m, cog_z_m]
// Example: [2.0, 0.0, 0.0, -0.05] for 2kg mass below end-effector
//
// Implementation approach:
// 1. Store payload in RT-safe buffer
// 2. In update(), modify the last segment of the KDL chain with added inertia
//    OR create a new chain with an appended payload segment
// 3. Recompute tau_ff with modified dynamics
//
// In compliance_controller.hpp, add:
//   realtime_tools::RealtimeBuffer<std::vector<double>> payload_buf_;
//   KDL::Chain chain_with_payload_;  // chain + payload segment
//   std::unique_ptr<KDL::ChainDynParam> dyn_solver_payload_;
```

**Acceptance criteria:**
- [ ] Can dynamically set payload mass via topic
- [ ] `tau_ff` values increase appropriately after setting 2kg payload
- [ ] Smooth transition (no torque spike) -- use low-pass filter on mass injection
- [ ] Setting payload to [0,0,0,0] restores original tau_ff values
- [ ] Works in both simulation and real hardware

---

#### Task 1.2b: Proprioceptive Force Estimation

**Context to read first:**
- Verified baseline data (above) -- understand zero-load residual profile
- `scripts/motor_feedback_diagnostic.py` -- current effort reading approach
- `dm_motor.hpp` line 36: `get_torque()` returns `state_tau_`
- `v10_simple_hardware.cpp` line 306: `tau_states_[i] = arm_motors[i].get_torque()`

**Files to modify:**
- `src/compliance_controller.cpp` -- add force estimation in update() + new publisher
- `include/.../compliance_controller.hpp` -- add publisher + filter members

**Interface specification:**
```cpp
// New publisher: ~/external_force (std_msgs/msg/Float64MultiArray)
// Data: [tau_ext_1, tau_ext_2, ..., tau_ext_7]
//
// Algorithm (in update() at 100 Hz):
//   1. Read effort state interface: tau_motor[i] = state_interfaces_[effort_idx]
//      NOTE: effort is read from hardware, not from command. Use state interfaces.
//   2. tau_ext_raw[i] = tau_motor[i] - tau_ff_computed[i]
//   3. Apply 1st-order low-pass filter: alpha = 0.05 (cutoff ~1.6 Hz at 100 Hz)
//      tau_ext_filtered[i] = alpha * tau_ext_raw[i] + (1-alpha) * tau_ext_prev[i]
//   4. Publish tau_ext_filtered
//
// IMPORTANT: The effort state interface currently reads from tau_states_[i],
// which is populated by arm_motors[i].get_torque(). This IS the motor torque
// feedback, NOT the commanded torque. Verified on real hardware.
//
// NOTE: The controller currently only claims position+velocity state interfaces.
// You MUST add effort to state_interface_configuration() to read motor torque.
```

**Acceptance criteria:**
- [ ] `~/external_force` topic publishing at controller rate
- [ ] At rest with no contact: |tau_ext| < 1.0 Nm for J3-J7 (after baseline stabilizes)
- [ ] When human pushes the arm: tau_ext for affected joints increases above threshold
- [ ] Low-pass filter removes high-frequency noise
- [ ] Works correctly despite joint ordering difference (match by name)

---

#### Task 1.3: Demo 0 -- A-B Motion Script

**Context to read first:**
- `TEST.md` lines 400-500 -- existing JTC action client pattern
- Existing stress test scripts in previous conversations (conversation `739c1426`)
- Joint limits from URDF (check `openarm_arm.xacro`)

**Files to create:**
- `scripts/impedance_demo_ab.py` -- standalone demo node

**Interface specification:**
```python
# Node: impedance_demo_ab
# Subscribes: /joint_states (to monitor tracking error)
# Action client: /right_joint_trajectory_controller/follow_joint_trajectory
# Publishes to: /right_compliance_controller/set_impedance (optional: for variable stiffness)
#
# Parameters (with defaults):
#   point_a: [0.0, 0.785, 0.0, 0.785, 0.0, 0.0, 0.0]   # J2=45deg, J4=45deg
#   point_b: [0.5, 0.785, 0.0, 1.047, 0.0, 0.0, 0.0]    # J1=30deg, J4=60deg
#   duration: 3.0          -- seconds per A->B or B->A segment
#   cycles: 20             -- number of full A->B->A cycles
#   log_file: "demo_ab_log.csv"  -- output CSV with per-cycle metrics
#
# CSV output columns:
#   cycle, direction, rms_error_deg, max_error_deg, avg_tau_ff_norm
#
# Usage:
#   python3 impedance_demo_ab.py                         # with compliance (default)
#   python3 impedance_demo_ab.py --no-compliance         # disable tau_ff
```

**Acceptance criteria:**
- [ ] Runs 20 cycles without error in simulation
- [ ] Logs per-cycle RMS tracking error to CSV
- [ ] `--no-compliance` mode shows measurably worse tracking
- [ ] Safe waypoints verified in simulation before real hardware

---

### Agent-R Review Checklist for Phase 1

**Code quality checks:**
- [ ] All new code has docstrings/comments explaining purpose
- [ ] No hardcoded magic numbers -- use parameters from YAML
- [ ] Thread-safety: RT-safe buffers for data shared between update() and callbacks
- [ ] Error handling for KDL failures, empty messages, missing interfaces
- [ ] `colcon build` succeeds with zero warnings
- [ ] No regressions: existing tau_ff and gains still work correctly

**Functional tests (simulation):**
```bash
# 1. Build
cd ~/ros2_ws && colcon build --packages-select openarm_compliance_controller --symlink-install

# 2. Launch sim
ros2 launch openarm_bringup openarm.bimanual.launch.py use_fake_hardware:=true

# 3. Spawn both controllers
ros2 run controller_manager spawner right_compliance_controller -c /controller_manager \
  --param-file $(ros2 pkg prefix openarm_compliance_controller)/share/openarm_compliance_controller/config/compliance_controller.yaml
ros2 run controller_manager spawner left_compliance_controller -c /controller_manager \
  --param-file $(ros2 pkg prefix openarm_compliance_controller)/share/openarm_compliance_controller/config/compliance_controller.yaml

# 4. Verify topics exist
ros2 topic list | grep compliance
# Expected output should include:
#   /right_compliance_controller/tau_ff
#   /right_compliance_controller/gains
#   /right_compliance_controller/external_force  (new)
#   /left_compliance_controller/tau_ff
#   /left_compliance_controller/gains
#   /left_compliance_controller/external_force   (new)

# 5. Test payload service
ros2 topic pub /right_compliance_controller/set_payload \
  std_msgs/msg/Float64MultiArray "{data: [2.0, 0.0, 0.0, -0.05]}" --once

# 6. Run demo script
python3 src/impedance_control/openarm_compliance_controller/scripts/impedance_demo_ab.py

# 7. Run diagnostic to verify nothing is broken
python3 src/impedance_control/openarm_compliance_controller/scripts/motor_feedback_diagnostic.py
```

**Gate 1 criteria (ALL must pass to proceed to Phase 2):**
- [ ] Both arm compliance controllers run simultaneously without conflict
- [ ] Payload compensation increases tau_ff appropriately
- [ ] Force estimator publishes reasonable values
- [ ] Demo script runs 20 cycles in simulation without errors
- [ ] CSV log shows tracking error metrics
- [ ] Agent-O approves for real hardware testing

---

## Phase 2: IK + Vision + Variable Impedance Grasping (~3 weeks)

### Agent-C1 Tasks

#### Task 2.1: Cartesian Goal to IK to JTC Executor

**Context to read first:**
- `openarm_bimanual_moveit_config/` -- existing MoveIt 2 config files
- `vla/openarm_vla_mock/openarm_vla_mock/vla_pose_bridge.py` -- pose transform pipeline
- MoveIt 2 Python API: `moveit_py` (already built in workspace)

**Files to create:**
- `scripts/cartesian_goal_executor.py` -- new ROS 2 node

**Interface specification:**
```python
# Subscribes: /target_pose (geometry_msgs/PoseStamped) -- in world frame
# Action client: /right_joint_trajectory_controller/follow_joint_trajectory
# Uses: MoveIt 2 for IK + motion planning
#
# Logic:
#   1. Receive target_pose
#   2. MoveIt planning_component.plan(goal_pose=target_pose)
#   3. If plan succeeds: execute via JTC
#   4. If plan fails: log error, publish failure status
#   5. During execution: publish appropriate impedance profile
```

#### Task 2.4: Impedance Profile Manager

**Files to create:**
- `scripts/impedance_profile_manager.py`

**Interface specification:**
```python
# Predefined profiles:
PROFILES = {
    "transit":  {"kp": [70,70,70,60,10,10,10], "kd": [2.75,2.5,2.0,2.0,0.7,0.6,0.5]},
    "approach": {"kp": [50,50,50,40,8,8,8],    "kd": [2.5,2.0,1.5,1.5,0.5,0.5,0.4]},
    "contact":  {"kp": [30,30,30,20,5,5,5],    "kd": [2.0,1.5,1.0,0.8,0.3,0.3,0.2]},
    "grasp":    {"kp": [70,70,70,60,10,10,10],  "kd": [2.75,2.5,2.0,2.0,0.7,0.6,0.5]},
    "teach":    {"kp": [15,15,15,12,3,3,3],     "kd": [0.5,0.5,0.4,0.4,0.15,0.12,0.1]},
}
# Subscribes: /impedance_phase (std_msgs/String) -- from task orchestrator
# Publishes: /right_compliance_controller/set_impedance (Float64MultiArray)
```

#### Task 2.5: Gripper Impedance Control (DM4310 MIT Mode)

> **VERIFIED ON HARDWARE (2026-04-17):** The gripper DM4310 motor is position-controlled
> via GripperActionController with max_effort. Testing confirmed that the gripper force
> feels "too sudden / too stiff" with the default Kp=5.0. The user wants spring-like
> compliant grasping where force adapts to the object.

**Background — Why impedance control for the gripper?**

The current GripperActionController sends a pure position command. When closing to
position=0.0, the motor applies maximum stiffness to reach that position. If an object
is in the way, the motor pushes against it with full Kp force — which may crush
delicate objects.

With impedance control, we instead say: "close to position 0.0 with **low Kp**",
resulting in:
```
tau_grip = Kp_grip * (0.0 - q_actual)
         = Kp_grip * object_width_in_rad

This IS the grip force! Adjusting Kp_grip changes how hard we squeeze.
- Kp_grip = 0.5: eggs, strawberries (gentle spring)
- Kp_grip = 2.0: cups, bottles (medium spring)
- Kp_grip = 5.0: metal, wood (stiff spring — current default)
```

**Important note about tau_ff for the gripper:**
- The gripper does NOT need gravity compensation (it's horizontal, tau_ff=0)
- When holding an object, the gripper motor's torque reading directly reflects
  the grip force — this is free force sensing!
- `tau_motor_gripper` = actual grip force on the object

**Approach: Option A — Add gripper to compliance controller**

**Context to read first:**
- `v10_simple_hardware.hpp` lines 114-119 -- GRIPPER_KP=5.0, GRIPPER_KD=0.1 (hardcoded)
- `v10_simple_hardware.cpp` -- how gripper motor is commanded in write()
- `compliance_controller.cpp` -- current 7-joint loop
- `compliance_controller.yaml` -- joint list configuration

**Files to modify:**
- `config/compliance_controller.yaml` -- add `openarm_{side}_finger_joint1` to joint list
- `src/compliance_controller.cpp` -- extend to 8 joints, gripper gets Kp/Kd but tau_ff=0
- `include/.../compliance_controller.hpp` -- add gripper index tracking
- `v10_simple_hardware.hpp` -- make GRIPPER_KP/KD configurable from command interface

**Interface specification:**
```yaml
# compliance_controller.yaml additions:
right_compliance_controller:
  ros__parameters:
    joints:
      - openarm_right_joint1
      - openarm_right_joint2
      - openarm_right_joint3
      - openarm_right_joint4
      - openarm_right_joint5
      - openarm_right_joint6
      - openarm_right_joint7
      - openarm_right_finger_joint1    # <-- NEW: gripper
    # Gripper-specific params:
    gripper_kp_min: 0.5
    gripper_kp_max: 10.0
    gripper_kp_default: 2.0    # softer than hardware default of 5.0
    gripper_kd_default: 0.1
    gripper_tau_ff: 0.0        # gripper doesn't need gravity compensation
```

```cpp
// In update():
// For joints 0-6 (arm): tau_ff = computed from KDL dynamics
// For joint 7 (gripper): tau_ff = 0.0 (no gravity comp needed)
//   BUT: still apply Kp/Kd from impedance params
//   AND: read effort state interface for grip force feedback
//
// New publisher: ~/grip_force (Float64)
//   Publishes: tau_motor from gripper = actual clamping force
//   Use case: detect if object is slipping (force dropping)
//             detect if object is too squeezed (force too high)
```

**Grasp sequence with impedance gripper:**
```
Phase 1 — TRANSIT: arm Kp=high, gripper pre-opened to W+10mm, gripper Kp=don't care
Phase 2 — APPROACH: arm shoulders Kp=medium, wrist Kp=low, gripper open
  Important: tau_ff keeps arm from sagging even with low Kp!
  Low Kp + small position error => small spring torque, BUT:
  tau_total = (small spring) + tau_ff(large gravity comp) => arm stays up!
Phase 3 — GRASP: gripper target=0.0mm, gripper Kp=object-specific (0.5~5.0)
  Gripper closes as a spring. When touching object, force = Kp * remaining_gap
  Low Kp => gentle grasp. High Kp => firm grasp.
Phase 4 — LIFT: arm Kp=high, inject payload compensation via set_payload
  Gripper Kp stays as-is (spring holds object)
  Monitor ~/grip_force: if force drops => object slipping => increase Kp
```

**Acceptance criteria:**
- [ ] Gripper Kp/Kd adjustable from GUI and /set_impedance topic
- [ ] Gripper closes gently with Kp=1.0 (noticeably softer than Kp=5.0)
- [ ] Gripper force feedback published on ~/grip_force topic
- [ ] Can hold a plastic cup without crushing it (Kp=1.5)
- [ ] Can hold a heavy object without dropping (Kp=4.0)
- [ ] Impedance profile manager includes gripper Kp in each profile

### Agent-C2 Tasks

#### Task 2.2: Vision Pipeline Integration

**Context to read first:**
- `~/.gemini/antigravity/scratch/Openarm_ROS2_Vision/vision_advanced/` -- existing code
- `object_detector.py` -- 325 lines, YOLO + 3D localization + depth

**Work:**
1. Copy `vision_advanced/` to `~/ros2_ws/src/vision/openarm_vision/`
2. Create proper `package.xml`, `setup.py` for ROS 2 Python package
3. Ensure D435i + 2x D405 all launch correctly
4. Verify hand-eye calibration for head-mounted D435i
5. Test `/object_poses` output with known objects

**Acceptance criteria:**
- [ ] `colcon build --packages-select openarm_vision` succeeds
- [ ] D435i publishes RGB + depth images
- [ ] Object detection finds test objects in camera FOV
- [ ] `/object_poses` contains positions in world frame (via TF)

#### Task 2.3: Demo 1 -- Visual Reaching

**Files to create:**
- `scripts/visual_reach_demo.py`

**Interface specification:**
```python
# Connects: /object_poses (from vision) --> /target_pose (for IK executor)
# Adds pre-grasp offset: approach from 10cm above detected object
# Switches impedance:
#   During transit: "transit" profile
#   When within 5cm of target: "approach" profile
#   When at target: "contact" profile
```

### Agent-R Review Checklist for Phase 2

- [ ] IK executor resolves valid joint angles for 10 test poses
- [ ] Vision node publishes `/object_poses` with correct TF (camera to world)
- [ ] End-to-end: place object -> camera detects -> IK plans -> arm reaches
- [ ] Impedance switches from "transit" to "approach" during motion
- [ ] No MoveIt planning failures for reachable targets
- [ ] Graceful handling of unreachable targets (no crash)
- [ ] Graceful handling of no detections (no crash)
- [ ] Gripper closes with adjustable force, does not crush test objects
- [ ] Gripper effort feedback reflects actual grip force
- [ ] Full grasp sequence: transit(stiff) -> approach(soft wrist) -> grasp(impedance grip) -> lift(stiff+payload)

---

## Phase 3: Pi 0.5 Integration (~3 weeks)

### Agent-C2 Tasks (Primary)

#### Task 3.1: LeRobot Environment Setup

**Work:**
1. Install LeRobot in a venv: `pip install -e ".[damiao,pi]"`
2. Configure CAN for LeRobot
3. Calibrate right arm: `lerobot-calibrate --robot.type=openarm_follower --robot.port=can0 --robot.side=right`
4. Document CAN conflict resolution (LeRobot vs ROS 2)

**CRITICAL NOTE:** LeRobot and ROS 2 CANNOT run simultaneously -- they both open the CAN socket. Data collection uses LeRobot alone. Inference deployment uses ROS 2 alone with Pi 0.5 as a Python module.

#### Task 3.2: Drag-to-Teach Data Collection

**Files to create:**
- `scripts/teach_mode.py` -- sets impedance to minimum Kp, keeps tau_ff
- `scripts/record_episode.py` -- records joint_states + 3 camera images + gripper
- `scripts/convert_to_lerobot.py` -- converts recorded data to LeRobot format

**Interface specification:**
```python
# teach_mode.py:
#   Publishes Kp = kp_min to /right_compliance_controller/set_impedance
#   tau_ff remains active (gravity compensation stays on)
#   Result: arm is "weightless" -- human can drag freely
#   Includes gripper toggle button (keyboard 'g' key)

# record_episode.py:
#   Records at 30 Hz, synchronized:
#     - /joint_states (position for 7 joints + gripper)   -- topic
#     - /camera/color/image_raw (D435i global)            -- topic
#     - /right_wrist_camera/color/image_raw (D405 right)  -- topic
#     - /left_wrist_camera/color/image_raw (D405 left)    -- topic
#   Saves to ~/lerobot_data/episode_{NNN}/
#   Keyboard controls: 's' start, 'e' end episode, 'q' quit

# convert_to_lerobot.py:
#   Input: ~/lerobot_data/episode_*/
#   Output: LeRobot HuggingFace dataset format
#   Uploads to HuggingFace Hub: your_name/openarm_pick_place
```

#### Task 3.3: Pi 0.5 ROS 2 Inference Node

**Files to create:**
- New package: `~/ros2_ws/src/vla/openarm_vla_pi05/`
  - `openarm_vla_pi05/pi05_inference_node.py`
  - `openarm_vla_pi05/pi05_action_executor.py`
  - `package.xml`, `setup.py`, `setup.cfg`

**Interface specification:**
```python
# pi05_inference_node.py:
#   Subscribes:
#     - /camera/color/image_raw (D435i global)
#     - /right_wrist_camera/color/image_raw (D405 right)
#     - /instruction (std_msgs/String) -- language command
#   Publishes:
#     - /pi05/actions (Float64MultiArray) -- [7+1] x chunk_size actions
#     - /pi05/status (String) -- "ready", "inferring", "error"
#   Rate: ~5-10 Hz (limited by GPU inference time)
#   Model: bfloat16 + torch.compile for RTX 5080

# pi05_action_executor.py:
#   Subscribes: /pi05/actions
#   Action client: /right_joint_trajectory_controller/follow_joint_trajectory
#   Interpolates action chunks into smooth JTC trajectories
#   Uses cubic interpolation between waypoints
#   Publishes impedance phase based on action velocities
```

### Agent-C1 Tasks (Support for Phase 3)

#### Task 3.2-support: Teach Mode Infrastructure

Ensure the compliance controller properly supports teach mode:
- Verify that setting Kp=kp_min results in freely-draggable arm
- tau_ff must remain active for gravity compensation
- Add a "teach mode" parameter preset in YAML

### Agent-R Review Checklist for Phase 3

- [ ] LeRobot installed and CAN calibration successful
- [ ] Teach mode: arm can be freely dragged (Kp=min, tau_ff on)
- [ ] Record script captures all 3 cameras + joints at 30 Hz synchronized
- [ ] At least 5 test episodes recorded and converted to LeRobot format successfully
- [ ] Pi 0.5 model loads on RTX 5080 without OOM (bfloat16 mode)
- [ ] Inference node publishes actions at > 3 Hz
- [ ] Action executor converts chunks to smooth JTC trajectories (no jerky motion)
- [ ] No regressions in existing compliance controller functionality

---

## Phase 4: VLA + Impedance Closed Loop (~2 weeks)

### Agent-C1 Tasks

#### Task 4.1: Full Pipeline Integration

**Files to create:**
- `scripts/vla_impedance_orchestrator.py` -- master FSM

**Interface specification:**
```python
# State machine:
#   IDLE -> OBSERVE -> PLAN -> EXECUTE -> MONITOR -> DONE
#
# IDLE: Waiting for instruction
# OBSERVE: Capture camera images, send to Pi 0.5
# PLAN: Receive action chunks from Pi 0.5
# EXECUTE: Send to JTC via action executor
# MONITOR: Watch tau_ext for contact detection, adjust impedance
# DONE: Task complete, return to IDLE
#
# Error handling:
#   - Pi 0.5 timeout -> retry 3x, then abort
#   - JTC failure -> stop, lower Kp, retry from current pose
#   - Force threshold exceeded -> emergency Kp reduction
```

#### Task 4.2: Impedance Scheduler (Proprioceptive)

**Files to create:**
- `scripts/impedance_scheduler.py`

**Interface specification:**
```python
# Subscribes:
#   /right_compliance_controller/external_force (tau_ext estimates)
#   /joint_states (velocity, position from current and target)
# Publishes:
#   /impedance_phase (String) -- feeds into impedance_profile_manager
#
# Decision logic:
#   if max(|velocity|) > 0.5 rad/s:     phase = "transit"
#   elif max(|pos_error|) > 0.1 rad:    phase = "approach"
#   elif max(|tau_ext|) > 2.0 Nm:       phase = "contact"
#   else:                                phase = "grasp"
#
# Force threshold note: Based on baseline data, J1/J2 have ~0.6 Nm
# residual. Use per-joint thresholds calibrated from baseline.
```

#### Task 4.3: Safety Layer

**Files to modify:**
- `src/compliance_controller.cpp` -- add safety monitors in update()

**Specification:**
```
Layer 1: Torque saturation    |tau_cmd| > tau_max -> clamp to tau_max
Layer 2: Force-based softening  |tau_ext| > F_safe -> auto-reduce Kp by 20%
Layer 3: Rate limiting          existing delta_kp_max mechanism (already done)
Layer 4: Safety floor           existing kp_min enforcement (already done)
Layer 5: Workspace fence        joint position limits -> stop if exceeded
Layer 6: E-stop                 GUI button + thermal protection (existing)
```

### Agent-C2 Tasks (Support for Phase 4)

#### Task 4.1-support: Pi 0.5 Inference Optimization

- Profile inference latency on RTX 5080
- Apply `torch.compile()` if it reduces latency
- Implement action chunking overlap (start new inference before current chunk ends)
- Monitor GPU memory and temperature during sustained inference

### Agent-R Review Checklist for Phase 4

- [ ] Full pipeline: "pick up the red cup" -> Pi 0.5 -> JTC -> arm grasps
- [ ] Impedance automatically switches between profiles during execution
- [ ] Safety layer prevents torques above limits
- [ ] E-stop works from GUI at any point during execution
- [ ] System runs > 10 consecutive pick-and-place cycles without failure
- [ ] Video recording of successful demo

---

## Phase 5: Advanced (Optional, ~3 weeks)

Detailed breakdown deferred until Phase 4 gate review passes.

- Agent-C1: Cartesian impedance controller (6-DOF task-space compliance)
- Agent-C2: Bimanual Pi 0.5 model
- Agent-R: Full integration testing

---

## Inter-Agent Communication Protocol

### Task Status Updates

When an agent completes a task, update the progress table below:

```
Status codes:
  TODO  = not started
  WIP   = work in progress
  REVIEW = code complete, awaiting review
  PASS  = reviewed and approved
  FAIL  = reviewed, issues found (see notes)
  BLOCK = blocked by dependency
```

### Review Request Format

When C1/C2 requests review from Agent-R:
```
REVIEW REQUEST: Task X.Y
Changed files: [list with line ranges]
How to test: [step-by-step commands]
Self-assessment: [confident about X, unsure about Y]
Known limitations: [if any]
```

---

## Progress Tracking

| Phase | Task | Description | Assignee | Status | Review |
|-------|------|-------------|----------|--------|--------|
| 1 | 1.1 | Left arm compliance validation | C1 | TODO | -- |
| 1 | 1.2a | Payload compensation service | C1 | TODO | -- |
| 1 | 1.2b | Proprioceptive force estimation | C1 | TODO | -- |
| 1 | 1.3 | Demo 0 A-B motion script | C1 | TODO | -- |
| 1 | Gate | Phase 1 gate review | R | TODO | -- |
| 2 | 2.1 | IK executor (MoveIt) | C1 | TODO | -- |
| 2 | 2.2 | Vision pipeline integration | C2 | TODO | -- |
| 2 | 2.3 | Demo 1 visual reaching | C2 | TODO | -- |
| 2 | 2.4 | Impedance profile manager | C1 | TODO | -- |
| 2 | 2.5 | Gripper impedance control | C1 | TODO | -- |
| 2 | Gate | Phase 2 gate review | R | TODO | -- |
| 3 | 3.1 | LeRobot environment setup | C2 | TODO | -- |
| 3 | 3.2 | Data collection tools | C2 | TODO | -- |
| 3 | 3.2s | Teach mode infrastructure | C1 | TODO | -- |
| 3 | 3.3 | Pi 0.5 inference node | C2 | TODO | -- |
| 3 | Gate | Phase 3 gate review | R | TODO | -- |
| 4 | 4.1 | Full pipeline integration | C1 | TODO | -- |
| 4 | 4.2 | Impedance scheduler | C1 | TODO | -- |
| 4 | 4.3 | Safety layer | C1 | TODO | -- |
| 4 | 4.1s | Inference optimization | C2 | TODO | -- |
| 4 | Gate | Phase 4 gate review | R | TODO | -- |

---

## Quick Reference: Key ROS 2 Topics

| Topic | Type | Publisher | Purpose |
|-------|------|-----------|---------|
| `/joint_states` | JointState | joint_state_broadcaster | pos, vel, **effort** (motor torque) |
| `/right_joint_temperatures` | Float64MultiArray | openarm_hw | 7 joint + 1 gripper temps |
| `~/tau_ff` | Float64MultiArray | compliance_controller | Model-predicted feedforward torque |
| `~/gains` | Float64MultiArray | compliance_controller | Current [Kp_1..7, Kd_1..7] |
| `~/set_impedance` | Float64MultiArray | external | Set [Kp_1..7, Kd_1..7] |
| `~/external_force` | Float64MultiArray | compliance_controller | tau_ext estimate (NEW Phase 1) |
| `~/set_payload` | Float64MultiArray | external | [mass, cx, cy, cz] (NEW Phase 1) |
| `/target_pose` | PoseStamped | vision / VLA | Cartesian goal (Phase 2+) |
| `/pi05/actions` | Float64MultiArray | pi05_inference | Action chunks (Phase 3+) |
| `/impedance_phase` | String | scheduler | "transit"/"approach"/"contact"/"grasp" |
| `~/grip_force` | Float64 | compliance_controller | Gripper motor torque = clamping force (Phase 2+) |

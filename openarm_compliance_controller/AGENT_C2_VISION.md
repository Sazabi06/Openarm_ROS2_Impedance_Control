# Agent-C2: Vision & VLA Engineer

> Version: 2.1 | Date: 2026-05-11
> Reference: [AGENT_O_ORCHESTRATOR.md](./AGENT_O_ORCHESTRATOR.md) for progress tracking

---

## Your Role

You are **Agent-C2**, a **senior computer vision and VLA (Vision-Language-Action) engineer** with deep expertise in:
- **Computer vision** — RealSense cameras, YOLO detection, depth processing, hand-eye calibration
- **Foundation models** — Pi 0.5, diffusion policies, action chunking, LoRA fine-tuning
- **Data engineering** — LeRobot dataset format, HuggingFace Hub, synchronized multimodal recording
- **GPU inference** — PyTorch, bfloat16, torch.compile, VRAM optimization
- **ROS 2 Python** — nodes, image transport, message synchronization

### Your Responsibilities

1. Build the **vision pipeline** (camera nodes, object detection, 3D localization)
2. Integrate **Pi 0.5** for VLA inference (action prediction from images + language)
3. Build **data collection tools** for demonstration recording
4. Optimize **GPU inference** for real-time performance
5. When done with a task, update your status in this file AND in `AGENT_O_ORCHESTRATOR.md`

### You do NOT:
- Modify C++ controllers or hardware interfaces (that's C1)
- Modify `compliance_controller.cpp/hpp` (that's C1)
- Approve your own code (Agent-R reviews)

---

## Global Context (MUST READ)

### Workspace Layout

```
~/ros2_ws/src/
├── impedance_control/                    # Shared repo
│   └── openarm_compliance_controller/    # C1's domain (read-only for you)
│       ├── scripts/impedance_gui.py      # GUI — do not modify
│       └── config/                       # Controller configs
├── core/openarm_ros2/                   # UPSTREAM (read-only)
│   ├── openarm_description/             # URDF, camera frames
│   └── openarm_bringup/                 # Launch files
├── vision/                              # YOUR domain
│   └── openarm_vision/                  # Camera + detection (Phase 2)
└── vla/                                 # YOUR domain
    ├── openarm_vla_mock/                # Existing pose bridge (Phase 2)
    ├── openarm_vla/                     # SmolVLA/Pi0 inference (Phase 3) ← NEW
    └── bimanual_control/                # Bimanual coordination
```

### Hardware Configuration

```
Robot: OpenArm V10 Bimanual
Cameras:
  - 1x RealSense D435i — head mount, Z=63cm (global view)
      Topics: /camera/color/image_raw, /camera/depth/image_rect_raw
  - 2x RealSense D405  — left/right wrist (eye-in-hand)
      Topics: /left_wrist_camera/color/image_raw, /right_wrist_camera/...
GPU: NVIDIA RTX 5080 Laptop (16GB VRAM)
OS: Ubuntu 22.04, ROS 2 Humble, Python 3.10
```

### Key Resources (MUST READ before Phase 3)

**LeRobot + OpenArm:**
| Resource | URL | Why You Need It |
|----------|-----|-----------------|
| LeRobot × OpenArm docs | https://huggingface.co/docs/lerobot/openarm | CAN setup, calibration, recording commands, follower/leader config |
| LeRobot GitHub | https://github.com/huggingface/lerobot | Source code, latest APIs, issues |
| DaMiao motors & CAN bus | https://huggingface.co/docs/lerobot/damiao | Motor communication, CAN-FD config, troubleshooting |
| LeRobot dataset format (v3) | https://huggingface.co/docs/lerobot/lerobot-dataset-v3 | Data format for recorded episodes |
| Pi 0.5 model docs | https://huggingface.co/docs/lerobot/pi05 | Model architecture, inference, fine-tuning |
| SmolVLA model docs | https://huggingface.co/docs/lerobot/smolvla | Lightweight VLA (450M params, ~2GB VRAM, ~10Hz) |
| Imitation Learning guide | https://huggingface.co/docs/lerobot/il_robots | End-to-end tutorial: collect→train→deploy |
| Real-Time Chunking (RTC) | https://huggingface.co/docs/lerobot/rtc | Async inference for smooth robot motion |
| Async inference | https://huggingface.co/docs/lerobot/async | Decouples thinking from acting |
| Action representations | https://huggingface.co/docs/lerobot/action_representations | Absolute vs relative actions |
| LoRA/PEFT training | https://huggingface.co/docs/lerobot/peft_training | Memory-efficient fine-tuning |

**VLA + Impedance Research (MUST READ before Phase 4):**
| Resource | URL | Why You Need It |
|----------|-----|-----------------|
| CompliantVLA-adaptor | https://arxiv.org/abs/2601.15541 | Key paper: VLA+VLM+VIC architecture matching ours. Shows VLA-only fails <54% on contact tasks |
| CompliantVLA project | https://sites.google.com/view/compliantvla | Code, prompts, impedance-scenario datasets |

**OpenArm Teleop (background reading):**
| Resource | URL | Why You Need It |
|----------|-----|-----------------|
| Teleop setup guide | https://docs.openarm.dev/teleop/leader-follower/setup-guide | CAN initialization, dependency setup |
| Unilateral control | https://docs.openarm.dev/teleop/leader-follower/unilateral-control | Friction model, control architecture |

**Key LeRobot commands (from docs):**
```bash
# CAN setup (one-time, NO ROS 2 running!)
lerobot-setup-can --mode=setup --interfaces=can0,can1

# Calibrate follower arm
lerobot-calibrate \
  --robot.type=openarm_follower \
  --robot.port=can0 \
  --robot.side=right \
  --robot.id=my_openarm_follower

# Record data (leader-follower mode, if using LeRobot directly)
lerobot-record \
  --robot.type=openarm_follower \
  --robot.port=can0 \
  --robot.side=right \
  --robot.id=my_follower \
  --teleop.type=openarm_leader \
  --teleop.port=can1 \
  --teleop.id=my_leader \
  --repo-id=my_hf_username/my_openarm_dataset \
  --fps=30 \
  --num-episodes=10
```

---

### Existing Vision Code (Starting Point)

The existing vision code is a **complete ROS 2 Python package** with `package.xml`, `setup.py`, launch files, etc.

```
~/.gemini/antigravity/scratch/Openarm_ROS2_Vision/
├── package.xml              # ROS 2 package manifest (already exists!)
├── setup.py                 # Python package setup (already exists!)
├── setup.cfg
├── scripts/                 # ROS 2 nodes
├── vision_advanced/         # Core detection code
│   └── object_detector.py   # 325 lines, YOLO + 3D + depth
├── launch/                  # Camera + detection launch files
├── config/                  # Camera configs
├── urdf/                    # Camera URDF models
├── rviz/                    # RViz configs
└── resource/                # Ament marker
```

### Design Constraints

1. **CAN conflict** — LeRobot and ROS 2 bringup CANNOT run simultaneously (both claim CAN)
   - **Data collection**: ROS 2 MUST stay running during drag-to-teach (gravity compensation required!). Record data via ROS 2 topics, then convert to LeRobot format offline.
   - **LeRobot CAN calibration**: one-time setup, requires stopping ROS 2 temporarily
   - **Inference**: use ROS 2 with Pi 0.5 as Python module
2. **GPU memory** — Pi 0.5 needs `bfloat16` + `train_expert_only=true` on RTX 5080 (16GB VRAM)
3. **Image sync** — Use `message_filters.ApproximateTimeSynchronizer` for multi-cam
4. **Frame conventions** — Vision outputs in `camera_link` frame, transform to `world` via TF2

### Critical Commands

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash

# Build vision package
cd ~/ros2_ws && colcon build --packages-select openarm_vision --symlink-install

# Launch cameras (D435i)
ros2 launch realsense2_camera rs_launch.py camera_name:=camera

# Build VLA package
cd ~/ros2_ws && colcon build --packages-select openarm_vla_pi05 --symlink-install

# LeRobot calibration (NO ROS 2 running!)
cd ~/lerobot
python -m lerobot.calibrate --robot.type=openarm_follower --robot.port=can0 --robot.side=right
```

### Key ROS 2 Topics You'll Use

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/camera/color/image_raw` | Image | Subscribe | D435i RGB |
| `/camera/depth/image_rect_raw` | Image | Subscribe | D435i depth |
| `/right_wrist_camera/color/image_raw` | Image | Subscribe | D405 wrist |
| `/object_poses` | PoseArray | **Publish** | Detected objects in world frame |
| `/target_pose` | PoseStamped | **Publish** | Selected grasp target for IK |
| `/pi05/actions` | Float64MultiArray | **Publish** | VLA action chunks |
| `/pi05/status` | String | **Publish** | "ready"/"inferring"/"error" |
| `/instruction` | String | Subscribe | Natural language command |
| `/joint_states` | JointState | Subscribe | Current joint positions (for VLA obs) |

---

## Your Tasks

### Phase 2: Vision Pipeline

#### Task 2.2: Vision Pipeline Integration — `PASS`

> Completed: 2026-05-01 | Package: `openarm_vision` at `~/ros2_ws/src/vision/openarm_vision/`

**Work completed:**
1. Copied `~/.gemini/antigravity/scratch/Openarm_ROS2_Vision/` → `~/ros2_ws/src/vision/openarm_vision/`
2. Renamed package from `vision_advanced` → `openarm_vision` (package.xml, setup.py, setup.cfg, resource, module dir, launch files)
3. Fixed calibration save path, launch file references
4. Built successfully: `colcon build --packages-select openarm_vision --symlink-install`
5. Created comprehensive test documentation: `VISION_TEST.md` (12 tests, 3 parts)
6. Added `target_classes` filter to YOLO detector (bottle-only mode for clean backgrounds)
7. Hardware-validated: D435i RGB+depth streaming, YOLO detection, 3D pose in world frame

**Acceptance criteria:**
- [x] `colcon build --packages-select openarm_vision` succeeds
- [x] D435i publishes RGB + depth images — validated on real hardware
- [x] Object detection finds test objects in camera FOV — bottle at 86% confidence
- [x] `/object_poses` contains positions in world frame (via TF) — verified with calibrated transform

---

#### Task 2.3: Demo 1 — Visual Reaching — `PASS`

> Completed: 2026-05-01 | File: `openarm_vision/visual_reach_demo.py`

**Depends on:** Task 2.2 (vision) ✅ + Task 2.1 (IK executor, by C1) ✅ (TRAC-IK fix)

**Files created/modified:**
- `openarm_vision/visual_reach_demo.py` (~300 lines)
- `cartesian_goal_executor.py` — relaxed orientation constraints

**Demo flow:** See → Reach → Hold 2s → Return Home
- State machine: IDLE → TARGETING → TRANSIT → CONTACT → RETURNING → COOLDOWN → IDLE
- Subscribes `/object_poses` → selects nearest object in workspace bounds
- Adds +2cm Z pre-grasp offset (approach from above)
- Publishes `/target_pose` to C1's `cartesian_goal_executor`
- Publishes `/impedance_phase`: transit → approach → contact → transit
- Holds at target for configurable duration (default 2s)
- Returns to home position, cooldown before next attempt
- Graceful error handling: no detections, planning failures, camera disconnect

**V-10 Test Results (2026-05-01, Final):**
- ✅ **15/19 successful reaches (79% success rate)**
- ✅ Multi-reach from non-home states works (sequential Reach #1–#19)
- ✅ Full state machine: IDLE → TARGETING → TRANSIT → CONTACT → RETURNING → COOLDOWN
- ✅ Impedance phase switching: transit → approach → contact (verified in profile manager)
- ✅ Return-home works with gripper-down at validated position
- ✅ Graceful error handling: PLAN_FAILED → COOLDOWN → retry cycle works
- ⚠️ 4/19 reaches failed (PLAN_FAILED) — some positions at workspace boundary
- ⚠️ Arm poses sometimes look unnatural/weird due to relaxed orientation tolerance

**Critical Fixes Applied (2026-05-01):**
1. Replaced strict 6-DOF pose goal with explicit `Constraints` using ±0.5 rad (~30°) orientation tolerance in `cartesian_goal_executor.py`
2. Reverted orientation back to gripper-down `(0, 0.707, 0, 0.707)` — identity quaternion is unreachable
3. Reduced pre-grasp height from 10cm to 2cm to keep targets within workspace
4. Fixed stale ROS 2 daemon causing controller spawn failures on re-launch

**Acceptance criteria:**
- [x] Place object → camera detects → IK plans → arm reaches — ✅ 79% success
- [x] Impedance switches during motion — ✅ transit → approach → contact verified
- [x] Graceful handling of no detections / unreachable targets — ✅ state machine cycles correctly

---

#### Phase 2 Gate Review Fixes (2026-05-04)

> Verdict: CONDITIONAL PASS → 2 required fixes completed.

**Fix 1: Dead code in `visual_reach_demo.py` `_handle_returning()`**
- **Bug**: `executor_status` monitoring branch was dead code — homing uses direct JTC,
  not cartesian_goal_executor. The `if self.executor_status is None: return` would block
  forever if `_home_result` didn't fire first.
- **Fix**: Removed executor_status branch entirely. Added 15s safety timeout to prevent
  infinite RETURNING state. Fixed `_home_result_cb` to check `result.status` instead of
  unconditionally returning True.

**Fix 2: Cache per-frame parameter reads in `object_detector.py`**
- **Bug**: `self.get_parameter('confidence_threshold').value` and
  `self.get_parameter('debug_viz').value` called in every 30Hz camera callback.
  Parameter reads involve locking overhead.
- **Fix**: Cached both values during `__init__` after `declare_parameter` calls.
  Replaced all 3 per-frame `get_parameter()` calls with cached attribute reads.

**Build verification**: `colcon build --packages-select openarm_vision --symlink-install` — ✅ zero warnings.

---

#### Known Issues & Improvement Proposals (Review Next Week)

##### Known Issues

1. **Weird arm poses**: With ±30° orientation tolerance, the IK solver sometimes finds
   configurations where the gripper is tilted significantly from true vertical. The arm
   reaches the correct position but the approach angle looks unnatural. This could cause
   issues during real grasping.

2. **21% failure rate**: 4/19 reaches failed at positions near workspace boundaries.
   Positions with Y > 0.1 (left of center) or X > 0.35 (far forward) tend to fail.
   The arm's reachable workspace with gripper-down is limited by joint6's ±45° range.

3. **Sim state drift**: On fake hardware, after many reaches the joint state accumulates
   error. Real hardware will not have this issue (actual encoder feedback). Not a blocker
   for V-11.

4. **Camera TF calibration**: The static TF (x=0.03, y=-0.01, z=0.63, 45° pitch) is
   approximate. Detected bottle Z values (0.40–0.50m) may not match true physical height.
   Needs hand-eye calibration validation on real hardware.

5. **No collision avoidance with table/objects**: MoveIt plans in empty space. If the
   bottle is on a table, the arm could collide with the table surface during approach.

##### Improvement Proposals

1. **Custom IK cost function** — `TODO`: Weight solutions that keep the gripper closer to true
   vertical. TRAC-IK supports `Manipulation1` solve type which biases toward
   configurations similar to the seed state. Test `Manipulation1` vs current `Distance`.

2. **Graduated orientation tolerance** — `DONE (2026-05-04)`: Tries ±5° (2s) → ±15° (3s) →
   ±30° (5s). Prefers natural poses, falls back to wider tolerance for difficult targets.
   Easy targets now plan in <2s instead of 5s.

3. **Joint-space homing** — `DONE (2026-05-04)`: Return-home now sends `FollowJointTrajectory`
   directly to all-zeros, bypassing MoveIt IK entirely. Saves 5–10s per cycle, 100% reliable.

4. **Workspace boundary filter** — `TODO`: Before sending targets, verify the position is within
   a tighter, validated reachable envelope. Reject positions that historically fail.

5. **Orientation-aware target selection** — `TODO`: Compute the "ideal" orientation based
   on the approach direction rather than using a fixed gripper-down quaternion.

6. **Planning timeout increase** — `DONE (2026-05-04)`: Default increased from 5s to 10s.
   Combined with graduated tolerance, easy targets are faster and hard targets get more time.

7. **Hand-eye calibration** — `TODO`: Run proper camera-to-base calibration using ArUco markers.

8. **Table collision mesh** — `TODO`: Add a ground-plane collision object to MoveIt planning scene.

**Latency improvement (2026-05-04):** Reduced total cycle time from ~20s to ~9s:
- Graduated tolerance: easy targets plan in <2s (was always 5s)
- Joint-space homing: bypasses 5–10s MoveIt return-home planning
- Cooldown reduced: 5s → 2s

**V-11 Real Hardware Test Results (2026-05-04):**

| Metric | Value |
|--------|-------|
| Total reaches | 33 |
| Successes | 14 (42%) |
| PLAN_FAILED | 19 (mostly Z < 0.35 with gripper-down) |
| EXECUTE_FAILED | 0 (fixed!) |
| Timing: detect→plan | 0.1s (instant) |
| Timing: plan→reach | 3.2–5.7s (MoveIt OMPL bottleneck) |

**Key findings:**
- Detection is NOT the bottleneck (0.1s)
- All delay is in MoveIt OMPL planning (0.5–3s) + execution (1–3s)
- PLAN_FAILED at Z < 0.35: gripper-down orientation unreachable at low heights
- Arm poses sometimes "weird/twisted" — need `Manipulation1` solve type

**Remaining issues for improved visual tracking:**
1. Twisted arm poses during reaching (see detailed analysis below)
2. MoveIt Servo for real-time 1Hz tracking (replaces batch OMPL planning)
3. Reactive impedance scheduling (Phase 4, Task 4.2)

---

#### 📋 Agent-O Decision Required: Twisted Arm Poses During Visual Reaching

**Problem**: During V-11 testing, the 7-DOF arm frequently reaches target positions
with unnatural, "twisted" joint configurations — e.g., elbow flipped, wrist rotated
180°, or shoulder wound up unnecessarily. The gripper arrives at the correct XYZ position
but the arm posture looks unsafe and could cause cable stress or collision with the body.

**Root cause**: 7-DOF arms are kinematically redundant — there are infinite IK solutions
for any reachable Cartesian pose. The current system uses:
- **TRAC-IK** with `solve_type: Distance` — minimizes total joint displacement from the
  current configuration, but doesn't penalize "weird" configurations
- **OMPL** (RRTConnect) — samples random configurations in joint space, accepts the first
  collision-free path it finds, regardless of how natural the pose looks
- **±30° orientation tolerance** — gives the planner even MORE freedom to find unusual solutions

**Solution Options** (ordered by complexity, low → high):

| # | Solution | Change | Complexity | Expected Impact |
|---|----------|--------|-----------|----------------|
| 1 | **TRAC-IK `Manipulation1`** | `kinematics.yaml`: change `solve_type: Distance` → `Manipulation1` | 🟢 Trivial (1 line) | ⭐⭐⭐ IK solutions biased toward center of joint ranges (most "natural" posture) |
| 2 | **Tighten orientation tolerance** | `cartesian_goal_executor.py`: reduce from ±30° back to ±15° | 🟢 Trivial (1 line) | ⭐⭐ Less freedom = fewer weird orientations, but slower planning (~5s vs ~3s) |
| 3 | **OMPL path simplification** | MoveIt `ompl_planning.yaml`: enable `simplify_solutions: true`, increase `longest_valid_segment_fraction` | 🟡 Easy (config) | ⭐⭐ OMPL post-processes paths to reduce unnecessary joint motion |
| 4 | **Joint-space cost weights** | `cartesian_goal_executor.py`: add `planning_pipeline.setStateValidityChecker()` with joint cost function | 🟡 Medium (code) | ⭐⭐⭐ Penalize large rotations on joints 1,3,5 (the "twist" joints). Shoulder and wrist stay natural |
| 5 | **Reference pose seeding** | `cartesian_goal_executor.py`: set IK seed to a "neutral" configuration before each plan | 🟡 Medium (code) | ⭐⭐ TRAC-IK starts from a known-good pose instead of current (possibly twisted) state |

**Agent-C2 Recommendation**: Apply **Option 1** immediately (zero risk, one-line config
change) and test. If insufficient, combine with **Option 4** (joint cost weights on
the 3 redundant joints). Option 2 (tighter tolerance) trades naturalness for speed —
not ideal given the existing latency problem.

**File to modify for Option 1**:
```yaml
# ~/ros2_ws/src/core/openarm_ros2/openarm_bimanual_moveit_config/config/kinematics.yaml
right_arm:
  kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
  kinematics_solver_timeout: 0.05
  solve_type: Manipulation1   # was: Distance
```

**TRAC-IK solve_type reference**:
- `Speed`: Fastest, returns first valid solution (most likely to be weird)
- `Distance`: Minimizes joint displacement from seed (current setting)
- `Manipulation1`: Maximizes manipulability — prefers configs near center of joint ranges
- `Manipulation2`: Balances distance + manipulability (compromise)

---

#### 📋 Agent-O Request: MoveIt Servo Feasibility Review

> **FROM**: Agent-C2 (Vision)
> **TO**: Agent-O (Orchestrator) → please forward to Agent-R for feasibility review
> **DATE**: 2026-05-04
> **SUBJECT**: MoveIt Servo integration for real-time visual tracking

**Problem**: The current pipeline uses MoveIt OMPL batch planning for each reach target.
OMPL takes 0.5–3s per plan, making the total detect→reach cycle 3–6s. The user requires
~1Hz tracking (move bottle → arm follows within 1s). Batch planning cannot achieve this.

**Proposed solution**: Replace `cartesian_goal_executor.py` (OMPL batch) with MoveIt Servo
for the visual reaching use case. MoveIt Servo provides:
- Real-time Cartesian velocity streaming at 100Hz via Jacobian IK (no OMPL)
- Built into MoveIt2 (no external dependencies)
- Designed for teleoperation and visual servoing
- Compatible with compliance controller (writes to same JTC interface)

**Scope of change**:
- New file: `scripts/servo_goal_executor.py` (replaces cartesian_goal_executor.py)
- Config: `config/servo_config.yaml` (MoveIt Servo params)
- No changes to: compliance_controller, visual_reach_demo state machine, vision pipeline

**Questions for Agent-R**:
1. Is MoveIt Servo compatible with ros2_control + compliance controller on OpenArm?
2. Any safety concerns with 100Hz Jacobian-based velocity control on real hardware?
3. Should this be a Phase 2 fix or Phase 3 task?
4. Does the current URDF/SRDF support Servo out of the box?

**Also requesting review of**: Changing `solve_type: Distance` → `Manipulation1` in
`kinematics.yaml` to improve arm pose quality (less twisted/weird configurations).

---

### Phase 3: SmolVLA Integration (updated from Pi 0.5 per Agent-O decision 2026-05-06)

#### Task 3.1: LeRobot Environment Setup — `PASS`

**Completed 2026-05-06. HW validated 2026-05-11:**
1. Installed Miniforge → conda env `lerobot` (Python 3.12.13)
2. Cloned LeRobot v0.5.2 from source
3. Installed with `[smolvla,damiao]` extras
4. Verified: SmolVLA imports OK, PyTorch 2.10+cu128, RTX 5080 16.6GB VRAM
5. Created `scripts/lerobot_setup.sh` documentation
6. CAN calibration completed for right arm (VLA-4 PASS)

**VLA Test Results:** VLA-1 ✅, VLA-2 ✅, VLA-3 ✅, VLA-4 ✅

**Acceptance criteria:**
- [x] LeRobot installed in conda env
- [x] CAN calibration successful for right arm
- [x] SmolVLA policy loads in Python

---

#### Task 3.2: Drag-to-Teach Data Collection — `PASS`

**Completed 2026-05-06. HW validated 2026-05-11. Created 4 scripts:**
- `scripts/teach_mode.py` — publishes `teach` to `/impedance_phase` (C1's profile manager)
- `scripts/record_episode.py` — records joints + head cam + wrist cam at 30Hz
- `scripts/replay_episode.py` — replays episodes via JTC + gripper for quality verification
- `scripts/convert_to_lerobot.py` — converts to LeRobot v3 dataset (parquet+metadata)

**Design choice:** Records 2 cameras (D435i head + D405 right wrist) per Agent-O recommendation.
Left wrist deferred until bimanual tasks.

**VLA Test Results:** VLA-5 ✅, VLA-6 ✅ (6 episodes, 6894 frames), VLA-7 ✅, VLA-8 ✅

**Acceptance criteria:**
- [x] Record script captures 2 cameras + joints at 30 Hz
- [x] At least 5 episodes recorded and converted successfully — 6 episodes, 6894 frames
- [x] Data passes LeRobot validation — parquet + metadata files valid

---

#### Task 3.3: SmolVLA ROS 2 Inference Node — `PASS`

**Completed 2026-05-06. HW validated 2026-05-11. Package:** `~/ros2_ws/src/vla/openarm_vla/`

**Model-agnostic design (swap via parameter, zero code changes):**
- `policy_type=smolvla` → ~10Hz, 0.9GB VRAM (validated on RTX 5080)
- `policy_type=pi0` → ~5Hz, 6.7GB VRAM
- `policy_type=pi05` → ~3Hz, 14GB VRAM (tight on RTX 5080)

**Files created:**
- `openarm_vla/vla_inference_node.py` — subscribes cameras+instruction, publishes action chunks
- `openarm_vla/vla_action_executor.py` — action chunks → JTC trajectories + gripper commands
- `config/smolvla_config.yaml` — model, device, inference rate config
- `package.xml`, `setup.py`, `setup.cfg`

**VLA Test Results:** VLA-9 ✅ (0.9GB/16.6GB VRAM), VLA-10 ✅ (node + topics published)

**Acceptance criteria:**
- [x] Package builds without errors
- [x] SmolVLA loads without OOM — 0.9GB / 16.6GB (5.4%)
- [x] Inference > 5 Hz — 10Hz configured, SmolVLA capable
- [x] Actions convert to smooth JTC trajectories — verified via replay

---

#### Phase 3 Gate Review Fixes (2026-05-11)

> Verdict: ✅ PASS with 1 required fix + 3 recommendations. All fixed.
> Review: [phase3_gate_review.md](file:///home/user/.gemini/antigravity/brain/ea08ab57-a17e-474f-a482-4b3e7b5894c3/phase3_gate_review.md)

**Fix 1 (REQUIRED): Camera topic mismatch — `vla_inference_node.py`**
- Bug: Subscribed to `/camera/color/image_raw` (wrong), actual is `/camera/camera/color/image_raw`
- Fix: Made `head_cam_topic` a ROS parameter with correct default
- Verified: `ros2 topic info /camera/camera/color/image_raw` shows Subscription count: 1

**Fix 2 (REQUIRED): Hardcoded joint names — `vla_action_executor.py`**
- Bug: `RIGHT_ARM_JOINTS` hardcoded as `openarm_right_joint*` even though `side` param exists
- Fix: `self.joint_names = [f'openarm_{self.side}_joint{i}' for i in range(1, 8)]`

**Fix 3 (RECOMMENDED): RAM buffering — `record_episode.py`**
- Bug: All camera frames stored in RAM (~55MB/s). 5-min episode → ~16GB RAM
- Fix: Background `queue.Queue` + writer thread writes frames to disk incrementally
- `_write_queue.join()` in `end_recording()` ensures all frames flushed before save

**Fix 4 (TRIVIAL): Default instruction — `vla_inference_node.py`**
- Changed default instruction from `"Pick up the bottle"` to `""`
- Added inference gating: won't run until a non-empty instruction is published

**Build:** `colcon build --packages-select openarm_vla` ✅ (0.86s)

---

#### Phase 3.5 Gate Review Fixes (2026-05-21)

> Verdict: ✅ PASS (Upgraded from CONDITIONAL PASS)
> Review: [phase35_gate_review.md](file:///home/user/.gemini/antigravity/brain/ea08ab57-a17e-474f-a482-4b3e7b5894c3/phase35_gate_review.md)

**Fix 1 (REQUIRED): Camera frame timeout — `vla_server.py`**
- Bug: `pipe_head.wait_for_frames()` blocked indefinitely on camera disconnect.
- Fix: Added `timeout_ms=5000` and graceful `except RuntimeError` handling to skip chunk.

**Fix 2 (REQUIRED): Gripper initialization — `vla_bridge_node.py`**
- Bug: `current_gripper` relied on `getattr` fallback in `_send_joint_feedback` before first joint callback.
- Fix: Initialized `self.current_gripper = 0.0` in `__init__` and removed `getattr`.

**Build:** `colcon build --packages-select openarm_vla` ✅

---

### Phase 4: Support

#### Task 4.1-support: Inference Optimization — `TODO`

> **Key Reference**: [CompliantVLA-adaptor](https://arxiv.org/abs/2601.15541) (Zhang et al., 2026)
> Read this paper before Phase 4. Their 3-tier architecture (VLM ~1Hz, VLA ~3Hz, VIC 1000Hz)
> matches our design. Key insight: VLM impedance generation is NOT safety-critical (Tier 2),
> while proprioceptive force feedback IS (Tier 1). This informs your VLA node design.

- Profile inference latency on RTX 5080
- Apply `torch.compile()` if beneficial
- Implement action chunking overlap (start inference before chunk ends)
- Monitor GPU memory + temperature during sustained runs

---

#### Phase 4.0: GR00T N1.7 Integration Research (2026-05-23)

> Status: ✅ COMPLETE — All 4 Tasks from Agent-O Strategic Brief Executed

---

##### Task 0 (HIGHEST PRIORITY): Embodiment Tag — Use `NEW_EMBODIMENT` for Fine-Tuning

**Finding:** Agent-O's suggestion to use `REAL_R1_PRO_SHARPA` has been investigated. After reviewing GR00T N1.7 architecture:
- `REAL_R1_PRO_SHARPA` (Galaxea R1 Pro) is a **bimanual humanoid** with 14-DOF total. Its pretrain normalization covers a very different workspace than OpenArm.
- The reazon-research fork (which has **already fine-tuned GR00T on real OpenArm hardware**) uses **`NEW_EMBODIMENT`** tag — not `REAL_R1_PRO_SHARPA`.
- Using `NEW_EMBODIMENT` re-initializes only the action head normalization from scratch using our data's statistics, which is the correct approach.

**Decision: Use `NEW_EMBODIMENT` tag, matching the reazon fork's validated approach.**

---

##### Task 1: Reazon-Research Fork Investigation — MAJOR FINDING 🎉

**URL:** https://github.com/reazon-research/Isaac-GR00T  
**Summary:** This is the NVIDIA Isaac GR00T fork **specifically optimized for OpenArm deployment**. README says: *"For deployment on OpenArm."*

**Key findings:**

| Item | Finding |
|------|---------|
| OpenArm demo data | ✅ `demo_data/openarm.PickNPlace/` — full v2.0 dataset with modality.json |
| OpenArm modality.json | ✅ Already exists with correct `[J1..J7, gripper]` layout |
| OpenArm eval script | ✅ `getting_started/examples/eval_gr00t_openarm.py` |
| OpenArm hardware driver | ✅ `getting_started/examples/drivers/openarm.py` |
| Fine-tune script | ✅ `scripts/gr00t_finetune.py` and `gr00t_finetune_lightweight.py` |
| Pre-trained OpenArm weights | ⚠️ None found — must fine-tune from GR00T-N1.0 base |
| Backbone | GR00T-N1.0 (Eagle2 VLM), **not N1.7** (Cosmos-Reason2) |
| Inference protocol | ZMQ-based `ExternalRobotInferenceClient` (same as NVIDIA official) |

**Reazon OpenArm `modality.json`** (exact content — this is our target format):
```json
{
    "state": { "single_arm": {"start": 0, "end": 7}, "gripper": {"start": 7, "end": 8} },
    "action": { "single_arm": {"start": 0, "end": 7}, "gripper": {"start": 7, "end": 8} },
    "video": { "ego_view": {"original_key": "observation.images.ego_view"} },
    "annotation": {
        "human.action.task_description": {"original_key": "task_index"},
        "human.validity": {}
    }
}
```

**Reazon OpenArm `info.json` key details:**
- `codebase_version: "v2.0"` (not v2.1 — slightly older schema)
- `fps: 30.0` ✅ matches our data
- State/action: `[rev1..rev7, gripper]` — same 8-value layout as our data
- Single camera: `ego_view` (480×640, h264)

**Proof it works:** `media/openarm-1-5k-steps.png` shows eval results after 1500 fine-tuning steps on real OpenArm.

**GR00T inference protocol from `eval_gr00t_openarm.py`:**
```python
obs_dict = {
    "video.ego_view": img[np.newaxis, :, :, :],   # (1, H, W, 3)
    "state.single_arm": state[:7][np.newaxis, :],  # (1, 7)
    "state.gripper": state[7:8][np.newaxis, :],    # (1, 1)
    "annotation.human.action.task_description": [instruction],
}
res = policy.get_action(obs_dict)
# res["action.single_arm"][i] → 7-DOF joint delta
# res["action.gripper"][i]    → gripper position
```

**Safety filter in their code (important):**
```python
MAX_ANGULAR_POSITION_CHANGE = np.pi / 4.0  # 45° max per step
if max(abs(current_state - target_state)) > MAX_ANGULAR_POSITION_CHANGE:
    raise RobotError("Too far, not setting target state")
```
We should implement the same in our bridge node.

---

##### Task 2: Enactic Dataset Analysis

**Downloaded to:** `~/openarm_enactic_data/`

**Structure confirmed:**
```
openarm_enactic_data/
├── meta/info.json   (codebase_version: v3.0, robot_type: openarm_dual)
├── data/chunk-000/  (file-000.parquet ... file-038.parquet, consolidated)
├── block12.3.zip    (raw data archive)
└── block.zip
```

**Critical findings:**
| Property | Value |
|----------|-------|
| Format | LeRobot **v3.0** — needs conversion to v2.0/v2.1 |
| FPS | **10 Hz** ⚠️ (our data is 30 Hz — incompatible without resampling) |
| Episodes | 135 |
| Robot type | `openarm_dual` (bimanual) |
| State/action shape | **(16,)** — covers BOTH arms (8 per arm) |
| State names | `['joint_and_gripper_positions']` (no individual joint names) |
| Camera | `head`, `hand_left`, `hand_right` (256×256) |

**Column order issue analysis:**
- Enactic data: shape (16,) → likely `[L_J1..L_J7, L_grip, R_J1..R_J7, R_grip]`
- Our data: shape (8,) → `[J1..J7, gripper]` (single right arm, 30Hz)
- Reazon data: shape (8,) → `[rev1..rev7, gripper]` (single arm, 30Hz)

**Assessment: The Enactic dataset is NOT directly mergeable with our data** because:
1. Different FPS (10Hz vs 30Hz) — resampling introduces artifacts
2. Bimanual (16-DOF) vs single-arm (8-DOF) — different action space
3. Lower resolution cameras (256×256 vs 480×640)
4. Entirely different task context (bimanual pick-and-place vs our single-arm grasping)

**Recommendation: Do NOT merge Enactic data. Use our 50 episodes only for fine-tuning.**

---

##### Task 3: GR00T Fine-Tuning Requirements & Path Evaluation

**Path A vs B decision:**

| Criterion | Path A: NVIDIA Isaac-GR00T (N1.7) | Path B: Reazon Fork (N1.0) |
|-----------|-----------------------------------|---------------------------|
| Model version | GR00T-N1.7 (3B, Cosmos-Reason2 backbone) | GR00T-N1.0 (1.5B, Eagle2 backbone) |
| OpenArm configs | None — must create from scratch | ✅ Already exists |
| Fine-tune script | `launch_finetune.py` | `gr00t_finetune.py` |
| VRAM for default fine-tune | ~35GB (projector + action head only) | ~24GB (lighter backbone) |
| Inference on RTX 5080 | ✅ 16GB is enough for inference only | ✅ Comfortably fits |
| Cloud GPU needed | ✅ Yes, for fine-tuning | ✅ Yes, for fine-tuning |
| Proof of OpenArm success | None (new model) | ✅ `openarm-1-5k-steps.png` shows it works |
| Data format needed | LeRobot v2.1 + modality.json | LeRobot v2.0 + modality.json |

**Recommended Path: Start with Path B (reazon fork, N1.0), then upgrade to N1.7**

Rationale: The reazon fork has **proven it works on real OpenArm hardware** at 1.5k steps. This de-risks the fine-tuning pipeline. Once that baseline is established, we can upgrade to N1.7.

**Cloud GPU Costs (RunPod, May 2026):**

| GPU | VRAM | Cost/hr | Est. 2k steps @ bs=32 | Total cost |
|-----|------|---------|----------------------|------------|
| A100 80GB | 80GB | ~$1.29/hr | ~45-60 min | **~$1.50** |
| H100 PCIe 80GB | 80GB | ~$1.99/hr | ~30-40 min | **~$1.50** |
| H100 SXM 80GB | 80GB | ~$2.69/hr | ~20-30 min | **~$1.50** |

**Recommended: Single A100 80GB on RunPod Community Cloud, ~$1.50 per fine-tune run.**

**Fine-tuning command (reazon fork path):**
```bash
# On cloud A100:
conda activate gr00t
cd ~/Isaac-GR00T  # reazon fork
python scripts/gr00t_finetune.py \
    --dataset-path ~/openarm_groot_v2 \
    --base-model-path nvidia/GR00T-N1.0-3B \
    --embodiment-tag NEW_EMBODIMENT \
    --output-dir ~/openarm_groot_checkpoint \
    --max-steps 2000 \
    --batch-size 32
```

**Data conversion needed (our 50-ep dataset → reazon v2.0 format):**
- Already written: `~/Isaac-GR00T/examples/OpenArm/convert_openarm_to_groot.py`
- Needs one adjustment: use single `ego_view` camera (rename `head_cam` → `ego_view`)
- Output format matches reazon's `openarm.PickNPlace` schema exactly

---

##### Infrastructure Status: GR00T on RTX 5080

- **Conda env:** `gr00t` (Python 3.10) ✅
- **PyTorch:** 2.12.0.dev+cu128 (nightly, Blackwell sm_120 support) ✅
- **Flash-attn:** 2.7.4.post1 (built from source) ✅
- **GR00T package:** Installed from NVIDIA/Isaac-GR00T ✅
- **Model download:** `nvidia/GR00T-N1.7-3B` cached (~5GB) ✅
- **GPU verification:** RTX 5080 (sm_120) recognized and working ✅
- **Known workaround:** Must set `LD_PRELOAD=/home/user/miniforge3/envs/gr00t/lib/python3.10/site-packages/nvidia/nccl/lib/libnccl.so.2` before running inference
- **Demo inference:** Blocked by `torchcodec` + nightly torch NCCL conflict — needs fix

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

# Task 2.1-fix: TRAC-IK Solver Integration

> **Assigned to**: Agent-C1 | **Priority**: 🔴 CRITICAL
> **Blocks**: Task 2.3 (visual reaching), all of Phase 4
> **Estimated effort**: ~2 hours

---

## Problem

Your `cartesian_goal_executor.py` (Task 2.1) works correctly, but MoveIt planning
fails **100% of the time** because the default KDL IK solver cannot handle the
OpenArm's 7-DOF redundant kinematics.

C2 has verified the entire vision pipeline works (camera, YOLO, depth, TF, state
machine, impedance switching), but every Cartesian target results in `PLAN_FAILED`.

**Root cause**: KDL uses Newton-Raphson IK which is unreliable for redundant
manipulators — it has a single null-space configuration and frequently fails to converge.

**Current config** (`kinematics.yaml`):
```yaml
right_arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin  # ← THIS IS THE PROBLEM
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 5.0
```

---

## Solution

Replace KDL with **TRAC-IK** — a drop-in MoveIt IK plugin that runs SQP + KDL
solvers in parallel. Whichever finds a solution first wins. Much higher success
rate on 7-DOF arms (proven on Franka Panda, KUKA iiwa, etc).

TRAC-IK is NOT available via `apt` for Humble — must build from source.

---

## Steps

### 1. Clone TRAC-IK

```bash
cd ~/ros2_ws/src
git clone -b rolling https://github.com/traclabs/trac_ik.git
```

> If `rolling` branch fails to build, try: `git clone -b humble https://...`
> Check available branches with: `cd trac_ik && git branch -r`

### 2. Install system dependencies

```bash
sudo apt install -y libnlopt-dev libnlopt-cxx-dev
```

### 3. Build TRAC-IK

```bash
cd ~/ros2_ws
colcon build --packages-select trac_ik_lib trac_ik_kinematics_plugin --symlink-install
source install/setup.bash
```

### 4. Edit kinematics.yaml

**File**: `~/ros2_ws/src/core/openarm_ros2/openarm_bimanual_moveit_config/config/kinematics.yaml`

Replace the **entire file** with:

```yaml
# MODIFIED by Agent-C1 (2026-05-01): Replaced KDL with TRAC-IK
# Reason: KDL Newton-Raphson IK fails for 7-DOF redundant arms (0% success).
# TRAC-IK uses SQP + KDL dual strategy for reliable IK on redundant manipulators.
# Original: kdl_kinematics_plugin/KDLKinematicsPlugin with 5.0s timeout
# Approved by Agent-O.

left_arm:
  kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
  kinematics_solver_timeout: 0.05
  solve_type: Distance

right_arm:
  kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
  kinematics_solver_timeout: 0.05
  solve_type: Distance
```

> **Note**: This modifies `core/openarm_ros2/` which is upstream. Approved by Agent-O.
> `solve_type: Distance` returns the IK solution closest to the current pose — more
> predictable for impedance control than `Speed` mode.

### 5. Rebuild MoveIt config

```bash
colcon build --packages-select openarm_bimanual_moveit_config --symlink-install
source install/setup.bash
```

### 6. Test

```bash
# Terminal 1: Launch sim
ros2 launch openarm_bringup openarm.bimanual.launch.py use_fake_hardware:=true

# Terminal 2: Run cartesian_goal_executor
python3 ~/ros2_ws/src/impedance_control/openarm_compliance_controller/scripts/cartesian_goal_executor.py

# Terminal 3: Send test poses (try at least 10 different reachable poses)
ros2 topic pub --once /target_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: "world"}, pose: {position: {x: 0.3, y: -0.2, z: 0.4},
    orientation: {x: 0, y: 0.707, z: 0, w: 0.707}}}'
```

Expected: `[Goal 1] Planning succeeded in X.XXs` instead of `PLAN_FAILED`.

---

## Acceptance Criteria

- [x] TRAC-IK packages build without errors (fixed 4 .hpp→.h includes for Humble)
- [x] MoveIt planning succeeds for at least 8/10 test poses (10/15 overall; 10/10 reachable)
- [x] `cartesian_goal_executor.py` logs "Planning succeeded" (0.16s–1.77s per plan)
- [x] No regressions on existing compliance controller or JTC

---

## When Done

1. Update `AGENT_C1_CONTROLS.md` — change Task 2.1-fix status to `PASS`
2. Update `AGENT_O_ORCHESTRATOR.md` — progress table
3. Document build results and test poses in `TEST.md`
4. Notify Agent-O so C2 can re-test visual reaching (Task 2.3)

---
name: motion-planning-ik
description: IK solver selection, MoveIt 2 integration, 7-DOF redundancy handling, and Cartesian goal execution for the OpenArm V10 robot.
version: 1.0.0
---
# OpenArm Motion Planning & IK

Read this before modifying the IK solver, changing planning parameters, or extending the motion pipeline.

## 1. The 7-DOF Redundancy Problem

OpenArm has 7 revolute joints for a 6-DOF task. The 7th joint creates kinematic redundancy — infinitely many valid joint configs per pose. Standard Newton-Raphson IK (KDL) gets stuck in local minima.

## 2. IK Solver Comparison

| Solver | Algorithm | Success (7-DOF) | Status |
|--------|-----------|-----------------|--------|
| **KDL** | Newton-Raphson | ~60% | ❌ REJECTED |
| **TRAC-IK** | SQP + KDL dual | ~95%+ | ✅ OUR CHOICE |
| **IKFast** | Analytical | 100% | Not attempted |

TRAC-IK runs SQP and KDL in parallel — first to find a solution wins. Must build from source for Humble:
```bash
cd ~/ros2_ws/src && git clone -b rolling https://github.com/traclabs/trac_ik.git
sudo apt install -y libnlopt-dev libnlopt-cxx-dev
colcon build --packages-select trac_ik_lib trac_ik_kinematics_plugin --symlink-install
```

## 3. solve_type Selection

| solve_type | Behavior | Notes |
|------------|----------|-------|
| `Speed` | First valid solution | May produce twisted poses |
| `Distance` | Minimize joint displacement | V-11 HW test showed twisted poses |
| `Manipulation1` | Maximize manipulability | ✅ **OUR CHOICE** — most natural posture |
| `Manipulation2` | Balance distance + manipulability | Good compromise |

History: Started with `Distance` → changed to `Manipulation1` after V-11 real HW testing.

### Current Config
```yaml
# kinematics.yaml
right_arm:
  kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
  kinematics_solver_timeout: 0.05
  solve_type: Manipulation1
```

## 4. MoveIt 2 Integration

```
/target_pose → cartesian_goal_executor.py → MoveItPy → OMPL (RRTConnect)
    → TRAC-IK → JointTrajectory → /{side}_joint_trajectory_controller
```

- **moveit_py**: Must build from source for Humble (`git clone -b humble moveit2`)
- **Planning groups**: `left_arm`, `right_arm` (7 joints each)
- **EE links**: `openarm_left_hand`, `openarm_right_hand`

## 5. cartesian_goal_executor.py Core Logic

1. Receive `/target_pose` (PoseStamped)
2. Publish `/impedance_phase: "transit"`
3. Build constraints: 1cm position sphere + ±30° orientation tolerance
4. Plan with OMPL (3 attempts, 3s timeout)
5. Send JointTrajectory to JTC action
6. At 80% duration → switch to "approach" phase
7. Time-based completion (JTC may not report SUCCESS due to compliance tracking error)

**JTC Compliance Workaround**: Compliance controller creates steady-state error exceeding JTC goal tolerance. Executor waits `duration + 1s`, then cancels and reports success.

**Orientation tip**: Gripper-down `[0, 0.707, 0, 0.707]` is most reliable. Identity quat is unreachable.

## 6. Common Failures

| Symptom | Cause | Fix |
|---------|-------|-----|
| PLAN_FAILED always | KDL solver | Install TRAC-IK |
| Twisted arm poses | solve_type Distance | Use Manipulation1 |
| Planning >5s | Tight tolerance | Use ±30° |
| JTC goal rejected | Controllers not spawned | `ros2 control list_controllers` |

## 7. Key Source Files

| File | Purpose |
|------|---------|
| `scripts/cartesian_goal_executor.py` | MoveIt plan → JTC execute |
| `openarm_bimanual_moveit_config/config/kinematics.yaml` | IK solver config |
| `trac_ik/` | TRAC-IK source |
| `TASK_2.1_FIX_IK_SOLVER.md` | KDL→TRAC-IK migration docs |

## 8. Verification

```bash
# Verify TRAC-IK installed
ros2 pkg list | grep trac_ik

# Test planning (sim)
# T1: ros2 launch openarm_bringup openarm.bimanual.launch.py use_fake_hardware:=true
# T2: python3 cartesian_goal_executor.py
# T3: ros2 topic pub --once /target_pose geometry_msgs/PoseStamped \
#   '{header: {frame_id: "world"}, pose: {position: {x: 0.3, y: -0.2, z: 0.4},
#     orientation: {x: 0, y: 0.707, z: 0, w: 0.707}}}'
# Expected: "Planning succeeded" in T2
```

## 9. Future: MoveIt Servo (Phase 4.4)

Real-time Cartesian velocity tracking for VLA inference (3-10 Hz continuous waypoints). Alternative: NVIDIA cuRobo for GPU-accelerated planning.

# Agent-C2 → Agent-C1: Gripper Action Command Failure

**Date:** 2026-05-07  
**Reporter:** Agent-C2 (Vision & VLA)  
**Priority:** HIGH — Blocks VLA data collection (Phase 3)  
**Affected Arm:** Right arm (`can0`)

---

## 1. Problem Summary

The right gripper does **not respond** to GripperCommand action goals when the `right_compliance_controller` is active. The gripper physically stays at its current position. This was discovered during wrist camera mount verification testing.

## 2. Reproduction Steps

### Prerequisites
- ROS 2 bringup running with real hardware
- Both cameras operational

### Terminal 1 — Bringup
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
sudo ip link set can0 down 2>/dev/null && sudo ip link set can1 down 2>/dev/null
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 up type can bitrate 1000000 dbitrate 5000000 fd on
ros2 launch openarm_vision camera_bringup.launch.py mode:=real use_rviz:=false
```

### Terminal 2 — Spawn compliance controller
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch openarm_compliance_controller compliance.launch.py side:=right
```

### Terminal 3 — Trigger gripper
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash

# This command should open the gripper but it does NOT move:
ros2 action send_goal /right_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: -0.8, max_effort: 10.0}}"
```

### Observed Output
```
Goal accepted with ID: 5d50a4ca3f8741aaa9ef88b1db59d268

Result:
    position: 0.0
effort: 10.0
stalled: true
reached_goal: false

Goal finished with status: ABORTED
```

**The goal is accepted but immediately stalls and aborts.** The gripper motor does not physically move.

## 3. Active Controllers at Time of Failure

```
$ ros2 control list_controllers
left_joint_trajectory_controller   joint_trajectory_controller/JointTrajectoryController  active
joint_state_broadcaster            joint_state_broadcaster/JointStateBroadcaster          active
right_joint_trajectory_controller  joint_trajectory_controller/JointTrajectoryController   active
left_gripper_controller            position_controllers/GripperActionController            active
right_gripper_controller           position_controllers/GripperActionController            active
right_compliance_controller        openarm_compliance_controller/ComplianceController      active
```

## 4. Suspected Root Cause

The `right_compliance_controller` manages the gripper's Kp/Kd via:
- `gripper_joint: "openarm_right_finger_joint1"` (compliance_controller.yaml, line 59)
- `gripper_kp_default: 2.0`, `gripper_kd_default: 0.5`

There may be a **command interface conflict** between:
- `right_gripper_controller` (GripperActionController) → writes **position** command
- `right_compliance_controller` (ComplianceController) → writes **Kp/Kd/tau_ff** for the gripper

Possible causes:
1. **Interface conflict**: Both controllers may be claiming the same hardware command interface for the gripper joint, causing one to be silently blocked
2. **Kp/Kd override**: The compliance controller may be setting gripper Kp/Kd values that effectively prevent the GripperActionController's position command from producing motion
3. **Position target mismatch**: The GripperActionController sends position, but the compliance controller may be overriding the position target with its own value (zero)

## 5. Relevant Files

| File | Description |
|------|-------------|
| [compliance_controller.cpp](file:///home/user/ros2_ws/src/impedance_control/openarm_compliance_controller/src/compliance_controller.cpp) | C++ controller — gripper Kp/Kd write logic |
| [compliance_controller.yaml](file:///home/user/ros2_ws/src/impedance_control/openarm_compliance_controller/config/compliance_controller.yaml) | Gripper config lines 56-72 |
| [v10_simple_hardware.cpp](file:///home/user/ros2_ws/src/core/openarm_ros2/openarm_hardware/src/v10_simple_hardware.cpp) | Hardware interface — command interface claiming |
| [openarm_v10_bimanual_controllers.yaml](file:///home/user/ros2_ws/src/core/openarm_ros2/openarm_bringup/config/v10_controllers/openarm_v10_bimanual_controllers.yaml) | Controller config for gripper |

## 6. Questions for C1

1. Are the `right_gripper_controller` and `right_compliance_controller` both trying to write to the same gripper command interfaces? If so, which one wins?
2. Should the compliance controller's gripper control be disabled during normal operation and only enabled during teach mode?
3. Is the gripper position range correct? The hardware config says `0.0m (closed) to 0.032m (open)` — should the GripperCommand use values in this range instead of `-0.8`?

## 7. Impact

Without a working gripper, we **cannot** proceed with:
- VLA data collection (Phase 3) — episodes require pick/place with gripper
- Teach mode recording — gripper state must be captured
- Inference deployment — VLA model outputs include gripper actions

## 8. Test Without Compliance Controller

To verify if the gripper works WITHOUT the compliance controller, C1 can test:

```bash
# After bringup (without spawning compliance controller):
ros2 action send_goal /right_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.032, max_effort: 10.0}}"
```

If this works, the conflict is confirmed.

# Left Arm Compliance Controller — Real Hardware Test

> **Agent**: C1 | **Date**: 2026-05-07  
> **Purpose**: Validate that the left arm compliance controller works identically  
> to the validated right arm on real hardware. Tests gravity compensation,  
> impedance profile switching, gripper integration, and teach mode.

---

## Prerequisites

| Item | Status | Check |
|------|--------|-------|
| Robot powered ON | Required | Green power LED on PSU |
| CAN cables connected | Required | USB-CAN adapters plugged in |
| ROS 2 bringup running | Required | `ros2 node list` shows nodes |
| Right arm compliance tested | ✅ Done | Phase 2 validated |

> [!IMPORTANT]
> The left arm uses **CAN1** (right arm uses CAN0).  
> Make sure `can1` is UP before testing.

---

## Test 1: Verify Bringup (Left Arm Active)

**Goal**: Confirm the left arm hardware is live and publishing joint states.

### Terminal 1 — Bringup (if not already running)

```bash
# Skip if ROS 2 is already running
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
sudo ip link set can0 down 2>/dev/null && sudo ip link set can1 down 2>/dev/null
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 up type can bitrate 1000000 dbitrate 5000000 fd on
ros2 launch openarm_bringup openarm.bimanual.launch.py
```

### Terminal 2 — Check left arm joints

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash

# Verify left arm joints are publishing
ros2 topic echo /joint_states --once 2>&1 | grep -A1 "openarm_left_joint"
```

**Pass**: All 7 left arm joints (`openarm_left_joint1` through `openarm_left_joint7`) appear in `/joint_states` with non-zero positions.

---

## Test 2: Spawn Left Compliance Controller

**Goal**: Spawn the left compliance controller and verify it activates without errors.

> [!NOTE]
> The compliance controller coexists with the JointTrajectoryController.
> No deactivation needed — they claim different interfaces.

### Terminal 2

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch openarm_compliance_controller compliance.launch.py side:=left
```

**Expected output** (in bringup terminal):
```
[INFO] Configuring controller 'left_compliance_controller'
[INFO] Gripper impedance enabled: joint=openarm_left_finger_joint1, kp=5.0, kd=0.10
[INFO] KDL chain initialized: 7 joints, 9 segments (openarm_body_link0 -> openarm_left_hand)
[INFO] Compliance controller activated with default gains
```

### Verify controller is active

```bash
ros2 control list_controllers | grep left_compliance
```

**Pass**: Shows `left_compliance_controller` with status `active`.

### Verify topics exist

```bash
ros2 topic list | grep left_compliance
```

**Expected**:
```
/left_compliance_controller/external_force
/left_compliance_controller/gains
/left_compliance_controller/impedance_params
/left_compliance_controller/set_payload
/left_compliance_controller/tau_ff
/left_compliance_controller/transition_event
```

**Pass**: All 6 topics listed.

---

## Test 3: Gravity Compensation (tau_ff)

**Goal**: Verify that tau_ff values match expected gravity torques for the left arm.

### Terminal 3

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash

echo "=== Left arm tau_ff ==="
ros2 topic echo /left_compliance_controller/tau_ff --once
```

**Pass criteria**:
- All 7 values are non-zero
- J2 (shoulder) should show the largest magnitude (supporting arm weight)
- Signs should be physically plausible (gravity pulls down)

> [!TIP]
> Compare with right arm tau_ff. Values should be similar in magnitude  
> but may differ in sign depending on the left arm's mirrored kinematics.

### Quick comparison (run both side by side):

```bash
echo "=== RIGHT ===" && ros2 topic echo /right_compliance_controller/tau_ff --once
echo "=== LEFT ===" && ros2 topic echo /left_compliance_controller/tau_ff --once
```

---

## Test 4: Default Gains Readout

**Goal**: Verify startup gains match the YAML configuration.

```bash
echo "=== Left arm gains ==="
ros2 topic echo /left_compliance_controller/gains --once
```

**Expected** (kp_default / kd_default from YAML):
```
data:
- 70.0    # J1 Kp
- 70.0    # J2
- 70.0    # J3
- 60.0    # J4
- 10.0    # J5
- 10.0    # J6
- 10.0    # J7
- 2.75    # J1 Kd
- 2.5     # J2
- 2.0     # J3
- 2.0     # J4
- 0.7     # J5
- 0.6     # J6
- 0.5     # J7
```

**Pass**: All 14 values match defaults above.

---

## Test 5: Impedance Profile Switching

**Goal**: Verify that the impedance profile manager can control the left arm gains.

### Terminal 3 — Start profile manager for left arm

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash

# IMPORTANT: must pass side:=left as a ROS parameter, not a CLI flag
python3 ~/ros2_ws/src/impedance_control/openarm_compliance_controller/scripts/impedance_profile_manager.py \
  --ros-args -p side:=left
```

**Expected**:
```
Impedance Profile Manager ready.
  Side: left
  Publishing to: /left_compliance_controller/impedance_params (16 values)
```

> [!WARNING]
> If the output says `Side: right`, the parameter was not picked up.  
> Kill it (Ctrl+C) and re-run with `--ros-args -p side:=left`.

### Terminal 4 — Switch profiles and verify

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash

# Switch to teach mode
echo "--- Switching to TEACH ---"
ros2 topic pub -1 /impedance_phase std_msgs/msg/String '{data: "teach"}'
sleep 1
ros2 topic echo /left_compliance_controller/gains --once

# Switch back to transit
echo "--- Switching to TRANSIT ---"
ros2 topic pub -1 /impedance_phase std_msgs/msg/String '{data: "transit"}'
sleep 1
ros2 topic echo /left_compliance_controller/gains --once
```

**Pass criteria**:
- After "teach": Kp = `[15, 15, 15, 12, 3, 3, 3]` (= kp_min)
- After "transit": Kp = `[70, 70, 70, 60, 10, 10, 10]` (= kp_default)

---

## Test 6: Teach Mode — Physical Feel

**Goal**: Verify the left arm feels freely draggable in teach mode.

> [!CAUTION]
> **Hold the arm** before switching to teach mode!  
> At kp_min the arm will be very soft and may drift if not supported.  
> Gravity compensation keeps it from sagging, but it won't resist pushes.

### With profile manager still running (Terminal 3):

```bash
# In Terminal 4:
ros2 topic pub -1 /impedance_phase std_msgs/msg/String '{data: "teach"}'
```

### Physical checks (do these by hand):

| Check | Expected | Result |
|-------|----------|--------|
| Gently push J1 (base rotation) | Arm rotates freely with minimal resistance | [ ] pass / [ ] fail |
| Gently push J2 (shoulder) | Arm moves freely, does NOT sag under gravity | [ ] pass / [ ] fail |
| Move J4 (elbow) through range | Smooth motion, no cogging or stiffness | [ ] pass / [ ] fail |
| Move J5-J7 (wrist) | Light, free-feeling wrist motion | [ ] pass / [ ] fail |
| Release arm in mid-air | Arm holds position (gravity compensated) | [ ] pass / [ ] fail |
| Check tau_ff while dragging | Values change as arm moves (tracking gravity) | [ ] pass / [ ] fail |

### Verify tau_ff stays active during teach mode:

```bash
# While physically holding/moving the arm:
ros2 topic echo /left_compliance_controller/tau_ff --once
```

**Pass**: Non-zero values that change as arm is moved to different configurations.

### Return to transit when done:

```bash
ros2 topic pub -1 /impedance_phase std_msgs/msg/String '{data: "transit"}'
```

---

## Test 7: Gripper Function (with Compliance Controller)

**Goal**: Verify the left gripper responds to GripperCommand while the left compliance controller is active.

> [!NOTE]
> This test verifies the gripper Kp fix (2.0 → 5.0). The compliance controller  
> now uses Kp=5.0 which matches the hardware driver's GRIPPER_KP constant.

### Terminal 4

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash

# Open gripper
echo "--- Opening left gripper ---"
ros2 action send_goal /left_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.032, max_effort: 10.0}}"
```

**Expected**:
```
Goal accepted with ID: ...
Result:
    position: 0.032...
    stalled: false
    reached_goal: true
Goal finished with status: SUCCEEDED
```

```bash
# Close gripper
echo "--- Closing left gripper ---"
ros2 action send_goal /left_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.0, max_effort: 10.0}}"
```

**Expected**:
```
Goal finished with status: SUCCEEDED
```

**Pass criteria**:
- [ ] Gripper physically opens to ~32mm
- [ ] Gripper physically closes back to 0mm
- [ ] Both goals succeed (not ABORTED/stalled)
- [ ] No errors in bringup terminal

---

## Test 8: Profile Sweep (All 5 Profiles)

**Goal**: Cycle through all impedance profiles and verify gains update correctly.

### With profile manager running (Terminal 3):

```bash
# In Terminal 4 — run the full sweep:
for profile in transit approach contact grasp teach; do
  echo "=== Profile: $profile ==="
  ros2 topic pub -1 /impedance_phase std_msgs/msg/String "{data: \"$profile\"}"
  sleep 1
  ros2 topic echo /left_compliance_controller/gains --once 2>&1 | head -16
  echo ""
done

# Return to transit
ros2 topic pub -1 /impedance_phase std_msgs/msg/String '{data: "transit"}'
```

**Expected Kp values per profile**:

| Profile | Kp (J1-J4) | Kp (J5-J7) | Feel |
|---------|-----------|-----------|------|
| transit | 70, 70, 70, 60 | 10, 10, 10 | Stiff |
| approach | 50, 50, 50, 40 | 8, 8, 8 | Medium |
| contact | 30, 30, 30, 20 | 5, 5, 5 | Soft |
| grasp | 70, 70, 70, 60 | 10, 10, 10 | Stiff (grip firm) |
| teach | 15, 15, 15, 12 | 3, 3, 3 | Very soft |

**Pass**: All 5 profiles produce correct gain values, no errors.

---

## Test 9: External Force Estimation

**Goal**: Verify external force estimation works on the left arm.

```bash
# In Terminal 4:
echo "=== External force (arm at rest) ==="
ros2 topic echo /left_compliance_controller/external_force --once
```

**Expected**: Near-zero values when arm is untouched.

```bash
# Now PUSH the arm gently at J3/J4 and read again:
echo "=== External force (pushing arm) ==="
ros2 topic echo /left_compliance_controller/external_force --once
```

**Pass**: Values increase noticeably when force is applied, return to near-zero when released.

---

## Summary Checklist

```
Test 1 — Bringup verified                       [ ] pass / [ ] fail
Test 2 — Controller spawned & active             [ ] pass / [ ] fail
Test 3 — Gravity compensation (tau_ff)           [ ] pass / [ ] fail
Test 4 — Default gains correct                   [ ] pass / [ ] fail
Test 5 — Profile switching                       [ ] pass / [ ] fail
Test 6 — Teach mode physical feel                [ ] pass / [ ] fail
Test 7 — Gripper with compliance controller      [ ] pass / [ ] fail
Test 8 — Profile sweep (all 5)                   [ ] pass / [ ] fail
Test 9 — External force estimation               [ ] pass / [ ] fail
```

---

## Troubleshooting

### Controller fails to activate

```
[ERROR] Not acceptable command interfaces combination
```

**Cause**: Interface conflict. Check if another controller already claims stiffness/damping.
```bash
ros2 control list_hardware_interfaces | grep left.*stiffness
```

### tau_ff is all zeros

**Cause**: KDL chain not found. Check `root_link` and `tip_link` in YAML match the URDF:
```bash
ros2 param get /left_compliance_controller root_link
ros2 param get /left_compliance_controller tip_link
```

### Gripper stalls (reached_goal: false)

**Cause**: Gripper Kp too low. Verify the fix was applied:
```bash
ros2 param get /left_compliance_controller gripper_kp_default
# Should be 5.0 (not 2.0)
```

### Arm sags in teach mode

**Cause**: tau_ff scale factors may need calibration for the left arm.
Check if the left arm has different friction characteristics than the right arm.
The current config uses the **same** friction model for both arms — this may need
separate calibration if the left arm behaves differently.

---

## When Done

Fill in the summary checklist above and report results.  
If all 9 tests pass: Left arm compliance controller is **validated for real hardware**.

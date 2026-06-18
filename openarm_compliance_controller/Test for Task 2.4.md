# Test for Task 2.4 — Impedance Profile Manager (Real Hardware)

> **Date**: 2026-04-30 | **Agent**: C1 → User | **Environment**: Real Hardware
> **Prerequisite**: Phase 1 complete, compliance controller validated

## Goal

Verify that the impedance profile manager correctly switches the arm's
stiffness on real hardware. Each profile should produce a noticeably
different feel when you push the arm by hand.

---

## Step 0: CAN + Build (if needed)

```bash
sudo ip link set can0 down 2>/dev/null; sudo ip link set can1 down 2>/dev/null
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 up type can bitrate 1000000 dbitrate 5000000 fd on
cd ~/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --packages-select openarm_compliance_controller --symlink-install
source install/setup.bash
```

## Step 1: Launch real hardware (Terminal 1)

```bash
cd ~/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch openarm_bringup openarm.bimanual.launch.py use_fake_hardware:=false
```

## Step 2: Spawn compliance controller (Terminal 2)

```bash
cd ~/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run controller_manager spawner right_compliance_controller -c /controller_manager \
  --param-file $(ros2 pkg prefix openarm_compliance_controller)/share/openarm_compliance_controller/config/compliance_controller.yaml
```

## Step 3: Move arm to J4=90° and HOLD (Terminal 2)

```bash
ros2 action send_goal /right_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [openarm_right_joint1, openarm_right_joint2, openarm_right_joint3, openarm_right_joint4, openarm_right_joint5, openarm_right_joint6, openarm_right_joint7], points: [{positions: [0, 0, 0, 1.571, 0, 0, 0], time_from_start: {sec: 3}}, {positions: [0, 0, 0, 1.571, 0, 0, 0], time_from_start: {sec: 600}}]}}"
```

Leave running. Open **Terminal 3** for remaining steps.

## Step 4: Start Profile Manager (Terminal 3)

```bash
cd ~/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
python3 src/impedance_control/openarm_compliance_controller/scripts/impedance_profile_manager.py
```

Expected output:
```
Profile: transit
  Kp: [70, 70, 70, 60, 10, 10, 10]
  Kd: [2.75, 2.50, 2.00, 2.00, 0.70, 0.60, 0.50]
  Grip Kp: 2.0
```

The arm should now be in **transit** mode (stiff). Try pushing the forearm
gently — it should resist firmly.

Open **Terminal 4** for profile switching commands.

---

## Step 5: Profile Switching Tests (Terminal 4)

Run each command below, then push the arm gently by hand after each switch.
Record your observations.

### 5a: Switch to TEACH mode (very soft)

```bash
cd ~/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 topic pub --once /impedance_phase std_msgs/String '{data: "teach"}'
```

> ⚠️ **SAFETY**: The arm will become very compliant. Hold the arm lightly
> before switching. In teach mode, the arm should feel easy to push around —
> like guiding a person's arm.

**What to observe**:
- [ ] Arm feels noticeably softer than transit mode
- [ ] You can push J4 (elbow) with light finger pressure
- [ ] Arm slowly returns to the held position when released
- [ ] No oscillation or instability

**Your notes**: _______________________________________________

### 5b: Switch to CONTACT mode (medium-soft)

```bash
ros2 topic pub --once /impedance_phase std_msgs/String '{data: "contact"}'
```

**What to observe**:
- [ ] Arm is stiffer than teach but softer than transit
- [ ] Moderate resistance when pushing
- [ ] Good for compliant interaction with objects

**Your notes**: _______________________________________________

### 5c: Switch to APPROACH mode (medium)

```bash
ros2 topic pub --once /impedance_phase std_msgs/String '{data: "approach"}'
```

**What to observe**:
- [ ] Arm is stiffer than contact
- [ ] Noticeable resistance but not fully rigid
- [ ] Appropriate for approaching an object before contact

**Your notes**: _______________________________________________

### 5d: Switch to TRANSIT mode (stiff)

```bash
ros2 topic pub --once /impedance_phase std_msgs/String '{data: "transit"}'
```

**What to observe**:
- [ ] Arm feels rigid — strong resistance to pushing
- [ ] Holds position firmly
- [ ] This is the default operating mode for free-space motion

**Your notes**: _______________________________________________

### 5e: Switch to GRASP mode (stiff arm + high grip force)

```bash
ros2 topic pub --once /impedance_phase std_msgs/String '{data: "grasp"}'
```

**What to observe**:
- [ ] Arm feels same as transit (stiff)
- [ ] Grip Kp is published as 5.0 (check Terminal 3 output)
- [ ] If gripper stiffness controller is loaded, gripper should feel firmer

**Your notes**: _______________________________________________

### 5f: Test invalid profile (should be rejected)

```bash
ros2 topic pub --once /impedance_phase std_msgs/String '{data: "unknown_mode"}'
```

**Expected**: Terminal 3 shows warning:
```
[WARN] Unknown profile "unknown_mode". Available: transit, approach, contact, grasp, teach
```

- [ ] Warning logged, no crash, arm stays in previous profile

### 5g: Rapid switching stress test

```bash
for p in teach transit contact grasp approach transit; do \
  ros2 topic pub --once /impedance_phase std_msgs/String "{data: '$p'}"; \
  sleep 1; \
done
```

**What to observe**:
- [ ] All 6 transitions happen smoothly
- [ ] No jerks, spikes, or instability during transitions
- [ ] Terminal 3 shows all transitions logged correctly

**Your notes**: _______________________________________________

---

## Step 6: Force Monitor During Profile Switch (Optional — Terminal 5)

For a more quantitative test, run the force monitor alongside the profile
manager. Push the arm with similar force in each profile and compare the
tau_ext readings:

```bash
cd ~/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
python3 src/impedance_control/openarm_compliance_controller/scripts/force_monitor.py
```

Expected: Same push force should produce **higher tau_ext in teach mode**
(arm deflects more, so position error × Kp is lower, leaving more
uncompensated force) and **lower tau_ext in transit mode** (arm barely
moves, Kp absorbs most of the force).

---

## Step 7: Return home and stop

```bash
# Terminal 4:
ros2 topic pub --once /impedance_phase std_msgs/String '{data: "transit"}'
```

Wait 2 seconds (arm stiffens), then:

```bash
# Terminal 2:
ros2 action send_goal /right_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [openarm_right_joint1, openarm_right_joint2, openarm_right_joint3, openarm_right_joint4, openarm_right_joint5, openarm_right_joint6, openarm_right_joint7], points: [{positions: [0, 0, 0, 0, 0, 0, 0], time_from_start: {sec: 3}}]}}"
```

Stop the profile manager (Ctrl+C in Terminal 3).

---

## Results Summary

Fill in after testing:

```
=== Task 2.4 Real Hardware Validation ===
Date: ____

Transit feels stiff?     [ ] yes / [ ] no
Teach feels soft?        [ ] yes / [ ] no
Contact feels medium?    [ ] yes / [ ] no
Approach feels medium?   [ ] yes / [ ] no
Grasp = transit + grip?  [ ] yes / [ ] no
Invalid profile rejected? [ ] yes / [ ] no
Rapid switching stable?  [ ] yes / [ ] no
Any oscillation/instability? [ ] yes / [ ] no
Any [ERROR] in logs?     [ ] yes / [ ] no

Stiffness ranking (softest to stiffest):
  teach < contact < approach < transit = grasp

Does the ranking match your feel? [ ] yes / [ ] no

Additional observations: _______________________________________________
```

## Pass Criteria

- [x] All 5 profiles switch without error
- [ ] Stiffness ranking matches expected order (teach < contact < approach < transit)
- [ ] No oscillation or instability during or after switching
- [ ] Invalid profile rejected gracefully
- [ ] Rapid switching produces no jerks or spikes

**Task 2.4 Real HW Status: ⬜ PENDING**

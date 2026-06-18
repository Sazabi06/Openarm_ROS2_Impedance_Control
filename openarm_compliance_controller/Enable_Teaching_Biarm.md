# Enable Teaching Mode — Bimanual Setup Guide

**Purpose**: Step-by-step reference to enable teach mode on the OpenArm bimanual system.  
**Last validated**: 2026-05-07 on real hardware (both arms).

---

## Quick Reference

| What you want | Terminals needed | Jump to |
|---------------|-----------------|---------|
| Left arm only | 3 terminals | [Option A](#option-a-left-arm-only) |
| Right arm only | 3 terminals | [Option B](#option-b-right-arm-only) |
| Both arms (CLI) | 4 terminals | [Option C](#option-c-both-arms) |
| Both arms (GUI) ✨ | 3 terminals | [Option D](#option-d-gui-recommended) |

---

## Step 0: CAN FD Bus Initialization

> [!IMPORTANT]
> This must be done **every time** the robot is powered on or after a USB
> disconnect. Skip if CAN buses are already up.

```bash
# Deactivate CAN buses (reset any stale state)
sudo ip link set can0 down 2>/dev/null
sudo ip link set can1 down 2>/dev/null

# Reactivate with CAN FD at 1Mbps / 5Mbps data rate
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 up type can bitrate 1000000 dbitrate 5000000 fd on
```

**Verify**:
```bash
ip -details link show can0 | grep "fd on"
ip -details link show can1 | grep "fd on"
```
Both should show `fd on`. Right arm uses **can0**, left arm uses **can1**.

---

## Option A: Left Arm Only

### Terminal 1 — Bringup

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch openarm_bringup openarm.bimanual.launch.py
```

Wait for: `Compliance controller activated with default gains` in the log.

### Terminal 2 — Left Compliance Controller

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch openarm_compliance_controller compliance.launch.py side:=left
```

### Terminal 3 — Left Profile Manager + Teach Mode

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/impedance_control/openarm_compliance_controller/scripts/impedance_profile_manager.py \
  --ros-args -p side:=left
```

Then **in any other terminal** (or open a 4th):

```bash
# Enable teach mode
ros2 topic pub -1 /impedance_phase std_msgs/msg/String '{data: "teach"}'

# Verify (J4 kp should be 5.0, J1-J3 = 15.0, J5-J7 = 3.0)
ros2 topic echo /left_compliance_controller/gains --once

# Exit teach mode when done
ros2 topic pub -1 /impedance_phase std_msgs/msg/String '{data: "transit"}'
```

---

## Option B: Right Arm Only

### Terminal 1 — Bringup

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch openarm_bringup openarm.bimanual.launch.py
```

### Terminal 2 — Right Compliance Controller

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch openarm_compliance_controller compliance.launch.py side:=right
```

### Terminal 3 — Right Profile Manager + Teach Mode

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/impedance_control/openarm_compliance_controller/scripts/impedance_profile_manager.py \
  --ros-args -p side:=right
```

Then **in any other terminal**:

```bash
# Enable teach mode
ros2 topic pub -1 /impedance_phase std_msgs/msg/String '{data: "teach"}'

# Verify
ros2 topic echo /right_compliance_controller/gains --once

# Exit teach mode
ros2 topic pub -1 /impedance_phase std_msgs/msg/String '{data: "transit"}'
```

---

## Option C: Both Arms

### Terminal 1 — Bringup

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch openarm_bringup openarm.bimanual.launch.py
```

### Terminal 2 — Both Compliance Controllers

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash

# Spawn left
ros2 launch openarm_compliance_controller compliance.launch.py side:=left &
sleep 3

# Spawn right
ros2 launch openarm_compliance_controller compliance.launch.py side:=right
```

### Terminal 3 — Left Profile Manager

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/impedance_control/openarm_compliance_controller/scripts/impedance_profile_manager.py \
  --ros-args -p side:=left
```

### Terminal 4 — Right Profile Manager

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/impedance_control/openarm_compliance_controller/scripts/impedance_profile_manager.py \
  --ros-args -p side:=right
```

### Enable Teach Mode (Both Arms)

Both profile managers listen on the same `/impedance_phase` topic, so **one
command** switches both arms simultaneously:

```bash
# Enable teach mode for both arms
ros2 topic pub -1 /impedance_phase std_msgs/msg/String '{data: "teach"}'

# Verify both arms
echo "=== LEFT ===" && ros2 topic echo /left_compliance_controller/gains --once
echo "=== RIGHT ===" && ros2 topic echo /right_compliance_controller/gains --once

# Exit teach mode for both arms
ros2 topic pub -1 /impedance_phase std_msgs/msg/String '{data: "transit"}'
```

---

## Option D: GUI (Recommended)

The bimanual GUI provides a single-window interface with tabs for each arm,
a teach mode toggle button, presets, per-joint sliders, and gripper control.

### Terminal 1 — CAN + Bringup

```bash
sudo ip link set can0 down 2>/dev/null && sudo ip link set can1 down 2>/dev/null
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 up type can bitrate 1000000 dbitrate 5000000 fd on
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch openarm_bringup openarm.bimanual.launch.py
```

### Terminal 2 — Compliance Controllers

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch openarm_compliance_controller compliance.launch.py side:=left &
sleep 3
ros2 launch openarm_compliance_controller compliance.launch.py side:=right
```

### Terminal 3 — Bimanual GUI

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/impedance_control/openarm_compliance_controller/scripts/impedance_gui.py --side both
```

### Using the GUI

- Click **🎓 Teach Mode** in the header to toggle teach mode ON/OFF for both arms
- Switch between **⬅ Left Arm** and **➡ Right Arm** tabs to view/control each arm
- Use presets: Full Stiff, Soft Wrist, Full Soft, Extra Stiff
- Drag individual Kp/Kd sliders for fine-tuning
- Control gripper via Open/Close buttons or position spinbox
- **⬛ E-STOP** resets both arms to defaults + sends home trajectory

> [!NOTE]
> The GUI publishes impedance gains directly — **no profile managers needed**.
> Options A–C require separate profile manager terminals; the GUI replaces them.

---

## Shutdown Procedure

1. Exit teach mode (click 🎓 button or `ros2 topic pub -1 /impedance_phase std_msgs/msg/String '{data: "transit"}'`)
2. Close the GUI (Ctrl+C or close window)
3. Ctrl+C compliance controller spawners (Terminal 2)
4. Ctrl+C bringup (Terminal 1)

---

## Troubleshooting

### Joint drifts slowly in teach mode

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Joint sags downward | Gravity under-compensated | Increase `tau_ff_scale` for that joint |
| Joint drifts to specific angle | Friction offset `Fo` or `Fc` too large | Set `Fo`/`Fc` to 0.0 |
| Joint feels springy | `kp_min` too high | Lower `kp_min` (match hardware + YAML) |
| Joint bounces back to 0° | Teach mode not detected | Check kp ≤ kp_min + 0.1 in hardware |

### Config file location

```
~/ros2_ws/src/impedance_control/openarm_compliance_controller/config/compliance_controller.yaml
```

After editing, rebuild and re-spawn:
```bash
cd ~/ros2_ws && colcon build --packages-select openarm_compliance_controller --symlink-install
# Then Ctrl+C compliance controller and re-spawn (see above)
```

### CAN bus errors

```bash
# Check CAN bus status
ip -details link show can0
ip -details link show can1

# If "state BUS-OFF" or "state ERROR-ACTIVE", reset:
sudo ip link set can0 down && sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 down && sudo ip link set can1 up type can bitrate 1000000 dbitrate 5000000 fd on
```

---

## Safety Notes

> [!CAUTION]
> **Always hold the arm** before entering teach mode! At kp_min, the arm is
> very soft. Gravity compensation prevents sagging, but there is no position
> holding force — the arm moves freely when pushed.

> [!WARNING]
> **Do not run teach mode without the compliance controller.** The JTC alone
> does not provide gravity compensation. Without tau_ff, the arm will drop.

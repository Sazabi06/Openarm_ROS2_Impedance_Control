# Phase 3 Integration Test Script

> **Purpose**: Validate the end-to-end VLA data pipeline on real hardware  
> **Date**: 2026-05-08  
> **Prepared by**: Agent-O (Orchestrator)  
> **Expected duration**: ~45 minutes

---

## Why This Test?

Phase 3 has **two independently built halves** that have never been tested together:

| Built by | What | Status |
|----------|------|--------|
| **C1** | Bimanual teach mode (arms float freely, gravity-compensated) | ✅ HW validated |
| **C2** | Data recording scripts + LeRobot conversion + SmolVLA node | ⚠️ Scripts exist, never run on real HW |

**This integration test answers 5 critical questions:**

1. Can C2's `record_episode.py` capture joint states + camera images while C1's teach mode is active?
2. Does the recorded data convert cleanly into LeRobot v3 format?
3. Does SmolVLA load on the RTX 5080 without running out of VRAM?
4. Are the camera topics publishing at expected rates?
5. Is the full pipeline (teach → record → convert → model load) ready for Gate 3?

**If this test passes, Phase 3 is complete and we can request Gate 3 review.**

---

## Prerequisites

- [ ] Robot powered on, USB connected
- [ ] RTX 5080 available (not occupied by other GPU processes)
- [ ] An object to manipulate (e.g., a bottle on the table)

---

## Part A: CAN Calibration (One-Time, ~5 min)

> [!IMPORTANT]
> This step requires ROS 2 to NOT be running. LeRobot and ROS 2 both open
> the CAN socket — they cannot run simultaneously. This is a one-time
> calibration; you won't need to repeat it.

### A.1 — Stop any running ROS 2

```bash
# Kill any ROS 2 processes
pkill -f ros2 2>/dev/null
pkill -f openarm 2>/dev/null
sleep 2
```

### A.2 — Activate LeRobot environment

```bash
export PATH="$HOME/miniforge3/bin:$PATH"
eval "$(conda shell.bash hook)"
conda activate lerobot
```

### A.3 — Setup CAN for LeRobot

```bash
sudo ip link set can0 down 2>/dev/null
sudo ip link set can1 down 2>/dev/null
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 up type can bitrate 1000000 dbitrate 5000000 fd on
```

### A.4 — Calibrate right arm

```bash
lerobot-calibrate \
  --robot.type=openarm_follower \
  --robot.port=can0 \
  --robot.side=right \
  --robot.id=openarm_right
```

**Expected**: Calibration completes with joint ranges detected. A `.cache/calibration/` file is saved.

### A.5 — Record result

```
✅ PASS / ❌ FAIL: CAN calibration
Notes: _______________________________________________
```

### A.6 — Deactivate LeRobot CAN (give it back to ROS 2)

```bash
conda deactivate
sudo ip link set can0 down
sudo ip link set can1 down
```

---

## Part B: Teach Mode + Recording Test (~20 min)

> This is the core integration test. We use 4 terminals.

### B.1 — Terminal 1: CAN + ROS 2 Bringup

```bash
sudo ip link set can0 down 2>/dev/null && sudo ip link set can1 down 2>/dev/null
sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 up type can bitrate 1000000 dbitrate 5000000 fd on
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch openarm_bringup openarm.bimanual.launch.py
```

**Wait for**: `Compliance controller activated with default gains` in the log.

### B.2 — Terminal 2: Compliance Controllers

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch openarm_compliance_controller compliance.launch.py side:=right
```

**Wait for**: Controller active message.

### B.3 — Terminal 3: Camera Bringup (check what cameras are available)

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash

# Check camera topics
ros2 topic list | grep -i "image\|camera"
```

**Record which topics exist:**
```
✅ /camera/color/image_raw (D435i head)         YES / NO
✅ /right_wrist_camera/color/image_raw (D405)    YES / NO
```

If cameras are not running, launch them:
```bash
ros2 launch openarm_vision camera_bringup.launch.py
```

### B.4 — Terminal 4: Teach Mode + Recording Integration

**Step 1: Verify teach mode works**

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash

# Start impedance profile manager (needed for teach mode topic)
python3 ~/ros2_ws/src/impedance_control/openarm_compliance_controller/scripts/impedance_profile_manager.py \
  --ros-args -p side:=right &
sleep 2

# Enable teach mode
ros2 topic pub -1 /impedance_phase std_msgs/msg/String '{data: "teach"}'
```

**Check**: Can you physically drag the right arm freely? Does gravity compensation hold it in place when you let go?

```
✅ PASS / ❌ FAIL: Arm floats freely in teach mode
✅ PASS / ❌ FAIL: Gravity compensation holds arm (no sagging)
✅ PASS / ❌ FAIL: Gripper is free (can be opened/closed by hand)
```

**Step 2: Record a test episode**

```bash
python3 ~/ros2_ws/src/impedance_control/openarm_compliance_controller/scripts/record_episode.py \
  --task "Pick up the bottle" \
  --side right
```

**Expected**: The recorder UI appears with keyboard controls.

Now perform a recording:
1. Place a bottle in the workspace
2. Press **`s`** to start recording
3. Guide the arm by hand: reach toward the bottle → grasp → lift → place back
4. Press **`e`** to end episode
5. Repeat steps 2-4 for **3 episodes total**
6. Press **`q`** to quit

**Check output:**
```bash
ls -la ~/lerobot_data/
ls -la ~/lerobot_data/episode_0000/
cat ~/lerobot_data/episode_0000/metadata.json
wc -l ~/lerobot_data/episode_0000/joints.csv
ls ~/lerobot_data/episode_0000/head_cam/ | wc -l
ls ~/lerobot_data/episode_0000/wrist_cam/ | wc -l 2>/dev/null
```

**Record results:**
```
✅ PASS / ❌ FAIL: Episodes saved to ~/lerobot_data/
  - Number of episodes recorded: ___
  - Joint CSV rows per episode: ___ (expect ~30 × duration_seconds)
  - Head camera frames per episode: ___ (should match CSV rows)
  - Wrist camera frames per episode: ___ (0 if no wrist cam)
  - metadata.json exists and looks correct: YES / NO
```

**Step 3: Restore arm to stiff mode**

```bash
ros2 topic pub -1 /impedance_phase std_msgs/msg/String '{data: "transit"}'
```

---

## Part C: LeRobot Conversion Test (~5 min)

### C.1 — Convert recorded episodes

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash

python3 ~/ros2_ws/src/impedance_control/openarm_compliance_controller/scripts/convert_to_lerobot.py \
  --input ~/lerobot_data \
  --output ~/lerobot_dataset \
  --repo-id local/openarm_bottle_pickup
```

**Expected**: Script processes all episodes and creates LeRobot-compatible output.

### C.2 — Verify output structure

```bash
echo "=== Dataset structure ==="
find ~/lerobot_dataset -type f | head -20

echo ""
echo "=== info.json ==="
cat ~/lerobot_dataset/meta/info.json

echo ""
echo "=== episodes.jsonl ==="
cat ~/lerobot_dataset/meta/episodes.jsonl

echo ""
echo "=== tasks.jsonl ==="
cat ~/lerobot_dataset/meta/tasks.jsonl

echo ""
echo "=== Parquet file size ==="
ls -lh ~/lerobot_dataset/data/
```

**Record results:**
```
✅ PASS / ❌ FAIL: Conversion completed without errors
✅ PASS / ❌ FAIL: data/train-00000-of-00001.parquet exists
✅ PASS / ❌ FAIL: meta/info.json has correct robot_type (openarm_follower)
✅ PASS / ❌ FAIL: meta/episodes.jsonl has correct episode count
✅ PASS / ❌ FAIL: meta/tasks.jsonl has the task description
```

---

## Part D: SmolVLA Model Load Test (~10 min)

> [!WARNING]
> This test requires the LeRobot conda environment. You can run it while
> ROS 2 is still running in other terminals — they don't conflict for GPU use.

### D.1 — Activate LeRobot and test model loading

```bash
export PATH="$HOME/miniforge3/bin:$PATH"
eval "$(conda shell.bash hook)"
conda activate lerobot

python3 -c "
import torch
import time

print('=== SmolVLA Model Load Test ===')
print(f'PyTorch: {torch.__version__}')
print(f'CUDA available: {torch.cuda.is_available()}')
if torch.cuda.is_available():
    print(f'GPU: {torch.cuda.get_device_name(0)}')
    total = torch.cuda.get_device_properties(0).total_mem / 1e9
    free = (torch.cuda.get_device_properties(0).total_mem - torch.cuda.memory_allocated()) / 1e9
    print(f'VRAM Total: {total:.1f} GB')
    print(f'VRAM Free: {free:.1f} GB')

print()
print('Loading SmolVLA policy...')
t0 = time.time()

from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
from lerobot.policies.smolvla.configuration_smolvla import SmolVLAConfig

config = SmolVLAConfig()
policy = SmolVLAPolicy(config)
print(f'Policy loaded in {time.time()-t0:.1f}s')

# Move to GPU
print('Moving to GPU (bfloat16)...')
t1 = time.time()
policy = policy.to(dtype=torch.bfloat16, device='cuda')
print(f'GPU transfer in {time.time()-t1:.1f}s')

used = torch.cuda.memory_allocated() / 1e9
total = torch.cuda.get_device_properties(0).total_mem / 1e9
print(f'VRAM used: {used:.1f} GB / {total:.1f} GB')
print(f'VRAM remaining: {total - used:.1f} GB')

if used < total * 0.8:
    print('✅ VRAM OK — sufficient headroom')
else:
    print('⚠️  VRAM tight — may need optimization')

print()
print('=== Test Complete ===')
"
```

**Record results:**
```
✅ PASS / ❌ FAIL: SmolVLA loads without OOM
  - VRAM used: ___ GB
  - VRAM remaining: ___ GB
  - Load time: ___ seconds
```

### D.2 — Quick inference speed test (optional)

```bash
python3 -c "
import torch
import time
import numpy as np

from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
from lerobot.policies.smolvla.configuration_smolvla import SmolVLAConfig

config = SmolVLAConfig()
policy = SmolVLAPolicy(config).to(dtype=torch.bfloat16, device='cuda')
policy.eval()

# Create dummy inputs matching expected format
batch = {
    'observation.images.head_cam': torch.randn(1, 3, 224, 224, dtype=torch.bfloat16, device='cuda'),
    'observation.state': torch.randn(1, 8, dtype=torch.bfloat16, device='cuda'),
}

# Warmup
with torch.no_grad():
    try:
        _ = policy.select_action(batch)
        print('Warmup done')
    except Exception as e:
        print(f'Inference test error (may need real model weights): {e}')
        print('This is expected if base weights are not downloaded yet.')
        print('To download: huggingface-cli download lerobot/smolvla_base')
        exit(0)

# Benchmark
times = []
with torch.no_grad():
    for i in range(10):
        t0 = time.time()
        _ = policy.select_action(batch)
        torch.cuda.synchronize()
        times.append(time.time() - t0)

avg = np.mean(times)
hz = 1.0 / avg
print(f'Average inference: {avg*1000:.0f}ms = {hz:.1f} Hz')
if hz > 5:
    print('✅ Inference speed OK (>5 Hz)')
else:
    print('⚠️  Inference below 5 Hz target')
"
```

**Record results:**
```
✅ PASS / ❌ FAIL: Inference runs without errors
  - Inference speed: ___ Hz (target: >5 Hz)
```

---

## Part E: Replay Test (Optional, ~5 min)

If Parts B-D pass, test replay to verify data quality:

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash

# Make sure arm is in stiff mode first!
ros2 topic pub -1 /impedance_phase std_msgs/msg/String '{data: "transit"}'

# Replay first episode
python3 ~/ros2_ws/src/impedance_control/openarm_compliance_controller/scripts/replay_episode.py \
  --episode ~/lerobot_data/episode_0000
```

**Check**: Does the arm reproduce the recorded motion smoothly?

```
✅ PASS / ❌ FAIL: Replay motion looks correct
```

---

## Results Summary

| Test | Part | Result | Notes |
|------|------|--------|-------|
| CAN calibration | A | __ | |
| Teach mode works | B.4-step1 | __ | |
| Episode recording | B.4-step2 | __ | |
| LeRobot conversion | C | __ | |
| SmolVLA model load | D.1 | __ | |
| Inference speed | D.2 | __ | |
| Replay quality | E | __ | |

### Decision Matrix

| Result | Next Step |
|--------|-----------|
| **All PASS** | Mark Tasks 3.1-3.3 as PASS → request Gate 3 review from Agent-R |
| **Recording fails** | Bring back Agent-C2 to debug `record_episode.py` |
| **Conversion fails** | Bring back Agent-C2 to fix `convert_to_lerobot.py` |
| **SmolVLA OOM** | Try `torch.float16` instead of `bfloat16`, or reduce batch size |
| **CAN cal fails** | Check motor power, CAN wiring, try `lerobot-setup-can --mode=setup` first |

---

## After the Test

Report results back to Agent-O with your filled-in Results Summary table.
If everything passes, I will:
1. Mark C2 Tasks 3.1, 3.2, 3.3 as PASS
2. Prepare the Agent-R prompt for Phase 3 Gate review
3. Outline Phase 4 kickoff plan

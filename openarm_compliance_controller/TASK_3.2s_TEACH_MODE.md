# Phase 3 Task: Teach Mode Infrastructure (Task 3.2s)

> **Assigned to**: Agent-C1 | **Priority**: 🟡 HIGH
> **Blocks**: C2's Task 3.2 (data collection requires teach mode)
> **Estimated effort**: ~1 day

---

## Objective

Ensure the compliance controller properly supports **teach mode** — where
the arm can be freely dragged by a human while gravity compensation keeps
it "weightless". This is the foundation for C2's drag-to-teach data collection.

---

## What Teach Mode Means

```
Normal mode:  Kp = tuned values (e.g., 70,70,70,60,10,10,10)
              Kd = tuned values
              tau_ff = gravity + friction model → arm holds position

Teach mode:   Kp = kp_min (safety floor, e.g., 0.3 per joint)
              Kd = kd_min (safety floor, e.g., 0.05 per joint)
              tau_ff = gravity + friction model → arm floats weightlessly
              Result: human can drag the arm freely, it won't sag
```

## Steps

### 1. Add teach mode preset to YAML

**File**: `config/compliance_controller.yaml`

Add a `teach_mode` section with Kp/Kd at minimum values for both arms:
```yaml
right_compliance_controller:
  ros__parameters:
    teach_mode:
      kp: [0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3]
      kd: [0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
```

### 2. Verify teach mode behavior in simulation

```bash
# Launch sim
ros2 launch openarm_bringup openarm.bimanual.launch.py use_fake_hardware:=true

# Spawn compliance controller
ros2 run controller_manager spawner right_compliance_controller ...

# Set Kp to minimum via existing impedance topic
ros2 topic pub --once /right_compliance_controller/set_impedance \
  std_msgs/msg/Float64MultiArray \
  '{data: [0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05]}'
```

Verify:
- tau_ff continues publishing (gravity compensation stays active)
- Gains topic shows Kp at minimum values
- No errors or instability

### 3. Test with impedance_profile_manager.py

Verify that the "teach" profile from Task 2.4 works:
```bash
ros2 topic pub --once /impedance_phase std_msgs/msg/String '{data: "teach"}'
```

The profile manager should set:
```python
"teach": {"kp": [15,15,15,12,3,3,3], "kd": [0.5,0.5,0.4,0.4,0.15,0.12,0.1]}
```

> **Note**: The "teach" profile in impedance_profile_manager.py has higher
> values than kp_min. For data collection, C2 may need even lower values.
> Make sure the controller accepts values down to kp_min without error.

### 4. Document in TEST.md

Add a teach mode section confirming:
- Minimum Kp/Kd values that work safely
- Whether tau_ff stays active at minimum impedance
- Any drift or instability observed

---

## Acceptance Criteria

- [ ] Teach mode preset exists in YAML config
- [ ] Setting Kp=kp_min via impedance topic works without error
- [ ] tau_ff (gravity compensation) remains active during teach mode
- [ ] "teach" impedance profile works via `/impedance_phase` topic
- [ ] Results documented in TEST.md

---

## When Done

1. Update `AGENT_C1_CONTROLS.md` — Task 3.2s status
2. Update `AGENT_O_ORCHESTRATOR.md` — progress table
3. Notify Agent-O so C2 can start data collection (Task 3.2)

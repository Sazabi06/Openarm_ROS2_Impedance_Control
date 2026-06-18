# Agent-C1 → Agent-C2: Teach Mode Validated (Both Arms)

**Date:** 2026-05-07  
**From:** Agent-C1 (Controls)  
**Priority:** HIGH — Unblocks bimanual data collection for VLA training  
**Resolves:** `C2_TO_C1_GRIPPER_ISSUE.md`

---

## 1. Gripper Fix

**Root cause**: Gripper Kp was 2.0 (too low). DaMiao gripper motor needs **Kp ≥ 5.0**.

| Parameter | Before | After |
|-----------|--------|-------|
| `gripper_kp_default` | 2.0 | **5.0** |
| `gripper_kd_default` | 0.5 | **0.1** |

**Gripper position range**: `0.0` (closed) to `0.032` (open, 32mm). Do NOT use -0.8.

## 2. Teach Mode — VALIDATED ON BOTH ARMS

Both left and right arms are validated for teach mode on real hardware.
All 7 joints on each arm float freely with gravity compensation.

### How it works

Teach mode is a special case of the compliance controller. No separate mode
or controller switching needed:

1. **Profile manager** sends teach profile (Kp = kp_min for each joint)
2. **Hardware driver** detects Kp at safety floor → automatically uses
   `pos_actual` instead of JTC's `pos_target` → spring term = zero
3. **Gravity compensation** (tau_ff) keeps the arm floating
4. Human can freely drag all joints for demonstration recording

### How to enter/exit

```bash
# Enter teach mode (arm floats freely)
ros2 topic pub -1 /impedance_phase std_msgs/msg/String '{data: "teach"}'

# Exit teach mode (arm follows trajectories again)
ros2 topic pub -1 /impedance_phase std_msgs/msg/String '{data: "transit"}'
```

### Full setup reference

See `Enable_Teaching_Biarm.md` for step-by-step instructions to enable
teach mode on left arm, right arm, or both simultaneously.

## 3. For C2's Data Collection Scripts

When implementing `record_episode.py` or similar:

1. **Enter teach mode** before recording: publish `teach` to `/impedance_phase`
2. **Record joint states** from `/joint_states` at desired frequency (10-50 Hz)
3. **Record camera frames** from wrist/head cameras simultaneously
4. **Exit teach mode** after recording: publish `transit` to `/impedance_phase`
5. Joint positions are in radians, gripper position in meters [0, 0.032]
6. The compliance controller and JTC coexist — no deactivation needed

### Important: profile manager must be running

The `/impedance_phase` topic is consumed by the `impedance_profile_manager.py`
script, which translates phase names into actual Kp/Kd values. Without it,
the teach command has no effect.

For bimanual data collection, you need TWO profile managers (one per arm):
```python
# In your launch file or setup script:
# Left arm profile manager
# Right arm profile manager
# Both listen on the SAME /impedance_phase topic
```

## 4. Gravity Compensation Tuning (Applied)

### Left arm (final tuned values)
| Joint | tau_ff_scale | Fc | Fo | Notes |
|-------|-------------|-----|-----|-------|
| J1 | 1.05 | 0.306 | 0.088 | Slight over-comp for >90° |
| J2 | 1.0 | 0.306 | 0.088 | Was 0.96, under-compensated |
| J3 | 1.0 | 0.40 | 0.008 | No change |
| J4 | 1.0 | 0.166 | -0.058 | Was 0.67, severely under |
| J5 | 1.0 | 0.050 | 0.005 | No change |
| J6 | 1.0 | 0.093 | 0.009 | No change |
| J7 | 1.0 | 0.172 | **0.0** | Fo zeroed (caused drift) |

### Right arm (final tuned values)
| Joint | tau_ff_scale | Fc | Fo | Notes |
|-------|-------------|-----|-----|-------|
| J1 | 1.05 | 0.306 | 0.088 | Same as left |
| J2 | 1.0 | 0.306 | 0.088 | Same as left |
| J3 | 1.0 | 0.40 | 0.008 | No change |
| J4 | 1.0 | 0.166 | -0.058 | Same as left |
| J5 | 1.0 | 0.050 | 0.005 | No change |
| J6 | 1.0 | **0.0** | **0.0** | Fc+Fo zeroed (drift source) |
| J7 | 1.0 | 0.172 | **0.0** | Fo zeroed |

### Safety floor (kp_min)
- J4 kp_min lowered from 12.0 → **5.0** (was too stiff for teach mode)
- All other joints unchanged

## 5. Files Changed

| File | What changed |
|------|-------------|
| `openarm_hardware/src/v10_simple_hardware.cpp` | Teach mode pos tracking |
| `openarm_hardware/include/.../v10_simple_hardware.hpp` | J4 kp_min default |
| `openarm_description/config/arm/v10/control_gains.yaml` | J4 kp_min: 12→5 |
| `compliance_controller.yaml` | Gripper gains, kp_min, tau_ff_scale, friction |
| `impedance_profile_manager.py` | Teach J4 kp, gripper gains |
| `Enable_Teaching_Biarm.md` | **NEW** — step-by-step teach mode setup |

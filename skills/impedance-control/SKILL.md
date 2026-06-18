---
name: impedance-control
description: Impedance control theory, implementation details, tuning guidelines, and teach mode operation for the OpenArm compliance controller. Covers the core control equation, gravity compensation, friction modeling, impedance profiles, and force estimation.
version: 1.0.0
---
# OpenArm Impedance Control

This skill contains the complete knowledge for understanding, tuning, and extending the OpenArm compliance controller. Read this before making any changes to control gains, impedance profiles, or the teach mode system.

## 1. Core Control Equation

Every 10ms (100 Hz), the compliance controller computes:

```
τ_cmd = τ_ff + Kp·(q_des - q) + Kd·(dq_des - dq)
```

| Term | Name | What It Does |
|------|------|-------------|
| `τ_ff` | Feedforward torque | Cancels gravity + friction (robot "floats") |
| `Kp·(q_des - q)` | Position spring | Pulls joint toward target (higher Kp = stiffer) |
| `Kd·(dq_des - dq)` | Velocity damper | Prevents overshooting (higher Kd = more damping) |

**Analogy:** Kp is a spring, Kd is a shock absorber. τ_ff cancels the arm's weight so the spring only fights disturbances, not gravity.

## 2. Gravity Compensation (τ_ff)

```
τ_ff = tau_ff_scale[j] × (τ_gravity[j] + τ_friction[j])
```

### Gravity torque
Computed using **KDL recursive Newton-Euler** from the URDF's link masses and inertias. Updated every cycle based on current joint positions.

### Friction model
```
τ_friction = Fc·tanh(0.1·k·dq) + Fv·dq + Fo
```

| Parameter | Meaning | Source |
|-----------|---------|--------|
| `Fc` | Coulomb friction magnitude | Calibrated from constant-velocity sweeps |
| `k` | Coulomb sharpness (tanh slope) | Higher = sharper transition at zero velocity |
| `Fv` | Viscous friction coefficient | Proportional to velocity |
| `Fo` | Static offset | Asymmetric friction (gravity bias at rest) |

### tau_ff_scale
Per-joint multiplier (0.85–1.05) to fine-tune gravity compensation:
- **Too low:** Arm drifts down in teach mode (gravity not fully canceled)
- **Too high:** Arm drifts up in teach mode (over-compensating)
- **Calibration method:** Put arm in teach mode. If it drifts, adjust tau_ff_scale by ±0.05.

### Current calibrated values (real hardware validated):
```yaml
tau_ff_scale: [1.05, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
friction:
  Fc: [0.306, 0.306, 0.40, 0.166, 0.050, 0.0, 0.172]
  k:  [28.417, 28.417, 29.065, 130.038, 151.771, 242.287, 7.888]
  Fv: [0.063, 0.063, 0.604, 0.813, 0.029, 0.072, 0.084]
  Fo: [0.088, 0.088, 0.008, -0.058, 0.005, 0.0, 0.0]
```

## 3. Impedance Profiles

Five pre-defined profiles for different operational phases.

**Source of truth**: `impedance_profile_manager.py` (the runtime profile switcher).

| Profile | Kp (J1-J3) | Kp (J4) | Kp (J5-J7) | Purpose |
|---------|-----------|---------|-----------|---------|
| **transit** | 70 | 60 | 10 | Moving to a target (stiff, precise) |
| **approach** | 50 | 40 | 8 | Approaching an object (medium compliance) |
| **contact** | 30 | 20 | 5 | Touching/grasping (soft, force-limited) |
| **grasp** | 70 | 60 | 10 | Holding an object (stiff arm + firm grip) |
| **teach** | 12 | 3 | 2 | Human dragging (at kp_min, floats) |

**Note**: `compliance_controller.yaml` has a separate `teach_mode` preset with slightly higher values `[15, 15, 15, 12, 3, 3, 3]`. The profile manager's `teach` profile uses `kp_min` values `[12, 12, 12, 3, 2, 2, 2]` which are the actual minimum the controller will enforce.

### Profile switching
```bash
# Switch via ROS 2 topic
ros2 topic pub -1 /impedance_phase std_msgs/msg/String '{data: "teach"}'
ros2 topic pub -1 /impedance_phase std_msgs/msg/String '{data: "transit"}'
```

### Rate limiting
Kp/Kd changes are rate-limited to prevent sudden jerks:
- `delta_kp_max = 2.0` per cycle → full transit→teach sweep takes ~0.35s
- `delta_kd_max = 0.1` per cycle

## 4. Teach Mode

**Purpose:** Make the arm "weightless" so a human can drag it to record demonstration trajectories for VLA training.

**How it works:**
1. Kp is set to kp_min (near-zero stiffness)
2. τ_ff still active (gravity compensation keeps arm floating)
3. q_des tracks current position (no restoring force)
4. Gripper also becomes compliant (grip_kp = 0.3)

**Activation:**
```bash
ros2 topic pub -1 /impedance_phase std_msgs/msg/String '{data: "teach"}'
```

**Safety:** Never leave teach mode unattended. The arm can drift slowly due to imperfect gravity compensation.

## 5. Proprioceptive Force Estimation

Without an F/T sensor, we estimate external forces from motor effort readings. This was implemented as a dedicated sensing layer inside `compliance_controller.cpp`.

### 5.1 Algorithm

At each control cycle (100 Hz):
```
τ_ext_raw[i] = τ_motor[i] - τ_ff[i]
τ_ext[i] = alpha × τ_ext_raw[i] + (1 - alpha) × τ_ext_prev[i]
```

| Term | Source | Meaning |
|------|--------|---------|
| `τ_motor` | Motor effort state interface (DaMiao MIT mode current feedback) | Actual torque at each joint |
| `τ_ff` | KDL gravity + Coriolis + friction model | Expected torque with no external load |
| `alpha` | `ext_force_alpha` param (default 0.05) | LPF coefficient (~0.8 Hz cutoff at 100 Hz) |

**State interface change:** The controller reads 3 interfaces per joint: `position(0)`, `velocity(1)`, `effort(2)`. The effort interface was added specifically for this feature.

### 5.2 Published Topic

```
~/external_force  (std_msgs/msg/Float64MultiArray)
  data: [τ_ext_J1, τ_ext_J2, ..., τ_ext_J7]
  rate: 100 Hz (every control cycle)
```

### 5.3 Configuration
```yaml
# In compliance_controller.yaml
ext_force_alpha: 0.05   # Higher = faster response, more noise
                         # Lower = smoother, slower response
                         # 0.05 at 100Hz → ~0.8 Hz cutoff
```

### 5.4 Real Hardware Validation

Test: arm at J4=90°, user pushed arm 3 times at different locations:

| Reading | J1 | J2 | J3 | J4 | J5 | J6 | J7 |
|---------|------|------|------|------|------|------|------|
| Push shoulder | **-4.56** | 0.33 | -0.06 | 0.07 | 0.05 | 0.09 | 0.03 |
| Push elbow | 4.03 | -0.29 | 0.35 | **6.45** | **1.13** | 0.09 | 0.03 |
| Push wrist | 3.81 | -0.48 | -0.06 | **4.37** | 0.16 | 0.09 | **1.42** |
| After release | -0.82 | -0.45 | -0.06 | 0.50 | 0.16 | 0.08 | -0.15 |

**Result:** Forces correctly localize to pushed joints. Coupled joints show expected secondary response. All values return to <1.0 Nm within 2 seconds after release.

### 5.5 Force Monitor Tool

`force_monitor.py` provides human-readable force detection with calibrated per-joint thresholds:

```bash
python3 ~/ros2_ws/src/impedance_control/openarm_compliance_controller/scripts/force_monitor.py --side right
```

Per-joint thresholds (Nm), calibrated from real hardware baseline at rest:
```python
FORCE_THRESHOLDS = [1.5, 1.0, 0.5, 1.0, 0.5, 0.3, 0.5]
```

Output: `🔴 You are pushing J4 (elbow), with estimated torque of 6.45 Nm`

### 5.6 Closing the Loop — Two Approaches

The `~/external_force` topic is a **sensing layer only**. Two approaches exist for using this data:

| Approach | Method | Pros | Cons | Best For |
|----------|--------|------|------|----------|
| **A: Estimate weight** | `τ_ext` → mass = τ/(g×lever) → `set_payload` → rebuild KDL | Physically correct model | Needs estimation logic | Grabbing known objects |
| **B: Disturbance observer** | `τ_ff[i] += gain × τ_ext[i]` directly | Instant, adapts to ANY disturbance | Can amplify noise | Contact, human collab |

Both can coexist. The `set_payload` mechanism (Approach A) is already implemented — it accepts a mass value, rebuilds the KDL solver, and updates `τ_ff` automatically.

### 5.7 Limitations

- **Accuracy:** ~±0.5 Nm (friction-limited), not suitable for precision force control
- **No acceleration term:** `M(q)·q̈` is not accounted for — transient errors during fast motion
- **Sensor noise:** Motor current sensing is noisy; LPF adds ~200ms latency to 95% of step response
- **Fake hardware:** Simulation returns effort=0, so `τ_ext = -τ_ff` (not useful for testing)

## 6. Tuning Guidelines

### When robot oscillates
- **Cause:** Kp too high relative to Kd
- **Fix:** Increase Kd or decrease Kp
- **Rule of thumb:** Kd ≈ 2·√(Kp) / 30

### When robot is too sluggish
- **Cause:** Kp too low
- **Fix:** Increase Kp in steps of 5–10, verify no oscillation

### When robot drifts in teach mode
- **Cause:** tau_ff_scale wrong for that joint
- **Fix:** Adjust tau_ff_scale by ±0.05 per joint

### When gripper won't hold objects
- **Cause:** gripper_kp too low
- **Fix:** Increase gripper_kp (max 10.0)

## 7. Key Source Files

| File | What It Is |
|------|-----------|
| `compliance_controller.cpp` | Core 100Hz impedance control loop |
| `compliance_controller.hpp` | Data structures, RealtimeBuffer declarations |
| `compliance_controller.yaml` | All tuned parameters (Kp/Kd, friction, tau_ff, ext_force_alpha) |
| `impedance_profile_manager.py` | Profile definitions and switching logic |
| `impedance_gui.py` | PyQt5 GUI for runtime Kp/Kd adjustment |
| `teach_mode.py` | CLI teach mode toggle |
| `force_monitor.py` | Human-readable force detection with calibrated thresholds |
| `PROPRIOCEPTIVE_FORCE.md` | Force estimation methodology, hardware test results, roadmap |

## 8. Relation to CompliantVLA Paper

Our compliance controller implements the **VIC (Variable Impedance Controller) layer** from the CompliantVLA-adaptor paper (arXiv:2601.15541). The paper's three-tier architecture maps to our system:

```
Paper VLM (~1Hz)  →  Our impedance_profile_manager.py (currently rule-based)
Paper VLA (~3Hz)  →  Our vla_server.py (SmolVLA / pi-0.5 / GR00T)
Paper VIC (1000Hz) → Our compliance_controller.cpp (100Hz, sufficient for QDD motors)
```

**To reach paper level:** Replace rule-based profile switching with VLM-guided impedance parameter generation (GPT-4V / Gemini API call per contact phase transition). The framework is already in place — the `impedance_profile_manager.py` already accepts arbitrary Kp/Kd values via `/impedance_phase` topic.

See `compliant-vla-integration` skill for the full gap analysis and roadmap.

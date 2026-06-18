# Proprioceptive Force Estimation — Technical Documentation

> **Module**: `openarm_compliance_controller` | **Task**: 1.2b | **Date**: 2026-04-29

---

## 1. Motivation

When a robot arm interacts with the real world — picking up objects, touching surfaces, or being pushed by a human — it needs to **sense external forces** without dedicated force/torque sensors. This is critical for:

- **Unknown payload detection**: The arm grabs an object of unknown weight. The system must detect how heavy it is and compensate automatically (feeds into Task 1.2a payload compensation).
- **Contact detection**: Knowing when the end-effector touches a surface enables safe interaction, insertion tasks, and collision avoidance.
- **Compliance control**: Force feedback enables the controller to modulate stiffness in response to external loads, enabling safe human-robot interaction.

The OpenArm V10 does **not** have a wrist-mounted force/torque sensor. However, each joint motor reports its actual torque via current sensing. By comparing this **measured** torque against the **expected** torque from our dynamics model, we can estimate external forces — a technique called **proprioceptive force estimation**.

---

## 2. Goal

Implement a real-time external torque estimator inside the compliance controller that:

1. Reads actual motor torque from the hardware (effort state interface)
2. Computes expected torque using the existing KDL dynamics model (gravity + Coriolis + friction)
3. Calculates the difference as the external torque estimate
4. Applies a low-pass filter to remove sensor noise
5. Publishes the result on `~/external_force` at the controller rate (100 Hz)

---

## 3. Method

### 3.1 Algorithm

At each control cycle (100 Hz):

```
tau_ext_raw[i] = tau_motor[i] - tau_ff[i]

tau_ext[i] = alpha * tau_ext_raw[i] + (1 - alpha) * tau_ext_prev[i]
```

Where:
- `tau_motor[i]` = actual motor torque from hardware (effort state interface, read from motor current sensing)
- `tau_ff[i]` = expected torque from dynamics model: `scale * (gravity + Coriolis) + friction`
- `alpha` = low-pass filter coefficient (configurable, default 0.05 → ~0.8 Hz cutoff at 100 Hz update rate)
- `tau_ext[i]` = filtered external torque estimate

### 3.2 Signal Flow

```
┌─────────────────────────────────────────────────────────┐
│                 Compliance Controller (100 Hz)           │
│                                                         │
│  ┌──────────────┐     ┌─────────────────┐               │
│  │  Joint State  │     │   KDL Dynamics   │               │
│  │  Interfaces   │     │   Model (URDF)   │               │
│  │              │     │                 │               │
│  │  q, qdot     │────▶│  gravity(q)     │               │
│  │              │     │  coriolis(q,qd) │               │
│  │  tau_motor   │     │  friction(qd)   │               │
│  └──────┬───────┘     └────────┬────────┘               │
│         │                      │                        │
│         │ actual               │ expected               │
│         │ torque               │ torque (tau_ff)        │
│         │                      │                        │
│         ▼                      ▼                        │
│    ┌──────────────────────────────┐                      │
│    │   tau_ext = tau_motor - tau_ff│                      │
│    └──────────┬───────────────────┘                      │
│               │                                         │
│               ▼                                         │
│    ┌─────────────────────┐                               │
│    │  Low-Pass Filter     │                               │
│    │  alpha = 0.05        │                               │
│    └──────────┬──────────┘                               │
│               │                                         │
│               ▼                                         │
│    ┌─────────────────────┐                               │
│    │  ~/external_force    │ Published at 100 Hz          │
│    │  [tau_ext_1..7]      │                               │
│    └─────────────────────┘                               │
└─────────────────────────────────────────────────────────┘
```

### 3.3 Why This Works

Newton's second law for a robot joint:

```
tau_motor = M(q) * qddot + C(q, qdot) * qdot + G(q) + F(qdot) + tau_ext
```

At steady state (qddot ≈ 0, qdot ≈ 0):

```
tau_motor ≈ G(q) + F_offset + tau_ext
```

Our model computes `tau_ff ≈ G(q) + F(qdot)`, so:

```
tau_ext ≈ tau_motor - tau_ff
```

### 3.4 Limitations

1. **Model accuracy**: The estimate is only as good as the dynamics model. Unmodeled friction, gear backlash, and URDF inertia errors appear as bias in `tau_ext`.
2. **No acceleration term**: We don't account for `M(q) * qddot` (inertia). During fast motion, this creates transient errors.
3. **Sensor noise**: Motor current sensing has noise. The low-pass filter (alpha=0.05) attenuates this at the cost of response latency (~200ms to 95% of step).
4. **Fake hardware**: Simulation returns effort=0, so `tau_ext = -tau_ff` in fake hardware (not useful).

---

## 4. Implementation Details

### Files Modified

| File | Change |
|------|--------|
| `compliance_controller.hpp` | Added `ext_force_pub_`, `tau_ext_filtered_`, `ext_force_alpha_` |
| `compliance_controller.cpp` | Added effort to `state_interface_configuration()`, tau_ext computation in `update()`, publisher setup in `on_configure()` |
| `compliance_controller.yaml` | Added `ext_force_alpha: 0.05` for both arms |

### State Interface Change

Before Task 1.2b:
```
state_interfaces_[i*2 + 0] = position
state_interfaces_[i*2 + 1] = velocity
```

After Task 1.2b:
```
state_interfaces_[i*3 + 0] = position
state_interfaces_[i*3 + 1] = velocity
state_interfaces_[i*3 + 2] = effort (actual motor torque)
```

### Configuration

```yaml
# In compliance_controller.yaml
ext_force_alpha: 0.05  # Low-pass filter coefficient
                       # Higher = faster response, more noise
                       # Lower = smoother, slower response
                       # 0.05 at 100 Hz → ~0.8 Hz cutoff frequency
```

---

## 5. Results

### 5.1 Simulation Verification

- `~/external_force` topic exists and publishes at controller rate ✅
- Values near zero in fake hardware (expected — effort state = 0) ✅
- Build succeeds with zero warnings (`-Wall -Wextra`) ✅

### 5.2 Real Hardware Results (04/29)

**Test**: Arm at J4=90°, user pushed arm 3 times at different locations.

| Reading | J1 | J2 | J3 | J4 | J5 | J6 | J7 |
|---------|------|------|------|------|------|------|------|
| Push 1 (shoulder) | **-4.56** | 0.33 | -0.06 | 0.07 | 0.05 | 0.09 | 0.03 |
| Push 2 (elbow) | 4.03 | -0.29 | 0.35 | **6.45** | **1.13** | 0.09 | 0.03 |
| Push 3 (wrist) | 3.81 | -0.48 | -0.06 | **4.37** | 0.16 | 0.09 | **1.42** |
| After release | -0.82 | -0.45 | -0.06 | 0.50 | 0.16 | 0.08 | -0.15 |

**Observations**:
- External forces correctly localize to the joint(s) being pushed
- Coupled joints show expected secondary response (e.g., pushing J4 also affects J1 and J5)
- After release, all values return to < 1.0 Nm within 2 seconds
- No oscillation or noise artifacts observed

---

## 6. What This Does vs. What Comes Next

### Current State: "The Sensor"

Task 1.2b is the **sensing layer only**. It measures the torque residual (`tau_ext`) and publishes it. It does NOT automatically adjust anything — no weight estimation, no auto-compensation. It just answers the question: *"How much unexpected force is acting on each joint right now?"*

### Two Approaches for Using This Data

There are two fundamentally different ways to close the loop:

#### Approach A: Estimate Weight → Update Model (Indirect)

```
tau_ext spike detected → estimate mass → publish to set_payload → rebuild KDL solver → tau_ff updates
```

- **How**: A higher-level node reads `~/external_force`, computes `mass = tau_ext / (g × lever_arm)`, publishes to `~/set_payload` (Task 1.2a)
- **Pro**: Physically correct model — works at any future pose
- **Con**: Needs mass estimation logic, pose-dependent calculation, separate node
- **Use case**: VLA says "pick up the bottle" → system confirms weight, updates model

#### Approach B: Direct Disturbance Compensation (Adaptive)

```
tau_ext measured → add directly to tau_ff → immediate compensation
```

- **How**: In `update()`, simply do `tau_ff[i] += gain * tau_ext_filtered_[i]`
- **Pro**: No weight estimation needed, adapts instantly to ANY disturbance (gravity, contact, friction changes)
- **Con**: Can amplify sensor noise, doesn't distinguish payload from collision, model stays "wrong"
- **Use case**: Arm grabs unknown object → automatically compensates without knowing the weight

This is what industrial robots call a **disturbance observer**. The residual torque IS the correction needed — you feed it back directly. No need to know the exact weight.

#### Which to Use?

**Both can coexist.** In practice:

| Scenario | Best Approach |
|----------|---------------|
| Grab unknown object, hold for a long time | **A** — estimate mass once, model stays correct |
| Brief contact with surface during insertion | **B** — instant compensation, no model update needed |
| Human pushes arm during collaboration | **B** — react immediately, don't update model |
| VLA specifies known object weight | **A** — use `set_payload` directly (skip estimation) |

### What We Have Now

```
✅ Task 1.2a: set_payload → rebuild solver (Approach A mechanism)
✅ Task 1.2b: tau_ext estimation + publishing (sensor layer)
🔲 Future:    Auto-estimation node (Approach A automation)
🔲 Future:    Disturbance observer mode (Approach B)
```

The foundation is in place. The `~/external_force` topic provides the raw data, and either approach (or both) can be built on top without modifying the compliance controller itself.

# Agent-C1 → Agent-O: Bimanual Teach Mode & GUI Complete

**Date:** 2026-05-08  
**From:** Agent-C1 (Controls)  
**Status:** ✅ COMPLETE — Both arms + GUI validated on real hardware

---

## What Was Done

### 1. Bimanual Teach Mode — Fully Operational
Both left and right arms now support gravity-compensated "teach mode" where
all 7 joints + gripper float freely for kinesthetic demonstration recording.

- **Mechanism**: Hardware-level detection — when stiffness (Kp) reaches the
  safety floor (kp_min), the driver substitutes `pos_actual` for `pos_target`,
  zeroing the spring force. Gravity compensation (tau_ff) keeps the arm floating.
- **Gripper**: Added teach mode detection for the gripper motor (was missing).
  At `grip_kp ≤ kp_min + 0.1`, the gripper also floats freely.

### 2. Bimanual GUI — New Feature
Single-window PyQt5 GUI for controlling both arms simultaneously:
- **Tabbed interface**: ⬅ Left Arm / ➡ Right Arm tabs
- **🎓 Teach Mode toggle**: One button enables/disables teach mode for both arms
- **Presets**: Full Stiff, Soft Wrist, Full Soft, Extra Stiff
- **Per-joint Kp/Kd sliders** with live tau_ff readout
- **Gripper control**: Open/Close buttons, position spinbox, stiffness modes
- **⬛ E-STOP**: Resets both arms + sends home trajectory

### 3. Safety Floor (kp_min) Redesigned
Lowered kp_min to prevent soft presets from accidentally triggering teach mode:

| Joint | Old kp_min | New kp_min | Lowest preset Kp |
|-------|-----------|-----------|------------------|
| J1-J3 | 15.0 | **12.0** | 15.0 (Full Soft) |
| J4    | 5.0  | **3.0**  | 12.0 (Full Soft) |
| J5-J7 | 3.0  | **2.0**  | 3.0 (Soft Wrist) |

Teach profile Kp = kp_min → only teach mode triggers floating.

### 4. Right Arm J6 Drift Fix
Right arm J6 had excessive drift in teach mode. Root cause: Coulomb friction
coefficient (Fc=0.093) with high sensitivity (k=242) caused velocity-noise-driven
drift. Fix: zeroed Fc for right arm J6.

---

## Files Changed (for Agent-O to review)

### Hardware (C++ — requires rebuild)
- `openarm_hardware/src/v10_simple_hardware.cpp` — Gripper teach mode detection
- `openarm_hardware/include/.../v10_simple_hardware.hpp` — kp_min defaults

### Configuration
- `openarm_description/config/arm/v10/control_gains.yaml` — kp_min values
- `compliance_controller/config/compliance_controller.yaml` — kp_min, tau_ff_scale, friction

### Scripts
- `compliance_controller/scripts/impedance_gui.py` — Bimanual GUI with teach toggle
- `compliance_controller/scripts/impedance_profile_manager.py` — Teach profile Kp

### Documentation
- `compliance_controller/Enable_Teaching_Biarm.md` — **NEW** step-by-step setup guide
- `compliance_controller/C1_TO_C2_GRIPPER_FIX.md` — Updated C2 handoff note

---

## Impact on Phase 3

This work **unblocks bimanual data collection** for VLA training:
1. Both arms can be guided simultaneously via teach mode
2. Joint states recorded from `/joint_states` capture the demonstration
3. Camera frames from wrist/head cameras can be recorded in parallel
4. The GUI provides a user-friendly interface for operators

**Next steps for Phase 3:**
- C2 implements `record_episode.py` for synchronized joint + camera recording
- C2 sets up LeRobot dataset format for Pi 0.5 training
- Operator uses teach mode GUI to record demonstration episodes

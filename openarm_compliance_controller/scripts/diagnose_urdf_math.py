#!/usr/bin/env python3
"""
Left Arm Coordinate Diagnostic — Automated Test
=================================================
Publishes test joint values to TF and compares the resulting
end-effector positions to determine if joint values need negation.

This does NOT move the real robot. It only tests the URDF math.

Run while bimanual bringup is active:
  python3 ~/ros2_ws/src/impedance_control/openarm_compliance_controller/scripts/diagnose_urdf_math.py
"""
import math
import numpy as np

def rotation_x(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1,0,0,0],[0,c,-s,0],[0,s,c,0],[0,0,0,1]])

def rotation_y(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1]])

def rotation_z(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]])

def translation(x, y, z):
    T = np.eye(4)
    T[0,3], T[1,3], T[2,3] = x, y, z
    return T

def rpy_to_rotation(r, p, y):
    return rotation_z(y) @ rotation_y(p) @ rotation_x(r)

def axis_rotation(axis, angle):
    """Rotation around arbitrary axis [ax, ay, az] by angle."""
    ax, ay, az = axis
    if abs(ax) > 0.5:
        return rotation_x(angle * ax)
    elif abs(ay) > 0.5:
        return rotation_y(angle * ay)
    else:
        return rotation_z(angle * az)

def main():
    # From the live URDF:
    arms = {
        'left': {
            'base_xyz': [0.0, 0.031, 0.698],
            'base_rpy': [-math.pi/2, 0, 0],
            'joints': [
                {'axis': [0,0,1],   'xyz': [0,0,0.0625],     'rpy': [0,0,0]},                      # J1
                {'axis': [-1,0,0],  'xyz': [-0.0301,0,0.06],  'rpy': [-math.pi/2,0,0]},             # J2
                {'axis': [0,0,1],   'xyz': [0.0301,0,0.0663], 'rpy': [0,0,0]},                      # J3
                {'axis': [0,1,0],   'xyz': [0,-0.0315,0.1537],'rpy': [0,0,0]},                      # J4 (xyz Y was inverted in output, using URDF value)
                {'axis': [0,0,1],   'xyz': [0,0.0315,0.0955], 'rpy': [0,0,0]},                      # J5
                {'axis': [1,0,0],   'xyz': [0.0375,0,0.1205], 'rpy': [0,0,0]},                      # J6
                {'axis': [0,-1,0],  'xyz': [-0.0375,0,0],     'rpy': [0,0,0]},                      # J7
            ],
        },
        'right': {
            'base_xyz': [0.0, -0.031, 0.698],
            'base_rpy': [math.pi/2, 0, 0],
            'joints': [
                {'axis': [0,0,1],   'xyz': [0,0,0.0625],     'rpy': [0,0,0]},                      # J1
                {'axis': [-1,0,0],  'xyz': [-0.0301,0,0.06],  'rpy': [math.pi/2,0,0]},              # J2
                {'axis': [0,0,1],   'xyz': [0.0301,0,0.0663], 'rpy': [0,0,0]},                      # J3
                {'axis': [0,1,0],   'xyz': [0,0.0315,0.1537], 'rpy': [0,0,0]},                      # J4
                {'axis': [0,0,1],   'xyz': [0,-0.0315,0.0955],'rpy': [0,0,0]},                      # J5
                {'axis': [1,0,0],   'xyz': [0.0375,0,0.1205], 'rpy': [0,0,0]},                      # J6
                {'axis': [0,1,0],   'xyz': [-0.0375,0,0],     'rpy': [0,0,0]},                      # J7
            ],
        },
    }

    def fk(arm, q_values):
        """Forward kinematics: compute end-effector position in world frame."""
        cfg = arms[arm]
        T = translation(*cfg['base_xyz']) @ rpy_to_rotation(*cfg['base_rpy'])
        for i, jnt in enumerate(cfg['joints']):
            T = T @ translation(*jnt['xyz']) @ rpy_to_rotation(*jnt['rpy'])
            T = T @ axis_rotation(jnt['axis'], q_values[i])
        return T[:3, 3]  # position only

    print("=" * 70)
    print("URDF FORWARD KINEMATICS ANALYSIS")
    print("=" * 70)
    print()

    # Test: for each joint, apply +0.3 rad and see which direction the EE moves
    # If the same raw motor value produces mirrored EE positions, no negation needed.
    # If it produces same-side positions, negation IS needed.
    print("Test: Apply q=+0.3 rad to each joint independently")
    print("      Compare left vs right end-effector displacement")
    print()
    print(f"{'Joint':>5} | {'Left EE delta (x,y,z)':>30} | {'Right EE delta (x,y,z)':>30} | {'Y-mirror match?':>15}")
    print("-" * 90)

    q_zero = [0.0] * 7
    left_home = fk('left', q_zero)
    right_home = fk('right', q_zero)
    print(f"Home positions:")
    print(f"  Left  EE: ({left_home[0]:+.4f}, {left_home[1]:+.4f}, {left_home[2]:+.4f})")
    print(f"  Right EE: ({right_home[0]:+.4f}, {right_home[1]:+.4f}, {right_home[2]:+.4f})")
    print()

    needs_negate = []
    for j in range(7):
        q_test = [0.0] * 7
        q_test[j] = 0.3  # +0.3 rad = ~17 degrees

        left_pos = fk('left', q_test)
        right_pos = fk('right', q_test)

        left_delta = left_pos - left_home
        right_delta = right_pos - right_home

        # For a correctly mirrored arm, the Y component should be opposite
        # (since the arms are on opposite sides), but X and Z should match
        # If they're the same, the motor value needs negation for one arm
        
        # Check: does left delta ≈ mirror(right delta)?
        # Mirror means: dx same, dy negated, dz same
        mirror_match = (
            np.sign(left_delta[0]) == np.sign(right_delta[0]) and  # X same direction
            np.sign(left_delta[1]) != np.sign(right_delta[1]) and  # Y opposite (mirrored)
            np.sign(left_delta[2]) == np.sign(right_delta[2])       # Z same direction
        )
        # Handle near-zero deltas
        for k in range(3):
            if abs(left_delta[k]) < 1e-6 and abs(right_delta[k]) < 1e-6:
                mirror_match = True  # both near zero = no information

        status = "✅ OK" if mirror_match else "❌ NEEDS NEGATE"
        if not mirror_match:
            needs_negate.append(j + 1)

        print(f"  J{j+1}  | ({left_delta[0]:+.4f}, {left_delta[1]:+.4f}, {left_delta[2]:+.4f}) | "
              f"({right_delta[0]:+.4f}, {right_delta[1]:+.4f}, {right_delta[2]:+.4f}) | {status}")

    print()
    print("=" * 70)
    if needs_negate:
        print(f"RESULT: Joints needing negation for left arm: {needs_negate}")
        print(f"        reflect[] = ", end="")
        for j in range(7):
            print(f"{-1 if (j+1) in needs_negate else 1}", end=", " if j < 6 else "\n")
    else:
        print("RESULT: No joints need negation — raw motor values are correct!")
    print("=" * 70)

    # Also test with negated values to verify
    print()
    print("Verification: Test with negated left arm values")
    for j in range(7):
        q_test_left = [0.0] * 7
        q_test_right = [0.0] * 7
        val = 0.3
        q_test_right[j] = val
        q_test_left[j] = -val if (j+1) in needs_negate else val

        left_pos = fk('left', q_test_left)
        right_pos = fk('right', q_test_right)
        left_delta = left_pos - left_home
        right_delta = right_pos - right_home

        mirror_err = np.sqrt(
            (left_delta[0] - right_delta[0])**2 +
            (left_delta[1] + right_delta[1])**2 +  # Y should be opposite
            (left_delta[2] - right_delta[2])**2
        )
        print(f"  J{j+1}: mirror error = {mirror_err:.6f} m {'✅' if mirror_err < 0.01 else '❌'}")

if __name__ == '__main__':
    main()

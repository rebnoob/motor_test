# kinematics.py
import numpy as np
from config import clamp

# leg geometry (mm)  +x fwd, +y left, +z DOWN
l1, l2, l3 = 90.0, 190.0, 120.0

def fk_leg(theta1, theta2, theta3, s_y=+1):
    """
    Forward kinematics (output-side radians) -> (x,y,z) mm, also returns planar z'
    """
    z_p = l2*np.cos(theta2) + l3*np.cos(theta2 + theta3)
    x   = l2*np.sin(theta2) + l3*np.sin(theta2 + theta3)
    y   = s_y*l1*np.cos(theta1) - z_p*np.sin(theta1)
    z   = s_y*l1*np.sin(theta1) + z_p*np.cos(theta1)
    return np.array([x, y, z]), z_p

def ik_leg(x, y, z, s_y=+1):
    """
    Inverse kinematics (mm) -> list of (t1,t2,t3) in output-side radians.
    Returns possibly multiple elbow-up/down solutions.
    """
    sols = []
    r_yz = np.hypot(y, z)
    if r_yz < l1 - 1e-9:
        return sols
    zprime_mag = np.sqrt(max(r_yz**2 - l1**2, 0.0))
    for z_p in (zprime_mag, -zprime_mag):
        theta1 = np.arctan2(z, y) - np.arctan2(z_p, s_y*l1)
        r = np.hypot(x, z_p)
        c3 = (r*r - l2*l2 - l3*l3) / (2.0*l2*l3)
        if c3 < -1.0 or c3 > 1.0:
            continue
        for knee_sign in (1.0, -1.0):  # elbow-down/up
            theta3 = knee_sign * np.arccos(np.clip(c3, -1.0, 1.0))
            theta2 = np.arctan2(x, z_p) - np.arctan2(l3*np.sin(theta3), l2 + l3*np.cos(theta3))
            sols.append((theta1, theta2, theta3))
    return sols

def pick_closest_solution(prev_angles, candidates):
    if not candidates:
        return None
    if prev_angles is None:
        elbow_down = [c for c in candidates if c[2] >= 0]
        return elbow_down[0] if elbow_down else candidates[0]
    pa = np.array(prev_angles)
    best, best_dist = None, float("inf")
    for c in candidates:
        ca = np.array(c)
        d = (ca - pa + np.pi) % (2*np.pi) - np.pi
        dist = np.linalg.norm(d)
        if dist < best_dist:
            best_dist, best = dist, c
    return best

def clamp_to_workspace(x, y, z, s_y=+1):
    """
    Simple workspace clamp so IK stays solvable.
    You can tune these bounds based on your actual leg.
    """
    # rough bounds from your GUI earlier
    X_MIN, X_MAX = -50.0, 520.0
    Y_MIN, Y_MAX = -260.0, 260.0
    Z_MIN, Z_MAX = -30.0, 520.0  # +z down
    return (
        clamp(x, X_MIN, X_MAX),
        clamp(y, Y_MIN, Y_MAX),
        clamp(z, Z_MIN, Z_MAX),
    )

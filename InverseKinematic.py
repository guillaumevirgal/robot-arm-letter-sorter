#Old IK code
import numpy as np

# -----------------------------------------------
# Link parameters
# -----------------------------------------------
L_0 = 20    # Base height (mm)
L_1 = 20    # Shoulder to elbow
L_2 = 31    # Elbow to gripper tip


JOINT_LIMITS = {
    # (min_deg, max_deg) ; hardware set these once you know your physical stops
    'theta_1': (-170, 170),   # Base: ±170° from zero
    'theta_2': (-90,  90),    # Shoulder: ±90° from horizontal
    'theta_3': (-150,  0),    # Elbow: elbow-up only (negative by convention)
    'theta_4': (-90,  90),    # Wrist compensation
}

def check_joint_limits(angles_deg):
    """
    Raises ValueError if any joint angle exceeds its physical limit.
    Call this after compute_ik() before sending over I2C.
    Limits must be calibrated against the physical hard-stops.
    """
    names = ['theta_1', 'theta_2', 'theta_3', 'theta_4']
    for name, angle in zip(names, angles_deg):
        lo, hi = JOINT_LIMITS[name]
        if not (lo <= angle <= hi):
            raise ValueError(
                f"Joint limit violation: {name} = {angle:.1f}° "
                f"(allowed [{lo}, {hi}])"
            )


def compute_ik(x_target, y_target, z_target, ux , uy, uz):
    """
    Compute joint angles for a 4R robot arm with horizontal gripper.
    Returns [theta_1, theta_2, theta_3, theta_4] in degrees.
    Raises ValueError if target is out of reach.
    """
    

    # Wrist center
    x = x_target - L_3 * ux
    y = y_target - L_3 * uy
    z = z_target - L_3 * uz

    # theta_1 (base)
    xp = np.sqrt(x**2 + y**2)
    zp = z - L_0
    theta_1 = np.arctan2(y, x)

    # theta_3 (elbow)
    ctheta_3 = (xp**2 + zp**2 - L_1**2 - L_2**2) / (2 * L_1 * L_2)
    if ctheta_3 > 1:
        raise ValueError(f'Target is out of reach (ctheta_3 = {ctheta_3:.3f})')
    if ctheta_3 < -1:
        raise ValueError(f'Target is too close of reach (ctheta_3 = {ctheta_3:.3f})')

    stheta_3 = -np.sqrt(1 - ctheta_3**2)  # elbow up
    #stheta_3 = np.sqrt(1 - ctheta_3**2)  # elbow down

    theta_3 = np.arctan2(stheta_3, ctheta_3)

    # theta_2 (shoulder)
    beta = np.arctan2(zp, xp)
    cpsi = (xp**2 + zp**2 + L_1**2 - L_2**2) / (2 * L_1 * np.sqrt(xp**2 + zp**2))
    spsi = (L_2 * stheta_3) / np.sqrt(xp**2 + zp**2)
    psi  = np.arctan2(spsi, cpsi)
    theta_2 = beta - psi

    # theta_4 (wrist); pi/2 keeps gripper horizontal
    theta_4 = -(theta_2 + theta_3) + np.pi / 2

    return [
        np.degrees(theta_1),
        np.degrees(theta_2),
        np.degrees(theta_3),
        np.degrees(theta_4)
    ]
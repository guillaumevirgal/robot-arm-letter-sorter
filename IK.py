#IK Pi
import numpy as np

# -----------------------------------------------
# Link parameters
# -----------------------------------------------
L_0 = 20    # Base height (mm)
L_1 = 20    # Shoulder to elbow
L_2 = 31    # Elbow to gripper tip


JOINT_LIMITS = {
    # (min_deg, max_deg) ; hardware set these once you know your physical stops
    'theta_1': (-180, 180),   # Base: ±180° from zero
    'theta_2': (0, 256),      # Shoulder: 0° (down) to 103° (just below hard stop at 108°)
    'theta_3': (-200,  0),    # Elbow: elbow-up only (negative by convention)
}

def check_joint_limits(angles_deg):
    """
    Raises ValueError if any joint angle exceeds its physical limit.
    Call this after compute_ik() before sending over I2C.
    Limits must be calibrated against the physical hard-stops.
    """
    names = ['theta_1', 'theta_2', 'theta_3']
    for name, angle in zip(names, angles_deg):
        lo, hi = JOINT_LIMITS[name]
        if not (lo <= angle <= hi):
            raise ValueError(
                f"Joint limit violation: {name} = {angle:.1f}° "
                f"(allowed [{lo}, {hi}])"
            )


def compute_ik(x_target, y_target, z_target):
    """
    Compute joint angles for a 4R robot arm with horizontal gripper.
    Returns [theta_1, theta_2, theta_3, theta_4] in degrees.
    Raises ValueError if target is out of reach.
    """
    
    x = x_target 
    y = y_target 
    z = z_target 

    # theta_1 (base)
    theta_1 = np.arctan2(y, x)

    # theta_3 (elbow)
    r = np.sqrt(x**2 + y**2)
    zp = z - L_0
    d = np.sqrt(r**2 + zp**2)
    ctheta_3 = (d**2 - L_1**2 - L_2**2) / (2 * L_1 * L_2)
    if ctheta_3 > 1:
        raise ValueError(f'Target is out of reach (ctheta_3 = {ctheta_3:.3f})')
    if ctheta_3 < -1:
        raise ValueError(f'Target is too close of reach (ctheta_3 = {ctheta_3:.3f})')

    stheta_3 = -np.sqrt(1 - ctheta_3**2)  # elbow up
    #stheta_3 = np.sqrt(1 - ctheta_3**2)  # elbow down

    theta_3 = np.arctan2(stheta_3, ctheta_3)

    # theta_2 (shoulder)
    cbeta = r/d
    sbeta = zp/d
    beta = np.arctan2(sbeta, cbeta)
    cpsi = (L_1**2 + d**2 - L_2**2) / (2 * L_1 * d)
    spsi = (L_2 * stheta_3) / d
    psi  = np.arctan2(spsi, cpsi)
    theta_2 = beta - psi

    return [
        np.degrees(theta_1),
        np.degrees(theta_2),
        np.degrees(theta_3),
    ]
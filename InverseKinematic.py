import numpy as np

# -----------------------------------------------
# Link parameters
# -----------------------------------------------
D_1 = 20    # Base height (mm)
L_1 = 25    # Shoulder to elbow
L_2 = 25    # Elbow to wrist
L_3 = 5     # Wrist to gripper tip

def compute_ik(x_target, y_target, z_target):
    """
    Compute joint angles for a 4R robot arm with horizontal gripper.
    Returns [theta_1, theta_2, theta_3, theta_4] in degrees.
    Raises ValueError if target is out of reach.
    """
    # Gripper approach vector (horizontal)
    norm_xy = np.sqrt(x_target**2 + y_target**2)
    ux = x_target / norm_xy
    uy = y_target / norm_xy
    uz = 0

    # Wrist center
    x = x_target - L_3 * ux
    y = y_target - L_3 * uy
    z = z_target - L_3 * uz

    # theta_1 (base)
    xp = np.sqrt(x**2 + y**2)
    zp = z - D_1
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
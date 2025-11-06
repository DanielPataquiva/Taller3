# robot_2r_pyqt.py
import math
import numpy as np

L1 = 10.0  # cm
L2 = 11.0  # cm

def fk(theta1, theta2):
    """Cinemática directa (radianes -> cm)."""
    x0, y0 = 0, 0
    x1 = L1 * math.cos(theta1)
    y1 = L1 * math.sin(theta1)
    x2 = x1 + L2 * math.cos(theta1 + theta2)
    y2 = y1 + L2 * math.sin(theta1 + theta2)
    return [(x0, y0), (x1, y1), (x2, y2)]

def inverse_kinematics(x, y, elbow='down'):
    """Cinemática inversa analítica para 2R."""
    r = math.hypot(x, y)
    if r > (L1 + L2) or r < abs(L1 - L2):
        return None

    cos_t2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_t2 = max(min(cos_t2, 1.0), -1.0)

    theta2 = math.acos(cos_t2)
    if elbow == 'up':
        theta2 = -theta2

    k1 = L1 + L2 * math.cos(theta2)
    k2 = L2 * math.sin(theta2)
    theta1 = math.atan2(y, x) - math.atan2(k2, k1)

    return theta1, theta2

def trajectory(t1_start, t2_start, t1_end, t2_end, steps=100):
    """Genera trayectorias lineales de juntas."""
    t1_traj = np.linspace(t1_start, t1_end, steps)
    t2_traj = np.linspace(t2_start, t2_end, steps)
    return list(zip(t1_traj, t2_traj))

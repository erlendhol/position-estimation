import numpy as np
import math

g_const = 9.80665

def get_pitch(a, degrees=True):
    norm = np.linalg.norm(a)
    phi = 0
    if norm > 0:
        a_norm = -np.divide(a.T, norm)
        phi = math.atan2(a_norm[0], math.sqrt(a_norm[1]**2 + a_norm[2]**2))
    if degrees:
        return math.degrees(phi)
    else:
        return phi

def get_roll(a, degrees=True):
    norm = np.linalg.norm(a)
    theta = 0
    if norm > 0:
        a_norm = np.divide(a.T, norm)
        theta = math.atan2(a_norm[1], math.sqrt(a_norm[0]**2 + a_norm[2]**2))
    if degrees:
        return math.degrees(theta)
    else:
        return theta

def get_yaw(pitch, roll, m, degrees=True):
    phi = pitch
    theta = roll
    c_phi = math.cos(math.radians(phi))
    s_phi = math.sin(math.radians(phi))
    c_theta = math.cos(math.radians(theta))
    s_theta = math.sin(math.radians(theta))
    mag = m.T
    # norm = np.linalg.norm(m)
    norm = math.sqrt((mag[0]**2) + (mag[1]**2) + (mag[2]**2))
    if norm > 0:
        m_norm = np.divide(mag, norm)

        XH = m_norm[0]*math.cos(math.radians(pitch)) + m_norm[1]*math.sin(math.radians(pitch))*math.sin(math.radians(roll)) + m_norm[2]*math.sin(math.radians(pitch))*math.cos(math.radians(roll))

        YH = m_norm[1]*math.cos(math.radians(roll)) + m_norm[2]*math.sin(math.radians(roll))

        psi = math.atan2(-YH, XH)

        if degrees:
            return math.degrees(psi)
        else:
            return psi
    else:
        return 0

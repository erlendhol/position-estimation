import numpy as np
import math

g_const = 9.80665

# def get_pitch(a):
#     norm = np.linalg.norm(a)
#     a_norm = np.divide(a.T, norm)
#     phi = math.atan2(-a_norm[0], math.sqrt(a_norm[1]**2 + a_norm[2]**2))
#     return math.degrees(phi)

# def get_roll(a):
#     norm = np.linalg.norm(a)
#     a_norm = np.divide(a.T, norm)
#     theta = math.atan2(a_norm[1], a_norm[2])
#     return math.degrees(theta)

def get_pitch(a):
    norm = np.linalg.norm(a)
    phi = 0
    if norm > 0:
        a_norm = np.divide(a.T, norm)
        phi = math.atan2(a_norm[0], math.sqrt(a_norm[0]**2 + a_norm[2]**2))
    return math.degrees(phi)

def get_roll(a):
    norm = np.linalg.norm(a)
    theta = 0
    if norm > 0:
        a_norm = np.divide(a.T, norm)
        theta = math.atan2(a_norm[1], math.sqrt(a_norm[1]**2 + a_norm[2]**2))
    return math.degrees(theta)

def get_yaw(a, m):
    phi = get_pitch(a)
    theta = get_roll(a)
    c_phi = math.cos(math.radians(phi))
    s_phi = math.sin(math.radians(phi))
    c_theta = math.cos(math.radians(theta))
    s_theta = math.sin(math.radians(theta))

    norm = np.linalg.norm(m)
    m_norm = np.divide(m.T, norm)
    R = np.array([[c_theta, s_theta*s_phi, s_theta*c_phi],
                  [0, c_phi, -s_phi],
                  [-s_theta, c_theta*s_phi, c_theta*c_phi]])
    b = R @ m_norm

    psi = math.atan2(-b[1], b[0])
    return math.degrees(psi)



if __name__ == "__main__":
    acc = np.array([[-2.82, -0.58, 9.28]])
    mag = np.array([[-40.06, 25.38, -80.06]])
    phi = get_pitch(acc)
    print('Phi: ', phi)
    theta = get_roll(acc)
    print('Theta: ', theta)
    psi = get_yaw(acc, mag)
    print('Psi: ', psi+180)
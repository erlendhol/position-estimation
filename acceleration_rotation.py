import numpy as np
import math
import pandas as pd
import time
"""
Represents an acceleration measurement with given angles in roll and yaw, 
and translates these to the fixed frame.
"""
def rotation(phi, psi):
    c_phi = math.cos(math.radians(phi))
    c_psi = math.cos(math.radians(psi))
    s_psi = math.sin(math.radians(psi))
    s_phi = math.sin(math.radians(phi))

    R = np.array([[c_psi, s_psi*c_phi, s_psi*s_phi],
                  [-s_psi, c_psi*c_phi, s_phi*c_psi],
                  [0, -s_phi, c_phi]])
    return R
print(rotation(10, 20))
acceleration_cframe = np.array([[0, -9.025, 3.558]]).T
print(acceleration_cframe)
acceleration_rotated = np.matmul(rotation(67.584, 64.398), acceleration_cframe)
print(acceleration_rotated)
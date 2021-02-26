import numpy as np
import matplotlib.pyplot as plt
import math
from mpl_toolkits.mplot3d import Axes3D


x_sb = 15
y_sb = 8
z_sb = 1

pos_sb = np.array([x_sb, y_sb, z_sb])

def update_pos_bc(l, phi, psi):
    x_bc = l * math.sin(math.radians(psi))
    y_bc = l * math.cos(math.radians(psi))
    z_bc = l * math.sin(math.radians(phi))
    return np.array([x_bc, y_bc, z_bc])

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([0,30])
ax.set_ylim([0,30])
ax.set_zlim([0,30])
pos_bc = update_pos_bc(17.5, 15, 7.8)
ax.quiver(0, 0, 0, pos_sb[0], pos_sb[1], pos_sb[2], color = 'blue', normalize = False)
ax.quiver(pos_sb[0], pos_sb[1], pos_sb[2], pos_bc[0], pos_bc[1], pos_bc[2], color = 'blue', normalize = False)
pos_sc = pos_sb + pos_bc
ax.quiver(0, 0, 0, pos_sc[0], pos_sc[1], pos_sc[2], color = 'red', normalize = False)

plt.show()

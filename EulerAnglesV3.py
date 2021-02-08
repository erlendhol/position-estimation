import numpy as np
import math as m
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D

# Defining the three elementary rotations
def Rx(theta):
    return np.matrix([[1, 0           , 0            ],
                      [0, m.cos(theta), -m.sin(theta)],
                      [0, m.sin(theta), m.cos(theta) ]])

def Ry(theta):
    return np.matrix([[m.cos(theta),  0, m.sin(theta)],
                      [0,             1, 0           ],
                      [-m.sin(theta), 0, m.cos(theta)]])

def Rz(theta):
    return np.matrix([[m.cos(theta), -m.sin(theta), 0],
                      [m.sin(theta), m.cos(theta) , 0],
                      [0           , 0            , 1]])

# The triplet of the angles used in these elementary
# rotations are the Euler angles and are normally indicated (φ, θ, ψ).

# Example: We choose three euler angles and then we multiply the elementary
# rotation matrices R ZYZ.
phi = m.pi/2
theta = m.pi/4
psi = m.pi/2
#print("phi = ", phi)
#print("theta = ", theta)
#print("psi = ", psi)

R = Rz(psi) * Ry(theta) * Rx(phi)
#print(np.round(R, decimals=2))

v1 = np.array([[1],[0],[0]])
v2 = R*v1
v3 = R*v2

print("Before rotation: ", v1)
print("After rotation: ", np.round(v2, decimals=2))

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# # Cartesian axes
# ax.quiver(-1, 0, 0, 3, 0, 0, color='#aaaaaa',linestyle='dashed')
# ax.quiver(0, -1, 0, 0,3, 0, color='#aaaaaa',linestyle='dashed')
# ax.quiver(0, 0, -1, 0, 0, 3, color='#aaaaaa',linestyle='dashed')
#
# ax.set_xlim([-1.5, 1.5])
# ax.set_ylim([-1.5, 1.5])
# ax.set_zlim([-1.5, 1.5])
#
# ##labelling the axes
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')

for i in range(0,10):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    # Cartesian axes
    ax.quiver(-1, 0, 0, 3, 0, 0, color='#aaaaaa',linestyle='dashed')
    ax.quiver(0, -1, 0, 0,3, 0, color='#aaaaaa',linestyle='dashed')
    ax.quiver(0, 0, -1, 0, 0, 3, color='#aaaaaa',linestyle='dashed')

    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_zlim([-1.5, 1.5])

    ##labelling the axes
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    # Vector before rotation
    ax.quiver(0, 0, 0, v1[[0]], v1[[1]], v1[[2]], color='b')
    v1 = R*v1
    #fig.canvas.draw()

    plt.show()

# Vector after rotation
#ax.quiver(0, 0, 0, v2[[0]], v2[[1]], v2[[2]], color='r')

#ax.quiver(0, 0, 0, v3[[0]], v3[[1]], v3[[2]], color='g')

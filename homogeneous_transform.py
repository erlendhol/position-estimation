"""
Code snippet is based on "Rotation Matrix To Euler Angles" from Learn OpenCV (Satya Mallick, 2016)
https://learnopencv.com/rotation-matrix-to-euler-angles/


"""


import numpy as np
import math
import matplotlib.pyplot as plt
import pandas as pd
import time

from mpl_toolkits.mplot3d import Axes3D

#data = pd.read_csv('orientation.csv')
#orientation = data.iloc[:, 1:4].values

#print(orientation[0, :])

def euler_to_quaternion(r):
    (yaw, pitch, roll) = (r[2], r[1], r[0])
    (yaw, pitch, roll) = (math.radians(yaw), math.radians(pitch), math.radians(roll))
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return np.array([qw, qx, qy, qz])


def quaternion_to_euler(q):
    (w, x, y, z) = (q[0], q[1], q[2], q[3])
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [roll, pitch, yaw]

def quat_to_rot_matrix(q):
    R = np.array([[q[0]**2 + q[1]**2 - q[2]**2 - q[3]**2, 2*(q[1]*q[2]-q[0]*q[3]), 2*(q[0]*q[2] + q[1]*q[3]), 0],
                  [2*(q[1]*q[2]+q[0]*q[3]), q[0]**2 - q[1]**2 + q[2]**2 - q[3]**2, 2*(q[2]*q[3]-q[0]*q[1]), 0],
                  [2*(q[1]*q[3]-q[0]*q[2]), 2*(q[0]*q[1] + q[2]*q[3]), q[0]**2 - q[1]**2 - q[2]**2 +q[3]**2, 0],
                  [0, 0, 0, 1]])
    return R



def rot_matrix_to_euler(r):
    sy = math.sqrt(r[0, 0] * r[0, 0]+ r[1, 0] * r[1, 0])

    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(r[2, 1], r[2,2])
        pitch = math.atan2(-r[2, 0], sy)
        yaw = math.atan2(r[1, 0], r[0, 0])
    else:
        roll = math.atan2(-r[1, 2], r[1, 1])
        pitch = math.atan2(-r[2, 0], sy)
        yaw = 0
    return np.array([math.degrees(roll), math.degrees(pitch), math.degrees(yaw)])

"""
Parameter "pos" is a 3x1 array which includes x, y, and z position.
"""
def pos_to_trans_matrix(pos):
    (new_x, new_y, new_z) = (pos[0], pos[1], pos[2])
    trans = np.array([[1, 0, 0, new_x], [0, 1, 0, new_y], [0, 0, 1, new_z], [0, 0, 0, 1]])
    return trans

def get_pos(representation):
    pos = representation[0:3, 3]
    return pos

def get_orientation(representation):
    """
    my function\n
    :param representation 

    """
    rot_matrix = representation[0:3, 0:3]
    orientation = rot_matrix_to_euler(rot_matrix)
    return orientation

q = np.array([-0.597, -0.002, -0.761, 0.254])
r = quat_to_rot_matrix(q)
print(r)


if __name__ == "__main__":
    representation = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    new_pos = [3, 2, 1]
    new_orientation = [6.1231, -24.123, 0.12312]
    new_quat = euler_to_quaternion(new_orientation)
    new_representation = np.matmul(np.matmul(pos_to_trans_matrix(new_pos), quat_to_rot_matrix(new_quat)), representation)
    print(new_representation, "\n")
    print("Pos: ", get_pos(new_representation))
    print("Orientation: ", get_orientation(new_representation))

    rep_pos = get_pos(new_representation)
    rep_quat = euler_to_quaternion(get_orientation(new_representation))

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    # Cartesian axes
    ax.quiver(-1, 0, 0, 3, 0, 0, color='#aaaaaa',linestyle='dashed')
    ax.quiver(0, -1, 0, 0, 3, 0, color='#aaaaaa',linestyle='dashed')
    ax.quiver(0, 0, -1, 0, 0, 3, color='#aaaaaa',linestyle='dashed')

    ax.set_xlim([-5, 5])
    ax.set_ylim([-5, 5])
    ax.set_zlim([-5, 5])

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    ax.quiver(rep_pos[0], rep_pos[1], rep_pos[2], rep_quat[1], rep_quat[2], rep_quat[3] , color='b', normalize=True)

    fig.canvas.draw()

    plt.show()


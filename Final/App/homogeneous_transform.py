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

def euler_to_quaternion(r):
    """
    Converts from Euler angles(degrees) to Quaternions \n
    :param r: array containing Euler angles(degrees): Roll, Pitch and Yaw
    """
    (yaw, pitch, roll) = (r[2], r[1], r[0])
    (yaw, pitch, roll) = (math.radians(yaw), math.radians(pitch), math.radians(roll))
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return np.array([qw, qx, qy, qz])


def quaternion_to_euler(q):
    """
    Converts quaternion to Euler angles \n
    :param q: quaternion as a list: qw, qx, qy, qz
    """
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
    """
    Converts quaternion to a 4x4 rotation matrix. \n
    :param q: quaternion as a list: qw, qx, qy, qz
    """
    R = np.array([[q[0]**2 + q[1]**2 - q[2]**2 - q[3]**2, 2*(q[1]*q[2]-q[0]*q[3]), 2*(q[0]*q[2] + q[1]*q[3]), 0],
                  [2*(q[1]*q[2]+q[0]*q[3]), q[0]**2 - q[1]**2 + q[2]**2 - q[3]**2, 2*(q[2]*q[3]-q[0]*q[1]), 0],
                  [2*(q[1]*q[3]-q[0]*q[2]), 2*(q[0]*q[1] + q[2]*q[3]), q[0]**2 - q[1]**2 - q[2]**2 +q[3]**2, 0],
                  [0, 0, 0, 1]])
    return R

def euler_to_rot_matrix(r, only_Z = False):
    (psi, theta, phi) = (r[2], r[1], r[0])
    c_phi = math.cos(math.radians(phi))
    s_phi = math.sin(math.radians(phi))
    c_theta = math.cos(math.radians(theta))
    s_theta = math.sin(math.radians(theta))
    c_psi = math.cos(math.radians(psi))
    s_psi = math.sin(math.radians(psi))

    Rx = np.array([[1, 0, 0, 0],
                   [0, c_phi, -s_phi, 0],
                   [0, s_phi, c_phi, 0],
                   [0, 0, 0, 1]])
    Ry = np.array([[c_theta, 0, s_theta, 0],
                   [0, 1, 0, 0],
                   [-s_theta, 0, c_theta, 0],
                   [0, 0, 0, 1]])
    Rz = np.array([[c_psi, -s_psi, 0, 0],
                   [s_psi, c_psi, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])

    if only_Z:
        return Rz
    else:
        R = Rz @ Ry @ Rx
        return R

def to_3x3_rot_matrix(m):
    """
    Extracts a 3x3 matrix from a larger than 3x3 matrix \n
    :param m: matrix to be converted
    """
    R = np.array([[m[0, 0], m[0, 1], m[0, 2]],
                    [m[1, 0], m[1, 1], m[1, 2]],
                    [m[2, 0], m[2, 1], m[2, 2]]])
    return R


def get_rot_matrix_list(data):
    """
    Creates a list of rotation matrices. \n
    :param data: dataframe to extract values.
    """
    orientation = data.iloc[:, 1:4].values
    rot_matrices = []
    for i in range (len(orientation)):
        quat = euler_to_quaternion(orientation[i, :])
        rot_matrix_4x4 = quat_to_rot_matrix(quat)
        rot_matrix = to_3x3_rot_matrix(rot_matrix_4x4)
        rot_matrices.append(rot_matrix)
    return rot_matrices


def rot_matrix_to_euler(r):
    """
    Converts a rotation matrix to euler angles(degrees)\n
    :param r: rotation matrix to convert
    """
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


def pos_to_trans_matrix(pos):
    """
    Creates a 4x4 translation matrix from given coordinates\n
    :param pos: how far the object should translate in x, y and z direction
    """
    (new_x, new_y, new_z) = (pos[0], pos[1], pos[2])
    trans = np.array([[1, 0, 0, new_x], [0, 1, 0, new_y], [0, 0, 1, new_z], [0, 0, 0, 1]])
    return trans

def get_pos(representation):
    """
    Get the position from a 4x4 homogeneous transformation matrix representation\n
    :param representation: current representation
    """
    pos = representation[0:3, 3]
    return pos

def get_orientation(representation):
    """
    Get the orientation from a 4x4 homogeneous transformation matrix representation\n
    :param representation: current representation

    """
    rot_matrix = representation[0:3, 0:3]
    orientation = rot_matrix_to_euler(rot_matrix)
    return orientation

import numpy as np
import math

def multiply(q0, q1):
    """
    Multiplies two quaternions.
    BE AWARE OF THE ORDER, QUATERNION MULTIPLICATION IS NON-COMMUTATIVE!
    q0*q1 =/= q1*q0!
    :param q0: array containing the first quaternion
    :param q1: array containing the second quaternion
    """
    # if q0.shape != (4, 1):
    #     q0 = q0.T
    # if q1.shape != (4, 1):
    #     q1 = q1.T
    w0, x0, y0, z0 = q0[0], q0[1], q0[2], q0[3]
    w1, x1, y1, z1 = q1[0], q1[1], q1[2], q1[3]
    w = w0*w1 - x0*x1 - y0*y1 - z0*z1
    x = w0*x1 + x0*w1 + y0*z1 - z0*y1
    y = w0*y1 + y0*w1 + z0*x1 - x0*z1
    z = w0*z1 + z0*w1 + x0*y1 - y0*x1
    return np.array([w, x, y, z])

def conjugate(q0):
    """
    Calculates the quaternion conjugate
    :param q0: array containing the quaternion to be conjugated
    """
    if q0.shape != (4, 1):
        q0 = q0.T
    w0, x0, y0, z0 = q0[0], q0[1], q0[2], q0[3]
    x0, y0, z0 = -x0, -y0, -z0
    return np.array([w0, x0, y0, z0])

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


def quaternion_to_euler_2(q, as_degrees=True):
    if q.shape != (4, 1):
        q = q.T
    q0, q1, q2, q3 = q[0], q[1], q[2], q[3]
    t0 = 2*(q0*q1 + q2*q3)
    t1 = 1 - 2*(q1**2 + q2**2)
    roll = math.atan2(t0, t1)
    t2 = 2*(q0*q2 - q3*q1)
    pitch = math.asin(t2)
    t3 = 2*(q0*q3 + q1*q2)
    t4 = 1 - 2*(q2**2 + q3**2)
    yaw = math.atan2(t3, t4)
    if as_degrees:
        return [np.degrees(roll), np.degrees(pitch), np.degrees(yaw)]
    else:
        return [roll, pitch, yaw]
    

def quaternion_to_euler(q, as_degrees=True):
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
    if as_degrees:
        return [np.degrees(roll), np.degrees(pitch), np.degrees(yaw)]
    else:
        return [roll, pitch, yaw]

def normalize(q):
    norm = np.linalg.norm(q)
    q_norm = 0
    if norm > 0:
        q_norm = np.divide(q, norm)
    return q_norm

if __name__ == '__main__':
    q0 = np.array([[1, 2, 3, 4]])
    q1 = np.array([[4, 3, 2, 1]])
    q2 = multiply(q0, q1)
    q3 = normalize(q2)
    print(q3)

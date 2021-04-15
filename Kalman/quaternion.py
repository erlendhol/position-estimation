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
    w0, x0, y0, z0 = q0
    w1, x1, y1, z1 = q1
    w = w0*w1 - x0*x1 - y0*y1 - z0*z1
    x = w0*x1 + x0*w1 + y0*z1 - z0*y1
    y = w0*y1 + y0*w1 + z0*x1 - x0*z1
    z = w0*z1 + z0*w1 + x0*y1 - y0*x1
    return [w, x, y, z]
    
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


if __name__ == '__main__':
    q0 = [1, 2, 3, 4]
    q1 = [4, 3, 2, 1]
    q2 = multiply(q0, q1)
    print(q2)
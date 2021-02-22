"""
Convertion of angles to quaternions and back again. 


"""
import matplotlib as plt
import numpy as np
import pandas as pd
import math
from time import sleep
from sys import stdout




"""
Convert euler angle to quaternion
"""
def euler_to_quaternion(r):
    (yaw, pitch, roll) = (math.radians(r[0]), math.radians(r[1]), math.radians(r[2]))
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qw, qx, qy, qz]


"""
Convert quaternion to euler angles
"""
def quaternion_to_euler(q):
    (x, y, z, w) = (q[0], q[1], q[2], q[3])
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
    return [yaw, pitch, roll]


"""
Converting several values 
returns eulerangles
"""
def convert_multiple(file):
    data = pd.read_csv(file)  
    quaternions = np.empty([1, 4]) 

    for i in range(len(data)):
        new_data = data.iloc[i, 1:4].values
        x = euler_to_quaternion(new_data)
        y = quaternion_to_euler(x)

        #print("\n\nQuaternion: ", x, "\n\n", "Euler: ", y, "\n\n")
        #stdout.write("\r%d" % x[0]) #y[1]
        #stdout.flush
        #sleep(0.1)
    
        #quaternions = np.vstack([quaternions, x])
        quaternions = np.append(quaternions,[x], axis=0)
    return quaternions
    

def convert_single(file, row):
     data = pd.read_csv(file)
     data = data.iloc[row, 1:4].values
     x = np.array([euler_to_quaternion(data)])
     print(x.shape)
     return x



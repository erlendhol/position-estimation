"""
Convertion of angles to quaternions and back again. 


"""
import matplotlib as plt
import numpy as np
import pandas as pd
import math
from time import sleep
from sys import stdout





def euler_to_quaternion(r):
    """
    Convert euler angle to quaternion\n
    :param r: (N,3) array\n
    Return:\n
        Quaternion (N,4) array\n
    """

    (yaw, pitch, roll) = (math.radians(r[0]), math.radians(r[1]), math.radians(r[2]))
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qw, qx, qy, qz]



def quaternion_to_euler(q):
    """
    Convert quaternion to euler angles\n
    :param q: (N,4) array:\n 
    Return:\n
        Euler Angle, (N,3) Array
    """
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
    


def convert_multiple(file):
    """
    Converting (N,4) array to (N,3) by removing the timestamp \n 
    Converts Euler Angles to Quoternions\n

    :param file: (N,4) array - IMU data\n
    
    Return:\n
        quaternion, (N,4) array
    """
    data = pd.read_csv(file)  
    quaternions = np.empty([1, 4]) 

    for i in range(len(data)):
        new_data = data.iloc[i, 1:4].values
        x = euler_to_quaternion(new_data)
        y = quaternion_to_euler(x)        
        quaternions = np.append(quaternions,[x], axis=0)
    return quaternions



def convert_single(file, row):
    """
    Convert singel (1,4) array row from dataset\n
    
    :param file: (N,4) array \n
    :param int row: which row to convert\n

    Return:
        (1,4) Quaternion   
    """ 
    data = pd.read_csv(file)
    data = data.iloc[row, 1:4].values
    x = np.array([euler_to_quaternion(data)])
    print(x.shape)
    return x


def convert_several_quaternion_to_euler(file):
    data = pd.read_csv(file)
    euler = np.empty([1,3])
    for i in range(len(data)):
        new_data = data.iloc[i,:].values
        x = quaternion_to_euler(new_data)
        euler = np.append(euler, [x], axis=0)       
    return euler
    
    




def transform_without_timestamp(file):
    """
    For Dataset without timestap\n
    Converts (N,3) array to (N,4) Quaternion\n

    :param file: (N,3) array 

    Return:\n
        Quaternion (N,4)
    """
    data = pd.read_csv(file)  
    quaternions = np.empty([1, 4]) 

    for i in range(len(data)):
        new_data = data.iloc[i, 0:3].values
        x = euler_to_quaternion(new_data)
        y = quaternion_to_euler(x)
        quaternions = np.append(quaternions,[x], axis=0)
    return quaternions


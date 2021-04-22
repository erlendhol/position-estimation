import numpy as np
import math

g_const = 9.80665

from threading import Thread
import serial
import time

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

class Logger(object):
    def __init__(self, parent=None):
        # Initial Arduino reading
        self.init_msg = arduino.readline()
        self.init_msg_vec = self.init_msg.decode('utf-8').split(',')
        while len(self.init_msg_vec) != 13:
            self.init_msg = arduino.readline()
            self.init_msg_vec = self.init_msg.decode('utf-8').split(',')

        self.msg = arduino.readline() #Read everything in the input buffer
        self.msgVec = self.msg.decode('utf-8').split(',')
        self.mag_meanX = 0 #-17.149
        self.mag_meanY = 0 #5.885
        self.mag_meanZ = 0 #-1.308
        self.time_stamp = float(self.msgVec[0]) / 1000
        self.accelX_current = float(self.msgVec[1])
        self.accelY_current = float(self.msgVec[2])
        self.accelZ_current = float(self.msgVec[3])
        self.magX_current = float(self.msgVec[4]) - self.mag_meanX
        self.magY_current = float(self.msgVec[5]) - self.mag_meanY
        self.magZ_current = float(self.msgVec[6]) - self.mag_meanZ
        self.gyroX_current = float(self.msgVec[7])
        self.gyroY_current = float(self.msgVec[8])
        self.gyroZ_current = float(self.msgVec[9])
        self.eulerX_current = -float(self.msgVec[10])
        self.eulerY_current = -float(self.msgVec[11])
        self.eulerZ_current = float(self.msgVec[12])

        self.roll_meas = 0
        self.pitch_meas = 0
        self.yaw_calc = 0


def get_pitch(a, degrees=True):
    norm = np.linalg.norm(a)
    phi = 0
    if norm > 0:
        a_norm = -np.divide(a.T, norm)
        phi = math.atan2(a_norm[0], math.sqrt(a_norm[1]**2 + a_norm[2]**2))
    if degrees:
        return math.degrees(phi)
    else:
        return phi

def get_roll(a, degrees=True):
    norm = np.linalg.norm(a)
    theta = 0
    if norm > 0:
        a_norm = np.divide(a.T, norm)
        theta = math.atan2(a_norm[1], math.sqrt(a_norm[0]**2 + a_norm[2]**2))
    if degrees:
        return math.degrees(theta)
    else:
        return theta

def get_yaw(pitch, roll, m, degrees=True):
    phi = pitch
    theta = roll
    c_phi = math.cos(math.radians(phi))
    s_phi = math.sin(math.radians(phi))
    c_theta = math.cos(math.radians(theta))
    s_theta = math.sin(math.radians(theta))
    mag = m.T
    # norm = np.linalg.norm(m)
    norm = math.sqrt((mag[0]**2) + (mag[1]**2) + (mag[2]**2))
    if norm > 0:
        m_norm = np.divide(mag, norm)

        XH = m_norm[0]*math.cos(math.radians(pitch)) + m_norm[1]*math.sin(math.radians(pitch))*math.sin(math.radians(roll)) + m_norm[2]*math.sin(math.radians(pitch))*math.cos(math.radians(roll))

        YH = m_norm[1]*math.cos(math.radians(roll)) + m_norm[2]*math.sin(math.radians(roll))

        psi = math.atan2(-YH, XH)

        #psi = math.atan2((-m_norm[1]*c_theta + m_norm[2]*s_theta), (m_norm[0]*c_phi + m_norm[1]*s_phi*s_theta + m_norm[2]*s_phi*c_theta))
        #
        #
        # R = np.array([[c_theta, s_theta*s_phi, s_theta*c_phi],
        #             [0, c_phi, -s_phi],
        #             [-s_theta, c_theta*s_phi, c_theta*c_phi]])
        # b = R @ m_norm
        #
        # psi = math.atan2(-b[1], b[0])
        if degrees:
            return math.degrees(psi)
        else:
            return psi
    else:
        return 0



if __name__ == "__main__":
    import sys
    arduino = serial.Serial(port='/dev/cu.usbserial-DN041PFR', baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=0)
    logger = Logger(sys.argv)
    #acc = np.array([[-2.82, -0.58, 9.28]])
    #mag = np.array([[-40.06, 25.38, -80.06]])
    #phi = get_pitch(acc)
    #print('Phi: ', phi)
    #theta = get_roll(acc)
    #print('Theta: ', theta)
    #psi = get_yaw(phi, theta, mag)
    #print('Psi: ', psi+180)

    while True:
        logger.msg = arduino.readline() #Read everything in the input buffer
        logger.msgVec = logger.msg.decode('utf-8').split(',')
        if len(logger.msgVec) == 13 and logger.msgVec[0] and logger.msgVec[1] and logger.msgVec[2] and logger.msgVec[3] and logger.msgVec[4] and logger.msgVec[5] and logger.msgVec[6] and logger.msgVec[7] and logger.msgVec[8] and logger.msgVec[9] and logger.msgVec[10] and logger.msgVec[11] and logger.msgVec[12]:
            logger.time_stamp = float(logger.msgVec[0]) / 1000
            logger.accelX_current = float(logger.msgVec[1])
            logger.accelY_current = float(logger.msgVec[2])
            logger.accelZ_current = float(logger.msgVec[3])
            logger.magX_current = float(logger.msgVec[4]) - logger.mag_meanX
            logger.magY_current = float(logger.msgVec[5]) - logger.mag_meanY
            logger.magZ_current = float(logger.msgVec[6]) - logger.mag_meanZ
            logger.gyroX_current = float(logger.msgVec[7])
            logger.gyroY_current = float(logger.msgVec[8])
            logger.gyroZ_current = float(logger.msgVec[9])
            logger.eulerX_current = -float(logger.msgVec[10])
            logger.eulerY_current = -float(logger.msgVec[11])
            logger.eulerZ_current = float(logger.msgVec[12])
        logger.roll_meas = get_roll(np.array([[logger.accelX_current, logger.accelY_current, logger.accelZ_current]]), degrees=True) * 2
        logger.pitch_meas = get_pitch(np.array([[logger.accelX_current, logger.accelY_current, logger.accelZ_current]]), degrees=True) * 2
        logger.yaw_calc = get_yaw((logger.pitch_meas), (logger.roll_meas), np.array([[logger.magX_current, logger.magY_current, logger.magZ_current]]), degrees=True)

        print(logger.yaw_calc)
        time.sleep(0.05)

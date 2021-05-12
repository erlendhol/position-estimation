#This is an application made for a bachelor project.
#The application reads IMU data from an Arduino board and filters the data
#using Kalman filters and Madgwick filter to estimate orientation.
#The application also includes a graphical user interface to plot and visualize
#the data, using the Qt framework.
#
#
#Written by Erlend Holseker and Arvin Khodabandeh.

from PyQt5 import QtCore, QtGui, QtWidgets
from threading import Thread
import time
import cv2
import pyqtgraph as pg
import numpy as np
from random import randint
import math
import serial
import pandas as pd

from kalman import KalmanFilter
from madgwick import MadgwickFilter
import orientation_conversion
import quaternion
import homogeneous_transform as ht
import frame
import mainwindow

class ControlMainWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super(ControlMainWindow, self).__init__(parent)
        self.ui = mainwindow.Ui_MainWindow()
        self.ui.setupUi(self)

        self.numberOfPlottingValues = 1

        #self.listenThread = Thread(target=self.readSerialInputs, args=())
        #self.listenThread.daemon = True

        self.measurements = pd.read_csv('datasets/pitch-roll-test.csv')

        # Connecting the menu buttons
        self.ui.RawAccBtn.clicked.connect(lambda : self.ui.WidgetPages.setCurrentIndex(0))
        self.ui.RawGyroBtn.clicked.connect(lambda : self.ui.WidgetPages.setCurrentIndex(1))
        self.ui.RawMagBtn.clicked.connect(lambda : self.ui.WidgetPages.setCurrentIndex(2))
        self.ui.EstValBtn.clicked.connect(lambda : self.ui.WidgetPages.setCurrentIndex(3))

        #self.ui.StartBtn.clicked.connect(lambda : self.listenThread.start())

        self.ui.SensorFrameXBtn.setChecked(True)
        self.ui.ShipFrameXBtn.setChecked(True)
        self.ui.SensorFrameYBtn.setChecked(True)
        self.ui.ShipFrameYBtn.setChecked(True)
        self.ui.SensorFrameZBtn.setChecked(True)
        self.ui.ShipFrameZBtn.setChecked(True)

        #self.capture = cv2.VideoCapture('videoplayback.mp4')

        # Create variables for holding the last 200 plotting values
        self.time_stamps = list(range(self.numberOfPlottingValues))
        ## ESTIMATE GRAPHS ##
        ## Roll ##
        self.est_roll_list = [0 for _ in range(self.numberOfPlottingValues)]
        self.meas_roll_list = [0 for _ in range(self.numberOfPlottingValues)]
        self.ship_pitch_list = [0 for _ in range(self.numberOfPlottingValues)]
        self.meas_roll_line = self.ui.EstValGraphX.plot(self.time_stamps, self.meas_roll_list, name="Pre-Calculated Roll angle", pen=pg.mkPen(color='b'))
        self.est_roll_line = self.ui.EstValGraphX.plot(self.time_stamps, self.est_roll_list, name="Kalman Roll angle", pen=pg.mkPen(color='r'))
        self.ship_pitch_line = self.ui.EstValGraphX.plot(self.time_stamps, self.ship_pitch_list, name="Ship Pitch angle", pen=pg.mkPen(color='g'))
        ## Pitch ##
        self.est_pitch_list = [0 for _ in range(self.numberOfPlottingValues)]
        self.meas_pitch_list = [0 for _ in range(self.numberOfPlottingValues)]
        self.ship_roll_list = [0 for _ in range(self.numberOfPlottingValues)]
        self.meas_pitch_line = self.ui.EstValGraphY.plot(self.time_stamps, self.meas_pitch_list, name="Pre-Calculated Pitch angle", pen=pg.mkPen(color='b'))
        self.est_pitch_line = self.ui.EstValGraphY.plot(self.time_stamps, self.est_pitch_list, name="Kalman Pitch angle", pen=pg.mkPen(color='r'))
        self.ship_roll_line = self.ui.EstValGraphY.plot(self.time_stamps, self.ship_roll_list, name="Ship Roll angle", pen=pg.mkPen(color='g'))
        ## Yaw ##
        self.est_yaw_list = [0 for _ in range(self.numberOfPlottingValues)]
        self.meas_yaw_list = [0 for _ in range(self.numberOfPlottingValues)]
        self.ship_yaw_list = [0 for _ in range(self.numberOfPlottingValues)]
        self.meas_yaw_line = self.ui.EstValGraphZ.plot(self.time_stamps, self.meas_yaw_list, name="Kalman Yaw angle", pen=pg.mkPen(color='r'))
        #self.est_yaw_line = self.ui.EstValGraphZ.plot(self.time_stamps, self.est_yaw_list, name="Kalman Yaw angle", pen=pg.mkPen(color='r'))
        self.ship_yaw_line = self.ui.EstValGraphZ.plot(self.time_stamps, self.ship_yaw_list, name="Ship Yaw angle", pen=pg.mkPen(color='g'))

        ## ACCELEROMETER GRAPHS ##
        ## X ##
        self.accX_list = [0 for _ in range(self.numberOfPlottingValues)]
        self.accX_line = self.ui.AccValGraphX.plot(self.time_stamps, self.accX_list, name="Acc X", pen=pg.mkPen(color='b'))
        ## Y ##
        self.accY_list = [0 for _ in range(self.numberOfPlottingValues)]
        self.accY_line = self.ui.AccValGraphY.plot(self.time_stamps, self.accY_list, name="Acc Y", pen=pg.mkPen(color='b'))
        ## Z ##
        self.accZ_list = [0 for _ in range(self.numberOfPlottingValues)]
        self.accZ_line = self.ui.AccValGraphZ.plot(self.time_stamps, self.accZ_list, name="Acc Z", pen=pg.mkPen(color='b'))

        ## GYROSCOPE GRAPHS ##
        ## X ##
        self.gyroX_list = [0 for _ in range(self.numberOfPlottingValues)]
        self.gyroX_line = self.ui.GyroValGraphX.plot(self.time_stamps, self.gyroX_list, name="Gyro X", pen=pg.mkPen(color='b'))
        ## Y ##
        self.gyroY_list = [0 for _ in range(self.numberOfPlottingValues)]
        self.gyroY_line = self.ui.GyroValGraphY.plot(self.time_stamps, self.gyroY_list, name="Gyro Y", pen=pg.mkPen(color='b'))
        ## Z ##
        self.gyroZ_list = [0 for _ in range(self.numberOfPlottingValues)]
        self.gyroZ_line = self.ui.GyroValGraphZ.plot(self.time_stamps, self.gyroZ_list, name="Gyro Z", pen=pg.mkPen(color='b'))

        ## MAGNETOMETER GRAPHS ##
        ## X ##
        self.magX_list = [0 for _ in range(self.numberOfPlottingValues)]
        self.magX_line = self.ui.MagValGraphX.plot(self.time_stamps, self.magX_list, name="Mag X", pen=pg.mkPen(color='b'))
        ## Y ##
        self.magY_list = [0 for _ in range(self.numberOfPlottingValues)]
        self.magY_line = self.ui.MagValGraphY.plot(self.time_stamps, self.magY_list, name="Mag Y", pen=pg.mkPen(color='b'))
        ## Z ##
        self.magZ_list = [0 for _ in range(self.numberOfPlottingValues)]
        self.magZ_line = self.ui.MagValGraphZ.plot(self.time_stamps, self.magZ_list, name="Mag Z", pen=pg.mkPen(color='b'))

        ## KALMAN VARIABLES ##
        #Accelerometer x variance:  0.0003049141476257811
        #Accelerometer y variance:  2.146392908505662e-05
        #Accelerometer z variance:  0.0004189126491039419
        #Gyroscope x variance:  0.007443206389568397
        #Gyroscope y variance:  0.009982315838002695
        #Gyroscope z variance:  0.00717653405276467

        self.A = np.array([[0, 0],
                           [0, 0]], dtype='float')
        self.H = np.array([[1, 0],
                           [0, 1]], dtype='float')
        self.delta_t = 0.05
        ## Kalman Variables Roll ##
        self.x_roll = np.array([[0],                 #Rotation angle (position)
                                [0]], dtype='float') #Angular velocity
        self.Q_roll = 0
        self.s2_roll = 0.1 ** 2
        self.est_state_roll = np.zeros(2)
        self.est_angularVel_roll = 0
        self.est_angle_roll = 0
        self.meas_roll = 0
        accelerometerX_variance = 0.0003049141476257811
        roll_variance = 0.01839461414087105
        gyroscopeX_variance = 0.007443206389568397
        self.R_roll = np.array([[roll_variance, 0],
                                [0, gyroscopeX_variance]])
        ## Kalman Variables Pitch ##
        self.x_pitch = np.array([[0],                 #Rotation angle (position)
                                 [0]], dtype='float') #Angular velocity
        self.Q_pitch = 0
        self.s2_pitch = 0.1 ** 2
        self.est_state_pitch = np.zeros(2)
        self.est_angularVel_pitch = 0
        self.est_angle_pitch = 0
        self.meas_pitch = 0
        accelerometerY_variance = 2.146392908505662e-05
        pitch_variance = 0.010618805958553145
        gyroscopeY_variance = 0.009982315838002695
        self.R_pitch = np.array([[pitch_variance, 0],
                                 [0, gyroscopeY_variance]])
        ## Kalman Variables Yaw ##
        self.x_yaw = np.array([[0],                     #Rotation angle (position)
                               [0]], dtype='float')     #Angular velocity
        self.Q_yaw = 0
        self.s2_yaw = 0.1 ** 2
        self.est_state_yaw = np.zeros(2)
        self.est_angularVel_yaw = 0
        self.est_angle_yaw = 0
        self.meas_yaw = 0
        #Madgwick yaw variance:  0.07849187709471803
        mad_yaw_variance = 0.07849187709471803
        gyroscopeZ_variance = 0.00717653405276467
        self.R_yaw = np.array([[mad_yaw_variance, 0],
                               [0, gyroscopeY_variance]])
        self.yaw_calc = 0
        self.mad_yaw = 0
        self.mad_pitch = 0
        self.mad_roll = 0
        #Madgwick Filter Gain:  0.09114646853830789
        self.beta = 0.041
        #self.madgwick = MadgwickFilter(beta=self.beta)
        self.accel_meas_norm = np.array([[0], [0], [0]])
        self.q = np.array([0, 0, 0, 0]).transpose()
        # Create Kalman filters
        self.kalman_filter_roll= KalmanFilter(self.A, self.H, self.Q_roll, self.R_roll, self.x_roll)
        self.kalman_filter_pitch = KalmanFilter(self.A, self.H, self.Q_pitch, self.R_pitch, self.x_pitch)
        self.kalman_filter_yaw = KalmanFilter(self.A, self.H, self.Q_yaw, self.R_yaw, self.x_yaw)

        ## Variables to hold data from Arduino ##
        self.time_stamp = 0
        self.last_time_stamp = 0
        self.accelX_current = 0
        self.accelY_current = 0
        self.accelZ_current = 0
        self.magX_current = 0
        self.magY_current = 0
        self.magZ_current = 0
        self.gyroX_current = 0
        self.gyroY_current = 0
        self.gyroZ_current = 0

        self.shipRoll = 0
        self.shipPitch = 0
        self.shipYaw = 0

        self.delta_t_list = self.measurements.iloc[:, 0].values
        self.accX_measurements = self.measurements.iloc[:, 1].values # Accel X
        self.accY_measurements = self.measurements.iloc[:, 2].values # Accel Y
        self.accZ_measurements = self.measurements.iloc[:, 3].values # Accel Z
        self.magX_measurements = self.measurements.iloc[:, 4].values # Magnetometer X
        self.magY_measurements = self.measurements.iloc[:, 5].values # Magnetometer Y
        self.magZ_measurements = self.measurements.iloc[:, 6].values # Magnetometer Z
        self.gyroX_measurements = self.measurements.iloc[:, 7].values # Gyro X
        self.gyroY_measurements = self.measurements.iloc[:, 8].values # Gyro Y
        self.gyroZ_measurements = self.measurements.iloc[:, 9].values # Gyro Z
        self.i = 0

        self.encoder_yaw = 70
        self.encoder_pitch = 10

        self.readSerialInputs()

    def readSerialInputs(self):
        # Initial Arduino reading
        self.madgwick = MadgwickFilter(beta=self.beta, initial_slew=self.encoder_yaw)

        self.worldFrame = frame.Frame()
        self.sensorFrame = frame.Frame(parentFrame=self.worldFrame)
        self.shipFrame = frame.Frame(orientation=np.array([0, self.ui.EncoderPitch.value(), self.ui.EncoderYaw.value()]), parentFrame=self.sensorFrame)
        self.sensorFrame.set_child_frame(self.shipFrame)

        for self.i in range(len(self.measurements)):
                #self.time_stamp = float(self.time_stamp_measurements[self.i])/1000
                self.delta_t = float(self.delta_t_list[self.i])
                self.accelX_current = float(self.accX_measurements[self.i])
                self.accelY_current = float(self.accY_measurements[self.i])
                self.accelZ_current = float(self.accZ_measurements[self.i])
                self.magX_current = float(self.magX_measurements[self.i])
                self.magY_current = float(self.magY_measurements[self.i])
                self.magZ_current = float(self.magZ_measurements[self.i])
                #Mean gyro x:  -0.05156950354609929
                #Mean gyro y:  0.04870000000000001
                #Mean gyro z:  0.06984326241134751
                self.gyroX_current = float(self.gyroX_measurements[self.i]) + 0.05156950354609929
                self.gyroY_current = float(self.gyroY_measurements[self.i]) - 0.04870000000000001
                self.gyroZ_current = float(self.gyroZ_measurements[self.i]) - 0.06984326241134751
                self.calculateEstimates()

        self.updateGUI()

    def calculateEstimates(self):
        self.meas_roll = orientation_conversion.get_roll(np.array([[self.accelX_current, self.accelY_current, self.accelZ_current]]), degrees=True)
        self.meas_pitch = orientation_conversion.get_pitch(np.array([[self.accelX_current, self.accelY_current, self.accelZ_current]]), degrees=True)

        # Kalman Estimate Roll
        y_roll = np.array([[self.meas_roll],
                           [self.gyroX_current]])
        self.kalman_filter_roll.predict()
        self.kalman_filter_roll.update(y_roll)
        (x_r, P_r) = self.kalman_filter_roll.get_state()
        self.est_state_roll = x_r.transpose()
        self.est_angularVel_roll = self.est_state_roll.transpose()[1]
        self.est_angle_roll = self.est_state_roll.transpose()[0]

        # Kalman Estimate Pitch
        y_pitch = np.array([[self.meas_pitch],
                            [self.gyroY_current]])
        self.kalman_filter_pitch.predict()
        self.kalman_filter_pitch.update(y_pitch)
        (x_p, P_p) = self.kalman_filter_pitch.get_state()
        self.est_state_pitch = x_p.transpose()
        self.est_angularVel_pitch = self.est_state_pitch.transpose()[1]
        self.est_angle_pitch = self.est_state_pitch.transpose()[0]

        ## MADGWICK ##
        accel_meas = np.array([self.accelX_current, self.accelY_current, self.accelZ_current]).T
        gyro_meas = np.array([(self.gyroX_current*math.pi/180), (self.gyroY_current*math.pi/180), (self.gyroZ_current*math.pi/180)]).T
        mag_meas = np.array([0, self.magX_current, self.magY_current, self.magZ_current]).T

        self.q = self.madgwick.get_estimated_orientation(gyro=gyro_meas, acc=accel_meas, mag=mag_meas, delta_t=self.delta_t)
        ###############################

        ## GET EULER ANGLES FROM MADGWICK QUATERNION ##
        self.mad_roll, self.mad_pitch, self.mad_yaw = quaternion.quaternion_to_euler(self.q, as_degrees=True)
        #self.meas_yaw = orientation_conversion.get_yaw(self.meas_pitch, self.meas_roll, mag_meas)
        #print(self.mad_yaw)

        # Kalman Estimate Yaw
        y_yaw = np.array([[self.mad_yaw],
                          [self.gyroZ_current]])
        self.kalman_filter_yaw.predict()
        self.kalman_filter_yaw.update(y_yaw)
        (x_y, P_y) = self.kalman_filter_yaw.get_state()
        self.est_state_yaw = x_y.transpose()
        self.est_angularVel_yaw = self.est_state_yaw.transpose()[1]
        self.est_angle_yaw = self.est_state_yaw.transpose()[0]

        # Update parameters
        base_sigma = np.array([[self.delta_t ** 3 / 3, self.delta_t ** 2 / 2],
                               [self.delta_t ** 2 / 2, self.delta_t]])

        self.A = np.array([[1, self.delta_t],
                           [0, 1]], dtype='float')
        self.Q_roll = self.s2_roll * base_sigma
        self.Q_pitch = self.s2_pitch * base_sigma
        self.Q_yaw = self.s2_yaw * base_sigma

        self.kalman_filter_roll.updateParameters(A=self.A, Q=self.Q_roll)
        self.kalman_filter_pitch.updateParameters(A=self.A, Q=self.Q_pitch)
        self.kalman_filter_yaw.updateParameters(A=self.A, Q=self.Q_yaw)

        self.sensorFrame.update_orientation([float(self.est_angle_roll), -float(self.est_angle_pitch), float(self.est_angle_yaw)])
        self.shipFrame.update_orientation([self.encoder_pitch, 0, -self.encoder_yaw])
        self.shipFrame.set_parent_frame(self.sensorFrame)
        self.sensorFrame.set_child_frame(self.shipFrame)

        shipAngles = ht.rot_matrix_to_euler(self.shipFrame.get_grandparent_representation())

        self.shipRoll = shipAngles[0]
        self.shipPitch = shipAngles[1]
        self.shipYaw = shipAngles[2]


        self.time_stamps.append(self.time_stamps[-1] + 1)

        self.est_roll_list.append(float(self.est_angle_roll))
        self.meas_roll_list.append(float(self.meas_roll))
        self.ship_pitch_list.append(self.shipPitch)

        self.est_pitch_list.append(float(self.est_angle_pitch))
        self.meas_pitch_list.append(float(self.meas_pitch))
        self.ship_roll_list.append(self.shipRoll)

        self.meas_yaw_list.append(float(self.est_angle_yaw))
        #self.meas_yaw_list.append(10)
        #self.est_yaw_list.append(float(self.est_angle_yaw))
        self.ship_yaw_list.append(self.shipYaw)

        self.accX_list.append(float(self.accelX_current))
        self.accY_list.append(float(self.accelY_current))
        self.accZ_list.append(float(self.accelZ_current))

        self.gyroX_list.append(float(self.gyroX_current))
        self.gyroY_list.append(float(self.gyroY_current))
        self.gyroZ_list.append(float(self.gyroZ_current))

        self.magX_list.append(float(self.magX_current))
        self.magY_list.append(float(self.magY_current))
        self.magZ_list.append(float(self.magZ_current))


    def updateGUI(self):
        self.est_roll_line.setData(self.time_stamps, self.est_roll_list)
        self.meas_roll_line.setData(self.time_stamps, self.meas_roll_list)
        self.ship_pitch_line.setData(self.time_stamps, self.ship_pitch_list)
        self.est_pitch_line.setData(self.time_stamps, self.est_pitch_list)
        self.meas_pitch_line.setData(self.time_stamps, self.meas_pitch_list)
        self.ship_roll_line.setData(self.time_stamps, self.ship_roll_list)
        self.meas_yaw_line.setData(self.time_stamps, self.meas_yaw_list)
        #self.est_yaw_line.setData(self.time_stamps, self.est_yaw_list)
        self.ship_yaw_line.setData(self.time_stamps, self.ship_yaw_list)

        self.accX_line.setData(self.time_stamps, self.accX_list)
        self.accY_line.setData(self.time_stamps, self.accY_list)
        self.accZ_line.setData(self.time_stamps, self.accZ_list)
        self.gyroX_line.setData(self.time_stamps, self.gyroX_list)
        self.gyroY_line.setData(self.time_stamps, self.gyroY_list)
        self.gyroZ_line.setData(self.time_stamps, self.gyroZ_list)
        self.magX_line.setData(self.time_stamps, self.magX_list)
        self.magY_line.setData(self.time_stamps, self.magY_list)
        self.magZ_line.setData(self.time_stamps, self.magZ_list)

        # if self.ui.SensorFrameXBtn.isChecked():
        #     self.est_roll_line.show()
        # else:
        #     self.est_roll_line.hide()
        # if self.ui.ShipFrameXBtn.isChecked():
        #     self.ship_pitch_line.show()
        # else:
        #     self.ship_pitch_line.hide()
        #
        # if self.ui.SensorFrameYBtn.isChecked():
        #     self.est_pitch_line.show()
        # else:
        #     self.est_pitch_line.hide()
        # if self.ui.ShipFrameYBtn.isChecked():
        #     self.ship_roll_line.show()
        # else:
        #     self.ship_roll_line.hide()
        #
        # if self.ui.SensorFrameZBtn.isChecked():
        #     self.meas_yaw_line.show()
        # else:
        #     self.meas_yaw_line.hide()
        # if self.ui.ShipFrameZBtn.isChecked():
        #     self.ship_yaw_line.show()
        # else:
        #     self.ship_yaw_line.hide()

    # def showVideo(self):
    #     while True:
    #         self.ret, self.frame = self.capture.read()
    #         if self.frame is not None:
    #             self.rgbImage = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
    #             self.convertToQtFormat = QtGui.QImage(self.rgbImage.data, self.rgbImage.shape[1], self.rgbImage.shape[0],
    #                                                 QtGui.QImage.Format_RGB888)
    #             self.convertToQtFormat = QtGui.QPixmap.fromImage(self.convertToQtFormat)
    #             self.pixmap = QtGui.QPixmap(self.convertToQtFormat)
    #             self.resizeImage = self.pixmap.scaled(480,480, QtCore.Qt.KeepAspectRatio)
    #             QtWidgets.QApplication.processEvents()
    #             self.ui.CamFeedLabel.setPixmap(self.resizeImage)
    #             time.sleep(0.025)
    #         else:
    #             self.ui.CamFeedLabel.setText('Never Gonna Give You Up <3')
    #             self.capture.release()


if __name__ == '__main__':
    #arduino = serial.Serial(port='/dev/cu.usbserial-DN041PFR', baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=0)
    import sys
    app = QtWidgets.QApplication(sys.argv)
    mySW = ControlMainWindow()
    mySW.show()

    sys.exit(app.exec_())

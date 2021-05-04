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

        self.numberOfPlottingValues = 300

        self.listenThread = Thread(target=self.readSerialInputs, args=())
        self.listenThread.daemon = True

        # Connecting the menu buttons
        self.ui.RawAccBtn.clicked.connect(lambda : self.ui.WidgetPages.setCurrentIndex(0))
        self.ui.RawGyroBtn.clicked.connect(lambda : self.ui.WidgetPages.setCurrentIndex(1))
        self.ui.RawMagBtn.clicked.connect(lambda : self.ui.WidgetPages.setCurrentIndex(2))
        self.ui.EstValBtn.clicked.connect(lambda : self.ui.WidgetPages.setCurrentIndex(3))

        self.ui.StartBtn.clicked.connect(lambda : self.listenThread.start())

        self.ui.SensorFrameXBtn.setChecked(True)
        self.ui.ShipFrameXBtn.setChecked(True)
        self.ui.SensorFrameYBtn.setChecked(True)
        self.ui.ShipFrameYBtn.setChecked(True)
        self.ui.SensorFrameZBtn.setChecked(True)
        self.ui.ShipFrameZBtn.setChecked(True)

        self.capture = cv2.VideoCapture('videoplayback.mp4')

        # Create variables for holding the last 200 plotting values
        self.time_stamps = list(range(self.numberOfPlottingValues))
        ## ESTIMATE GRAPHS ##
        ## Roll ##
        self.est_roll_list = [0 for _ in range(self.numberOfPlottingValues)]
        self.meas_roll_list = [0 for _ in range(self.numberOfPlottingValues)]
        self.ship_pitch_list = [0 for _ in range(self.numberOfPlottingValues)]
        self.est_roll_line = self.ui.EstValGraphX.plot(self.time_stamps, self.est_roll_list, name="Kalman Roll angle", pen=pg.mkPen(color='r'))
        self.ship_pitch_line = self.ui.EstValGraphX.plot(self.time_stamps, self.ship_pitch_list, name="Ship Pitch angle", pen=pg.mkPen(color='g'))
        ## Pitch ##
        self.est_pitch_list = [0 for _ in range(self.numberOfPlottingValues)]
        self.meas_pitch_list = [0 for _ in range(self.numberOfPlottingValues)]
        self.ship_roll_list = [0 for _ in range(self.numberOfPlottingValues)]
        self.est_pitch_line = self.ui.EstValGraphY.plot(self.time_stamps, self.est_pitch_list, name="Kalman Pitch angle", pen=pg.mkPen(color='r'))
        self.ship_roll_line = self.ui.EstValGraphY.plot(self.time_stamps, self.ship_roll_list, name="Ship Roll angle", pen=pg.mkPen(color='g'))
        ## Yaw ##
        self.est_yaw_list = [0 for _ in range(self.numberOfPlottingValues)]
        self.meas_yaw_list = [0 for _ in range(self.numberOfPlottingValues)]
        self.ship_yaw_list = [0 for _ in range(self.numberOfPlottingValues)]
        self.meas_yaw_line = self.ui.EstValGraphZ.plot(self.time_stamps, self.meas_yaw_list, name="Madgwick Yaw angle", pen=pg.mkPen(color='b'))
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
        self.A = np.array([[0, 0],
                           [0, 0]], dtype='float')
        self.H = np.array([[1, 0],
                           [0, 1]], dtype='float')
        self.delta_t = 0.05
        ## Kalman Variables Roll ##
        self.x_roll = np.array([[0],                 #Rotation angle (position)
                                [0]], dtype='float') #Angular velocity
        self.Q_roll = 0
        self.s2_roll = 0.2 ** 2
        self.est_state_roll = np.zeros(2)
        self.est_angularVel_roll = 0
        self.est_angle_roll = 0
        self.meas_roll = 0
        accelerometerX_variance = 0.00031141365254884923
        roll_variance = 0.01839461414087105
        gyroscopeX_variance = 0.007197688446362619
        self.R_roll = np.array([[roll_variance, 0],
                                [0, gyroscopeX_variance]])
        ## Kalman Variables Pitch ##
        self.x_pitch = np.array([[0],                 #Rotation angle (position)
                                 [0]], dtype='float') #Angular velocity
        self.Q_pitch = 0
        self.s2_pitch = 0.2 ** 2
        self.est_state_pitch = np.zeros(2)
        self.est_angularVel_pitch = 0
        self.est_angle_pitch = 0
        self.meas_pitch = 0
        accelerometerY_variance = 0.0005460492817487228
        pitch_variance = 0.010618805958553145
        gyroscopeY_variance = 0.010014742586806145
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
        mad_yaw_variance = 0.2296308583589739
        gyroscopeZ_variance = 0.009201665020393227
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

        # Connect a timer to the update GUI function to update the plots at a given interval
        self.timer = QtCore.QTimer()
        self.timer.setInterval(5)
        self.timer.timeout.connect(self.updateGUI)
        self.timer.start()

        # Starting a thread to capture camera video
        self.videoThread = Thread(target=self.showVideo, args=())
        self.videoThread.daemon = True
        self.videoThread.start()

    def readSerialInputs(self):
        # Initial Arduino reading
        self.madgwick = MadgwickFilter(beta=self.beta, initial_slew=self.ui.EncoderYaw.value())

        self.worldFrame = frame.Frame()
        self.sensorFrame = frame.Frame(parentFrame=self.worldFrame)
        self.shipFrame = frame.Frame(orientation=np.array([0, self.ui.EncoderPitch.value(), self.ui.EncoderYaw.value()]), parentFrame=self.sensorFrame)
        self.sensorFrame.set_child_frame(self.shipFrame)

        init_msg = arduino.readline()
        init_msg_vec = init_msg.decode('utf-8').split(',')
        while len(init_msg_vec) != 11:    # Wait for serial communication to stabilize
            init_msg = arduino.readline()
            init_msg_vec = init_msg.decode('utf-8').split(',')

        while True:
            msg = arduino.readline() #Read everything in the input buffer
            msgVec = msg.decode('utf-8').split(',')
            if len(msgVec) == 11 and msgVec[0] and msgVec[1] and msgVec[2] and msgVec[3] and msgVec[4] and msgVec[5] and msgVec[6] and msgVec[7] and msgVec[8] and msgVec[9] and msgVec[10]:
                #self.delta_t = float(msgVec[0])
                self.accelX_current = float(msgVec[1])
                self.accelY_current = float(msgVec[2])
                self.accelZ_current = float(msgVec[3])
                self.magX_current = float(msgVec[4])
                self.magY_current = float(msgVec[5])
                self.magZ_current = float(msgVec[6])
                self.gyroX_current = (float(msgVec[7]) + 0.042910521140609635)
                self.gyroY_current = (float(msgVec[8]) - 0.08874139626352016)
                self.gyroZ_current = (float(msgVec[9]) - 0.08517207472959686)

            arduino.reset_input_buffer()
            self.calculateEstimates()
            time.sleep(0.05)

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

        self.q = self.madgwick.get_estimated_orientation(gyro=gyro_meas, acc=accel_meas, mag=None, delta_t=self.delta_t)
        ###############################

        ## GET EULER ANGLES FROM MADGWICK QUATERNION ##
        self.mad_roll, self.mad_pitch, self.mad_yaw = quaternion.quaternion_to_euler(self.q, as_degrees=True)

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

        self.A = np.array([[1, -self.delta_t],
                           [0, 1]], dtype='float')
        self.Q_roll = self.s2_roll * base_sigma
        self.Q_pitch = self.s2_pitch * base_sigma
        self.Q_yaw = self.s2_yaw * base_sigma

        self.kalman_filter_roll.updateParameters(A=self.A, Q=self.Q_roll)
        self.kalman_filter_pitch.updateParameters(A=self.A, Q=self.Q_pitch)
        self.kalman_filter_yaw.updateParameters(A=self.A, Q=self.Q_yaw)

        self.sensorFrame.update_orientation([float(self.est_angle_roll), -float(self.est_angle_pitch), float(self.mad_yaw)])
        self.shipFrame.update_orientation([self.ui.EncoderPitch.value(), 0, -self.ui.EncoderYaw.value()])
        self.shipFrame.set_parent_frame(self.sensorFrame)
        self.sensorFrame.set_child_frame(self.shipFrame)

        shipAngles = ht.rot_matrix_to_euler(self.shipFrame.get_grandparent_representation())

        self.shipRoll = shipAngles[0]
        self.shipPitch = shipAngles[1]
        self.shipYaw = shipAngles[2]

    def updateGUI(self):
        self.time_stamps = self.time_stamps[1:] #Remove first element
        self.time_stamps.append(self.time_stamps[-1] + 1)

        self.est_roll_list = self.est_roll_list[1:]
        self.est_roll_list.append(float(self.est_angle_roll))

        self.ship_pitch_list = self.ship_pitch_list[1:]
        self.ship_pitch_list.append(self.shipPitch)

        self.est_pitch_list = self.est_pitch_list[1:]
        self.est_pitch_list.append(float(self.est_angle_pitch))

        self.ship_roll_list = self.ship_roll_list[1:]
        self.ship_roll_list.append(self.shipRoll)

        self.meas_yaw_list = self.meas_yaw_list[1:]
        self.meas_yaw_list.append(float(self.mad_yaw))

        self.ship_yaw_list = self.ship_yaw_list[1:]
        self.ship_yaw_list.append(self.shipYaw)


        self.accX_list = self.accX_list[1:]
        self.accX_list.append(float(self.accelX_current))
        self.accY_list = self.accY_list[1:]
        self.accY_list.append(float(self.accelY_current))
        self.accZ_list = self.accZ_list[1:]
        self.accZ_list.append(float(self.accelZ_current))

        self.gyroX_list = self.gyroX_list[1:]
        self.gyroX_list.append(float(self.gyroX_current))
        self.gyroY_list = self.gyroY_list[1:]
        self.gyroY_list.append(float(self.gyroY_current))
        self.gyroZ_list = self.gyroZ_list[1:]
        self.gyroZ_list.append(float(self.gyroZ_current))

        self.magX_list = self.magX_list[1:]
        self.magX_list.append(float(self.magX_current))
        self.magY_list = self.magY_list[1:]
        self.magY_list.append(float(self.magY_current))
        self.magZ_list = self.magZ_list[1:]
        self.magZ_list.append(float(self.magZ_current))

        self.ui.dialX.setValue(int(self.est_angle_roll))
        self.ui.dialY.setValue(int(self.est_angle_pitch))
        self.ui.dialZ.setValue(int(self.mad_yaw))

        roll_text = ('Estimated Roll: ' + "{:.2f}".format(float(self.est_angle_roll)) + '°' + '\n' +
                     'Ship Pitch: ' + "{:.2f}".format(float(self.shipPitch)) + '°')

        pitch_text = ('Estimated Pitch: ' + "{:.2f}".format(float(self.est_angle_pitch)) + '°' + '\n' +
                     'Ship Roll: ' + "{:.2f}".format(float(self.shipRoll)) + '°')

        yaw_text = ('Estimated Yaw: ' "{:.2f}".format(float(self.mad_yaw)) + '°' + '\n' +
                   'Ship Yaw: ' + "{:.2f}".format(float(self.shipYaw)) + '°')

        self.ui.label_7.setText(roll_text)
        self.ui.label_6.setText(pitch_text)
        self.ui.label_5.setText(yaw_text)

        self.est_roll_line.setData(self.time_stamps, self.est_roll_list)
        self.ship_pitch_line.setData(self.time_stamps, self.ship_pitch_list)
        self.est_pitch_line.setData(self.time_stamps, self.est_pitch_list)
        self.ship_roll_line.setData(self.time_stamps, self.ship_roll_list)
        self.meas_yaw_line.setData(self.time_stamps, self.meas_yaw_list)
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

        if self.ui.SensorFrameXBtn.isChecked():
            self.est_roll_line.show()
        else:
            self.est_roll_line.hide()
        if self.ui.ShipFrameXBtn.isChecked():
            self.ship_pitch_line.show()
        else:
            self.ship_pitch_line.hide()

        if self.ui.SensorFrameYBtn.isChecked():
            self.est_pitch_line.show()
        else:
            self.est_pitch_line.hide()
        if self.ui.ShipFrameYBtn.isChecked():
            self.ship_roll_line.show()
        else:
            self.ship_roll_line.hide()

        if self.ui.SensorFrameZBtn.isChecked():
            self.meas_yaw_line.show()
        else:
            self.meas_yaw_line.hide()
        if self.ui.ShipFrameZBtn.isChecked():
            self.ship_yaw_line.show()
        else:
            self.ship_yaw_line.hide()

    def showVideo(self):
        while True:
            self.ret, self.frame = self.capture.read()
            if self.frame is not None:
                self.rgbImage = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
                self.convertToQtFormat = QtGui.QImage(self.rgbImage.data, self.rgbImage.shape[1], self.rgbImage.shape[0],
                                                    QtGui.QImage.Format_RGB888)
                self.convertToQtFormat = QtGui.QPixmap.fromImage(self.convertToQtFormat)
                self.pixmap = QtGui.QPixmap(self.convertToQtFormat)
                self.resizeImage = self.pixmap.scaled(480,480, QtCore.Qt.KeepAspectRatio)
                QtWidgets.QApplication.processEvents()
                self.ui.CamFeedLabel.setPixmap(self.resizeImage)
                time.sleep(0.025)
            else:
                self.ui.CamFeedLabel.setText('Never Gonna Give You Up <3')
                self.capture.release()


if __name__ == '__main__':
    arduino = serial.Serial(port='/dev/cu.usbserial-DN041PFR', baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=0)
    import sys
    app = QtWidgets.QApplication(sys.argv)
    mySW = ControlMainWindow()
    mySW.show()

    sys.exit(app.exec_())

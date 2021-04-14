from PyQt5 import QtCore, QtGui, QtWidgets
from pyqtgraph import PlotWidget
from threading import Thread
import time
import pyqtgraph as pg
from random import randint

import numpy as np
import serial
from kalman_2 import KalmanFilter
import orientation_conversion

arduino = serial.Serial(port='/dev/cu.usbserial-DN041PFR', baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=0)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(921, 797)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.graphWidgetX = PlotWidget(self.centralwidget)
        self.graphWidgetX.setGeometry(QtCore.QRect(10, 10, 611, 231))
        self.graphWidgetX.setObjectName("graphWidgetX")
        self.graphWidgetY = PlotWidget(self.centralwidget)
        self.graphWidgetY.setGeometry(QtCore.QRect(10, 250, 611, 241))
        self.graphWidgetY.setObjectName("graphWidgetY")
        self.graphWidgetZ = PlotWidget(self.centralwidget)
        self.graphWidgetZ.setGeometry(QtCore.QRect(10, 500, 611, 241))
        self.graphWidgetZ.setObjectName("graphWidgetZ")
        self.dialX = QtWidgets.QDial(self.centralwidget)
        self.dialX.setGeometry(QtCore.QRect(630, 13, 281, 221))
        self.dialX.setMinimum(-180)
        self.dialX.setMaximum(180)
        self.dialX.setWrapping(True)
        self.dialX.setNotchesVisible(True)
        self.dialX.setObjectName("dialX")
        self.dialY = QtWidgets.QDial(self.centralwidget)
        self.dialY.setGeometry(QtCore.QRect(630, 250, 271, 231))
        self.dialY.setMinimum(-180)
        self.dialY.setMaximum(180)
        self.dialY.setWrapping(True)
        self.dialY.setNotchesVisible(True)
        self.dialY.setObjectName("dialY")
        self.dialZ = QtWidgets.QDial(self.centralwidget)
        self.dialZ.setGeometry(QtCore.QRect(630, 500, 271, 231))
        self.dialZ.setMaximum(360)
        self.dialZ.setWrapping(True)
        self.dialZ.setNotchesVisible(True)
        self.dialZ.setObjectName("dialZ")
        self.Roll_label = QtWidgets.QLabel(self.centralwidget)
        self.Roll_label.setGeometry(QtCore.QRect(740, 100, 71, 31))
        self.Roll_label.setObjectName("Roll_label")
        self.Pitch_label = QtWidgets.QLabel(self.centralwidget)
        self.Pitch_label.setGeometry(QtCore.QRect(730, 350, 71, 31))
        self.Pitch_label.setObjectName("Pitch_label")
        self.Yaw_label = QtWidgets.QLabel(self.centralwidget)
        self.Yaw_label.setGeometry(QtCore.QRect(730, 600, 71, 31))
        self.Yaw_label.setObjectName("Yaw_label")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 921, 24))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.Roll_label.setText(_translate("MainWindow", "Roll:"))
        self.Pitch_label.setText(_translate("MainWindow", "Pitch:"))
        self.Yaw_label.setText(_translate("MainWindow", "Yaw:"))

class ControlMainWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super(ControlMainWindow, self).__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.ui.graphWidgetX.setBackground('w')
        # Set Title
        self.ui.graphWidgetX.setTitle("Angle X (Roll) in degrees", color="b", size="15pt")
        # Add Axis labels
        styles = {"color": "#f00", "font-size": "15px"}
        self.ui.graphWidgetX.setLabel("left", "Roll Angle", **styles)
        self.ui.graphWidgetX.setLabel("bottom", "Time steps", **styles)
        # Add legend
        self.ui.graphWidgetX.addLegend()
        # Add grid
        self.ui.graphWidgetX.showGrid(x=True, y=True)
        # Set range
        #self.ui.graphWidgetX.setYRange(0, 6.5, padding=0)

        self.ui.graphWidgetY.setBackground('w')
        # Set Title
        self.ui.graphWidgetY.setTitle("Angle Y (Pitch) in degrees", color="b", size="15pt")
        # Add Axis labels
        styles = {"color": "#f00", "font-size": "15px"}
        self.ui.graphWidgetY.setLabel("left", "Pitch Angle", **styles)
        self.ui.graphWidgetY.setLabel("bottom", "Time steps", **styles)
        # Add legend
        self.ui.graphWidgetY.addLegend()
        # Add grid
        self.ui.graphWidgetY.showGrid(x=True, y=True)
        # Set range
        #self.ui.graphWidgetY.setYRange(0, 6.5, padding=0)

        self.ui.graphWidgetZ.setBackground('w')
        # Set Title
        self.ui.graphWidgetZ.setTitle("Angle Z (Yaw) in degrees", color="b", size="15pt")
        # Add Axis labels
        styles = {"color": "#f00", "font-size": "15px"}
        self.ui.graphWidgetZ.setLabel("left", "Yaw Angle", **styles)
        self.ui.graphWidgetZ.setLabel("bottom", "Time steps", **styles)
        # Add legend
        self.ui.graphWidgetZ.addLegend()
        # Add grid
        self.ui.graphWidgetZ.showGrid(x=True, y=True)
        # Set range
        #self.ui.graphWidgetZ.setYRange(0, 6.5, padding=0)

        # Initial Arduino reading
        self.init_msg = arduino.readline()
        self.init_msg_vec = self.init_msg.decode('utf-8').split(',')
        while len(self.init_msg_vec) != 13:
            self.init_msg = arduino.readline()
            self.init_msg_vec = self.init_msg.decode('utf-8').split(',')
        # Initial IMU raw data
        self.time_stamp = int(self.init_msg_vec[0]) / 1000
        self.last_time_stamp = self.time_stamp
        self.accelX_current = float(self.init_msg_vec[1])
        self.accelY_current = float(self.init_msg_vec[2])
        self.accelZ_current = float(self.init_msg_vec[3])
        self.magX_current = float(self.init_msg_vec[4])
        self.magY_current = float(self.init_msg_vec[5])
        self.magZ_current = float(self.init_msg_vec[6])
        self.gyroX_current = float(self.init_msg_vec[7])
        self.gyroY_current = float(self.init_msg_vec[8])
        self.gyroZ_current = float(self.init_msg_vec[9])

        self.time_stamps = list(range(200))

        self.est_roll = [randint(0,0) for _ in range(200)]
        self.meas_roll = [randint(1,1) for _ in range(200)]
        self.eulerX_list = [randint(0,0) for _ in range(200)]
        self.est_roll_line = self.ui.graphWidgetX.plot(self.time_stamps, self.est_roll, name="Estimated Roll angle", pen=pg.mkPen(color='r'))
        self.meas_roll_line = self.ui.graphWidgetX.plot(self.time_stamps, self.meas_roll, name="Measured Roll angle", pen=pg.mkPen(color='b'))
        #self.eulerX_line = self.ui.graphWidgetX.plot(self.time_stamps, self.eulerX_list, name="Euler Roll angle", pen=pg.mkPen(color='g'))
        #self.est_rollVel = [randint(0,0) for _ in range(200)]
        #self.meas_rollVel = [randint(1,1) for _ in range(200)]
        #elf.est_rollVel_line = self.ui.graphWidgetY.plot(self.time_stamps, self.est_rollVel, name="Estimated Roll Velocity", pen=pg.mkPen(color='r'))
        #self.meas_rollVel_line = self.ui.graphWidgetY.plot(self.time_stamps, self.meas_rollVel, name="Measured Roll Velocity", pen=pg.mkPen(color='b'))
        self.est_pitch = [randint(0,0) for _ in range(200)]
        self.meas_pitch = [randint(1,1) for _ in range(200)]
        self.eulerY_list = [randint(0,0) for _ in range(200)]
        self.est_pitch_line = self.ui.graphWidgetY.plot(self.time_stamps, self.est_pitch, name="Estimated Pitch angle", pen=pg.mkPen(color='r'))
        self.meas_pitch_line = self.ui.graphWidgetY.plot(self.time_stamps, self.meas_pitch, name="Measured Pitch angle", pen=pg.mkPen(color='b'))
        #self.eulerY_line = self.ui.graphWidgetY.plot(self.time_stamps, self.eulerY_list, name="Euler Pitch angle", pen=pg.mkPen(color='g'))

        self.est_yaw = [randint(0,0) for _ in range(200)]
        self.meas_yaw = [randint(1,1) for _ in range(200)]
        self.eulerZ_list = [randint(0,0) for _ in range(200)]
        self.est_yaw_line = self.ui.graphWidgetZ.plot(self.time_stamps, self.est_yaw, name="Estimated Yaw angle", pen=pg.mkPen(color='r'))
        #self.eulerZ_line = self.ui.graphWidgetZ.plot(self.time_stamps, self.eulerZ_list, name="Euler Yaw angle", pen=pg.mkPen(color='g'))
        self.meas_yaw_line = self.ui.graphWidgetZ.plot(self.time_stamps, self.meas_yaw, name="Calculated Yaw angle", pen=pg.mkPen(color='b'))

        self.timer = QtCore.QTimer()
        self.timer.setInterval(5)
        self.timer.timeout.connect(self.plotTheGraphs)
        self.timer.start()

        ## KALMAN ##
        self.A = np.array([[0, 0],
                           [0, 0]], dtype='float')
        # Kalman Roll
        self.H_roll = np.array([[1, 0],
                           [0, 1]], dtype='float')
        self.x_roll = np.array([[0],                 #Rotation angle (position)
                           [0]], dtype='float')      #Angular velocity
        self.Q_roll = 0
        self.s2_roll = 0.01 ** 2
        self.est_state_roll = np.zeros(2)
        self.est_angularVel_Roll = 0
        self.est_angle_Roll = 0
        self.roll_meas = 0
        accelerometerX_variance = 0.00011154289877952518
        gyroscopeX_variance = 0.0203081596772978
        self.R_roll = np.array([[accelerometerX_variance, 0],
                      [0, gyroscopeX_variance]])

        # Kalman Pitch
        self.H_pitch = np.array([[1, 0],
                           [0, 1]], dtype='float')
        self.x_pitch = np.array([[0],                 #Rotation angle (position)
                           [0]], dtype='float')       #Angular velocity
        self.Q_pitch = 0
        self.s2_pitch = 0.01 ** 2
        self.est_state_pitch = np.zeros(2)
        self.est_angularVel_Pitch = 0
        self.est_angle_Pitch = 0
        self.pitch_meas = 0
        accelerometerY_variance = 0.00025083176623752354
        gyroscopeY_variance = 0.02365472804860408
        self.R_pitch = np.array([[accelerometerY_variance, 0],
                        [0, gyroscopeY_variance]])

        # Kalman Yaw
        self.H_yaw = np.array([[1, 0],
                           [0, 1]], dtype='float')
        self.x_yaw = np.array([[0],                     #Rotation angle (position)
                           [0]], dtype='float')         #Angular velocity
        self.Q_yaw = 0
        self.s2_yaw = 0.1 ** 2
        self.est_state_yaw = np.zeros(2)
        self.est_angularVel_Yaw = 0
        self.est_angle_Yaw = 0
        self.yaw_meas = 0
        yaw_calc_variance = 0.05
        gyroscopeZ_variance = 0.04776449576032258
        self.R_yaw = np.array([[yaw_calc_variance, 0],
                        [0, gyroscopeY_variance]])

        # Yaw Calculation
        self.yaw_est = 0
        self.mag_meanX = -17.149
        self.mag_meanY = 5.885
        self.mag_meanZ = -1.308
        self.current_mean_mag_x = 0
        self.magnetoX = np.zeros(4)
        self.current_mean_mag_y = 0
        self.magnetoY = np.zeros(4)
        self.current_mean_mag_z = 0
        self.magnetoZ = np.zeros(4)
        #self.yaw_estimates = np.zeros(4)
        #self.current_mean_yaw_est = 0

        self.delta_t = 0

        self.kalman_filter_roll= KalmanFilter(self.A, self.H_roll, self.Q_roll, self.R_roll, self.x_roll)
        self.kalman_filter_pitch = KalmanFilter(self.A, self.H_pitch, self.Q_pitch, self.R_pitch, self.x_pitch)
        self.kalman_filter_yaw = KalmanFilter(self.A, self.H_yaw, self.Q_yaw, self.R_yaw, self.x_yaw)

        ## MAGNETOMETER KALMAN ##
        self.A_mag = np.array([[1]])
        self.x_magX = np.array([[0]])
        self.x_magY = np.array([[0]])
        self.x_magZ = np.array([[0]])
        self.H_mag = np.array([[1]])
        self.Q_magX = 0
        self.Q_magY = 0
        self.Q_magZ = 0
        self.s2_magX = 0.001 ** 2
        self.s2_magY = 0.001 ** 2
        self.s2_magZ = 0.001 ** 2
        self.est_state_magX = 0
        self.est_state_magY = 0
        self.est_state_magZ = 0
        self.R_magX = 0.4017811230975991
        self.R_magY = 2.900135890311062
        self.R_magZ = 0.3936153217491893

        self.kalman_filter_magX = KalmanFilter(self.A_mag, self.H_mag, self.Q_magX, self.R_magX, self.x_magX)
        self.kalman_filter_magY = KalmanFilter(self.A_mag, self.H_mag, self.Q_magY, self.R_magY, self.x_magY)
        self.kalman_filter_magZ = KalmanFilter(self.A_mag, self.H_mag, self.Q_magZ, self.R_magZ, self.x_magZ)

        # Euler angles from IMU internal calculation
        self.eulerX_current = 0
        self.eulerY_current = 0
        self.eulerZ_current = 0

        # Magnetometer thread
        self.magnetThread = Thread(target=self.magnetometerInputs, args=())
        self.magnetThread.daemon = True
        self.startMagnetThread()

        # Starting a thread to listen for inputs from Arduino
        self.listenThread = Thread(target=self.readSerialInputs, args=())
        self.listenThread.daemon = True
        self.startListenThread()

    def startMagnetThread(self):
        self.magnetThread.start()
        return self

    def startListenThread(self):
        self.listenThread.start()
        return self

    def readSerialInputs(self):
        while True:
            msg = arduino.readline() #Read everything in the input buffer
            msgVec = msg.decode('utf-8').split(',')
            if len(msgVec) == 13 and msgVec[0] and msgVec[1] and msgVec[2] and msgVec[3] and msgVec[4] and msgVec[5] and msgVec[6] and msgVec[7] and msgVec[8] and msgVec[9] and msgVec[10] and msgVec[11] and msgVec[12]:
                self.time_stamp = float(msgVec[0]) / 1000
                self.accelX_current = float(msgVec[1])
                self.accelY_current = float(msgVec[2])
                self.accelZ_current = float(msgVec[3])
                self.magX_current = float(msgVec[4]) - self.mag_meanX
                self.magY_current = float(msgVec[5]) - self.mag_meanY
                self.magZ_current = float(msgVec[6]) - self.mag_meanZ
                self.gyroX_current = float(msgVec[7])
                self.gyroY_current = float(msgVec[8])
                self.gyroZ_current = float(msgVec[9])
                self.eulerX_current = -float(msgVec[10])
                self.eulerY_current = -float(msgVec[11])
                self.eulerZ_current = float(msgVec[12])
                self.delta_t = (self.time_stamp - self.last_time_stamp)
                #print(self.delta_t)

                # FIR filter for magnetometer
                # self.magnetoX = self.magnetoX[1:]
                # self.magnetoX = np.append(self.magnetoX, float(self.magX_current))
                # self.current_mean_mag_x = np.mean(self.magnetoX)
                #
                # self.magnetoY = self.magnetoY[1:]
                # self.magnetoY = np.append(self.magnetoY, float(self.magY_current))
                # self.current_mean_mag_y = np.mean(self.magnetoY)
                #
                # self.magnetoZ = self.magnetoZ[1:]
                # self.magnetoZ = np.append(self.magnetoZ, float(self.magZ_current))
                # self.current_mean_mag_z = np.mean(self.magnetoZ)

                #print("Measured Mag Y: ", self.magY_current)

                #print("Magneto mean X: ", self.current_mean_mag_x)
                #print("Magneto mean Y: ", self.current_mean_mag_y)
                #print("Magneto mean Z: ", self.current_mean_mag_z)

            self.last_time_stamp = self.time_stamp
            arduino.reset_input_buffer()
            time.sleep(0.01)

    def magnetometerInputs(self):
        while True:
            self.roll_meas = orientation_conversion.get_roll(np.array([[self.accelX_current, self.accelY_current, self.accelZ_current]]), degrees=True) * 2
            self.pitch_meas = orientation_conversion.get_pitch(np.array([[self.accelX_current, self.accelY_current, self.accelZ_current]]), degrees=True) * 2

            # Kalman Estimate Roll
            y_roll = np.array([[self.roll_meas],
                          [self.gyroX_current]])
            self.kalman_filter_roll.predict()
            self.kalman_filter_roll.update(y_roll)
            (x_r, P_r) = self.kalman_filter_roll.get_state()
            self.est_state_roll = x_r.transpose()
            self.est_angularVel_Roll = self.est_state_roll.transpose()[1]
            self.est_angle_Roll = self.est_state_roll.transpose()[0]

            # Kalman Estimate Pitch
            y_pitch = np.array([[self.pitch_meas],
                          [self.gyroY_current]])
            self.kalman_filter_pitch.predict()
            self.kalman_filter_pitch.update(y_pitch)
            (x_p, P_p) = self.kalman_filter_pitch.get_state()
            self.est_state_pitch = x_p.transpose()
            self.est_angularVel_Pitch = self.est_state_pitch.transpose()[1]
            self.est_angle_Pitch = self.est_state_pitch.transpose()[0]

            # Tilt compensation for yaw calculation
            self.yaw_est = (orientation_conversion.get_yaw((self.est_angle_Pitch), (self.est_angle_Roll), np.array([[self.est_state_magX, self.est_state_magY, self.est_state_magZ]]), degrees=True) + 180)

            # Kalman Estimate Yaw
            y_yaw = np.array([[self.yaw_est],
                          [self.gyroZ_current]])
            self.kalman_filter_yaw.predict()
            self.kalman_filter_yaw.update(y_yaw)
            (x_y, P_y) = self.kalman_filter_yaw.get_state()
            self.est_state_yaw = x_y.transpose()
            self.est_angularVel_Yaw = self.est_state_yaw.transpose()[1]
            self.est_angle_Yaw = self.est_state_yaw.transpose()[0]

            # Kalman Estimate Magneto X
            y_magX = self.magX_current
            self.kalman_filter_magX.predict()
            self.kalman_filter_magX.update(y_magX)
            (x_magnetX, P_magX) = self.kalman_filter_magX.get_state()
            self.est_state_magX = x_magnetX[0][0]
            #print("Kalman filtered Mag X: ", self.est_state_magX)
            # Kalman Estimate Magneto Y
            y_magY = self.magY_current
            self.kalman_filter_magY.predict()
            self.kalman_filter_magY.update(y_magY)
            (x_magnetY, P_magY) = self.kalman_filter_magY.get_state()
            self.est_state_magY = x_magnetY[0][0]
            #print("Kalman filtered Mag Y: ", self.est_state_magY)
            # Kalman Estimate Magneto Z
            y_magZ = self.magZ_current
            self.kalman_filter_magZ.predict()
            self.kalman_filter_magZ.update(y_magZ)
            (x_magnetZ, P_magZ) = self.kalman_filter_magZ.get_state()
            self.est_state_magZ = x_magnetZ[0][0]
            #print("Kalman filtered Mag Z: ", self.est_state_magZ)

            base_sigma = np.array([[self.delta_t ** 3 / 3, self.delta_t ** 2 / 2],
                                   [self.delta_t ** 2 / 2, self.delta_t]])

            self.A = np.array([[1, -self.delta_t],
                               [0, 1]], dtype='float')
            self.Q_roll = self.s2_roll * base_sigma
            self.Q_pitch = self.s2_pitch * base_sigma
            self.Q_yaw = self.s2_yaw * base_sigma
            self.Q_magX = self.s2_magX * self.delta_t
            self.Q_magY = self.s2_magY * self.delta_t
            self.Q_magZ = self.s2_magZ * self.delta_t

            self.kalman_filter_roll.updateParameters(A=self.A, H=self.H_roll, Q=self.Q_roll, R=self.R_roll)
            self.kalman_filter_pitch.updateParameters(A=self.A, H=self.H_pitch, Q=self.Q_pitch, R=self.R_pitch)
            self.kalman_filter_yaw.updateParameters(A=self.A, H=self.H_yaw, Q=self.Q_yaw, R=self.R_yaw)
            self.kalman_filter_magX.updateParameters(A=self.A_mag, H=self.H_mag, Q=self.Q_magX, R=self.R_magX)
            self.kalman_filter_magY.updateParameters(A=self.A_mag, H=self.H_mag, Q=self.Q_magY, R=self.R_magY)
            self.kalman_filter_magZ.updateParameters(A=self.A_mag, H=self.H_mag, Q=self.Q_magZ, R=self.R_magZ)

            time.sleep(0.1)

    def plotTheGraphs(self):
        self.time_stamps = self.time_stamps[1:] #Remove first element
        self.time_stamps.append(self.time_stamps[-1] + 1)

        self.est_roll = self.est_roll[1:]
        self.est_roll.append(float(self.est_angle_Roll))

        self.meas_roll = self.meas_roll[1:]
        self.meas_roll.append(float(self.roll_meas))

        self.est_pitch = self.est_pitch[1:]
        self.est_pitch.append(float(self.est_angle_Pitch))

        self.meas_pitch = self.meas_pitch[1:]
        self.meas_pitch.append(float(self.pitch_meas))

        self.meas_yaw = self.meas_yaw[1:]
        self.meas_yaw.append(float(self.yaw_est))
        #self.meas_yaw.append(float(self.current_mean_yaw_est))

        self.est_yaw = self.est_yaw[1:]
        self.est_yaw.append(float(self.est_angle_Yaw))

        #self.eulerX_list = self.eulerX_list[1:]
        #self.eulerX_list.append(float(self.eulerX_current))
        #self.eulerY_list = self.eulerY_list[1:]
        #self.eulerY_list.append(float(self.eulerY_current))
        #self.eulerZ_list = self.eulerZ_list[1:]
        #self.eulerZ_list.append(float(self.eulerZ_current))

        self.ui.dialX.setValue(int(self.est_angle_Roll))
        self.ui.dialY.setValue(int(self.est_angle_Pitch))
        self.ui.dialZ.setValue(int(self.est_angle_Yaw))
        #self.ui.dialZ.setValue(int(self.current_mean_yaw_est))

        roll_text = 'Roll: ' + str(int(self.est_angle_Roll)) + '°'
        pitch_text = 'Pitch: ' + str(int(self.est_angle_Pitch)) + '°'
        yaw_text = 'Yaw: ' + str(int(self.est_angle_Yaw)) + '°'
        #yaw_text = 'Yaw: ' + str(int(self.current_mean_yaw_est)) + '°'
        self.ui.Roll_label.setText(roll_text)
        self.ui.Pitch_label.setText(pitch_text)
        self.ui.Yaw_label.setText(yaw_text)

        #self.meas_yaw = self.meas_yaw[1:]
        #self.meas_yaw.append(float(self.gyroZ_current))

        self.est_roll_line.setData(self.time_stamps, self.est_roll)
        self.meas_roll_line.setData(self.time_stamps, self.meas_roll)
        self.est_pitch_line.setData(self.time_stamps, self.est_pitch)
        self.meas_pitch_line.setData(self.time_stamps, self.meas_pitch)
        self.est_yaw_line.setData(self.time_stamps, self.est_yaw)
        self.meas_yaw_line.setData(self.time_stamps, self.meas_yaw)

        #self.eulerX_line.setData(self.time_stamps, self.eulerX_list)
        #self.eulerY_line.setData(self.time_stamps, self.eulerY_list)
        #self.eulerZ_line.setData(self.time_stamps, self.eulerZ_list)

if __name__ == '__main__':
    import sys
    app = QtWidgets.QApplication(sys.argv)
    mySW = ControlMainWindow()
    mySW.show()

    sys.exit(app.exec_())

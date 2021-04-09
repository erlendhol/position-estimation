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
        self.graphWidgetX.setGeometry(QtCore.QRect(10, 10, 901, 231))
        self.graphWidgetX.setObjectName("graphWidgetX")
        self.graphWidgetY = PlotWidget(self.centralwidget)
        self.graphWidgetY.setGeometry(QtCore.QRect(10, 250, 901, 241))
        self.graphWidgetY.setObjectName("graphWidgetY")
        self.graphWidgetZ = PlotWidget(self.centralwidget)
        self.graphWidgetZ.setGeometry(QtCore.QRect(10, 500, 901, 241))
        self.graphWidgetZ.setObjectName("graphWidgetZ")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 921, 22))
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

class ControlMainWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super(ControlMainWindow, self).__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.ui.graphWidgetX.setBackground('w')
        # Set Title
        self.ui.graphWidgetX.setTitle("Angle X (Roll)", color="b", size="15pt")
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
        self.ui.graphWidgetY.setTitle("Angle Y (Pitch)", color="b", size="15pt")
        # Add Axis labels
        styles = {"color": "#f00", "font-size": "15px"}
        self.ui.graphWidgetY.setLabel("left", "Roll Angular Velocity", **styles)
        self.ui.graphWidgetY.setLabel("bottom", "Time steps", **styles)
        # Add legend
        self.ui.graphWidgetY.addLegend()
        # Add grid
        self.ui.graphWidgetY.showGrid(x=True, y=True)
        # Set range
        #self.ui.graphWidgetY.setYRange(0, 6.5, padding=0)

        self.ui.graphWidgetZ.setBackground('w')
        # Set Title
        self.ui.graphWidgetZ.setTitle("Angle Z (Yaw)", color="b", size="15pt")
        # Add Axis labels
        styles = {"color": "#f00", "font-size": "15px"}
        self.ui.graphWidgetZ.setLabel("left", "Angle (rad)", **styles)
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
        while len(self.init_msg_vec) != 10:
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
        #self.accelX = [randint(0,0) for _ in range(200)]
        #self.accelY = [randint(0,0) for _ in range(200)]
        #self.accelZ = [randint(0,0) for _ in range(200)]
        #self.eulerX = [randint(-4,4) for _ in range(200)]
        #self.eulerY = [randint(-4,4) for _ in range(200)]
        #self.eulerZ = [randint(-4,4) for _ in range(200)]
        #self.accelX_line = self.ui.graphWidgetX.plot(self.time_stamps, self.accelX, name="Accelerometer X", pen=pg.mkPen(color='r'))
        #self.accelY_line = self.ui.graphWidgetY.plot(self.time_stamps, self.accelY, name="Accelerometer Y", pen=pg.mkPen(color='r'))
        #self.accelZ_line = self.ui.graphWidgetZ.plot(self.time_stamps, self.accelZ, name="Accelerometer Z", pen=pg.mkPen(color='r'))
        #self.eulerX_line = self.ui.graphWidgetX.plot(self.time_stamps, self.eulerX, name="Euler X", pen=pg.mkPen(color='r'))
        #self.eulerY_line = self.ui.graphWidgetY.plot(self.time_stamps, self.eulerY, name="Euler Y", pen=pg.mkPen(color='r'))
        #self.eulerZ_line = self.ui.graphWidgetZ.plot(self.time_stamps, self.eulerZ, name="Euler Z", pen=pg.mkPen(color='r'))
        self.est_roll = [randint(0,0) for _ in range(200)]
        self.meas_roll = [randint(1,1) for _ in range(200)]
        self.est_roll_line = self.ui.graphWidgetX.plot(self.time_stamps, self.est_roll, name="Estimated Roll angle", pen=pg.mkPen(color='r'))
        self.meas_roll_line = self.ui.graphWidgetX.plot(self.time_stamps, self.meas_roll, name="Measured Roll angle", pen=pg.mkPen(color='b'))
        self.est_rollVel = [randint(0,0) for _ in range(200)]
        self.meas_rollVel = [randint(1,1) for _ in range(200)]
        self.est_rollVel_line = self.ui.graphWidgetY.plot(self.time_stamps, self.est_rollVel, name="Estimated Roll Velocity", pen=pg.mkPen(color='r'))
        self.meas_rollVel_line = self.ui.graphWidgetY.plot(self.time_stamps, self.meas_rollVel, name="Measured Roll Velocity", pen=pg.mkPen(color='b'))

        self.timer = QtCore.QTimer()
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.plotTheGraphs)
        self.timer.start()

        # Angles calculated by IMU
        #self.eulerX_current = 0
        #self.eulerY_current = 0
        #self.eulerZ_current = 0

        # Kalman Roll
        self.A_roll = np.array([[0, 0],
                           [0, 0]], dtype='float')
        self.H_roll = np.array([[1, 0],
                           [0, 1]], dtype='float')
        self.x_roll = np.array([[0],                 #Rotation angle
                           [0]], dtype='float')      #Angular velocity
        self.Q_roll = 0
        self.s2 = 0.2 ** 2
        self.est_state = np.zeros(2)
        self.est_angularVel_Roll = 0
        self.est_angle_Roll = 0
        self.roll_meas = 0
        self.delta_t = 0

        accelerometerX_variance = 0.00011154289877952518
        gyroscopeX_variance = 0.0203081596772978
        self.R_roll = np.array([[accelerometerX_variance, 0],
                      [0, gyroscopeX_variance]])

        self.kalman_filter_roll= KalmanFilter(self.A_roll, self.H_roll, self.Q_roll, self.R_roll, self.x_roll)

        # Starting a thread to listen for inputs from Arduino
        self.listenThread = Thread(target=self.readSerialInputs, args=())
        self.listenThread.daemon = True
        self.startListenThread()

    def startListenThread(self):
        self.listenThread.start()
        return self

    def readSerialInputs(self):
        while True:
            msg = arduino.readline() #Read everything in the input buffer
            msgVec = msg.decode('utf-8').split(',')
            if len(msgVec) == 10 and msgVec[0] and msgVec[1] and msgVec[2] and msgVec[3] and msgVec[4] and msgVec[5] and msgVec[6] and msgVec[7] and msgVec[8] and msgVec[9]:
                self.time_stamp = int(msgVec[0]) / 1000
                self.accelX_current = float(msgVec[1])
                self.accelY_current = float(msgVec[2])
                self.accelZ_current = float(msgVec[3])
                self.magX_current = float(msgVec[4])
                self.magY_current = float(msgVec[5])
                self.magZ_current = float(msgVec[6])
                self.gyroX_current = float(msgVec[7])
                self.gyroY_current = float(msgVec[8])
                self.gyroZ_current = float(msgVec[9])
                self.delta_t = (self.time_stamp - self.last_time_stamp)
                self.roll_meas = orientation_conversion.get_roll(np.array([[self.accelX_current, self.accelY_current, self.accelZ_current]]), degrees=True)
                y = np.array([[self.roll_meas],
                              [self.gyroX_current]])

                self.kalman_filter_roll.predict()
                self.kalman_filter_roll.update(y)
                (x, P) = self.kalman_filter_roll.get_state()
                self.est_state = x.transpose()


            self.A_roll = np.array([[1, -self.delta_t],
                               [0, 1]], dtype='float')
            base_sigma = np.array([[self.delta_t ** 3 / 3, self.delta_t ** 2 / 2],
                                   [self.delta_t ** 2 / 2, self.delta_t]])
            self.Q_roll = self.s2 * base_sigma

            self.kalman_filter_roll.updateParameters(A=self.A_roll, H=self.H_roll, Q=self.Q_roll, R=self.R_roll)
            self.est_angularVel_Roll = self.est_state.transpose()[1]
            self.est_angle_Roll = self.est_state.transpose()[0]

            self.last_time_stamp = self.time_stamp
            arduino.reset_input_buffer()
            time.sleep(0.1)

    def plotTheGraphs(self):
        self.time_stamps = self.time_stamps[1:] #Remove first element
        self.time_stamps.append(self.time_stamps[-1] + 1)

        self.est_roll = self.est_roll[1:]
        self.est_roll.append(float(self.est_angle_Roll))

        self.est_rollVel = self.est_rollVel[1:]
        self.est_rollVel.append(float(self.est_angularVel_Roll))

        self.meas_roll = self.meas_roll[1:]
        self.meas_roll.append(float(self.roll_meas))

        self.meas_rollVel = self.meas_rollVel[1:]
        self.meas_rollVel.append(float(self.gyroX_current))

        self.est_roll_line.setData(self.time_stamps, self.est_roll)
        self.est_rollVel_line.setData(self.time_stamps, self.est_rollVel)
        self.meas_roll_line.setData(self.time_stamps, self.meas_roll)
        self.meas_rollVel_line.setData(self.time_stamps, self.meas_rollVel)

        #self.accelX = self.accelX[1:] #Remove first element
        #self.accelX.append(float(self.accelX_current))

        #self.accelY = self.accelY[1:] #Remove first element
        #self.accelY.append(float(self.accelY_current))

        #self.accelZ = self.accelZ[1:] #Remove first element
        #self.accelZ.append(float(self.accelZ_current))

        #self.accelX_line.setData(self.time_stamps, self.accelX)
        #self.accelY_line.setData(self.time_stamps, self.accelY)
        #self.accelZ_line.setData(self.time_stamps, self.accelZ)

if __name__ == '__main__':
    import sys
    app = QtWidgets.QApplication(sys.argv)
    mySW = ControlMainWindow()
    mySW.show()

    sys.exit(app.exec_())

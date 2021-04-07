from PyQt5 import QtCore, QtGui, QtWidgets
from threading import Thread
import time
import pyqtgraph as pg
from random import randint

import numpy as np
import serial
from kalman import KalmanFilter

arduino = serial.Serial(port='/dev/cu.usbserial-DN041PFR', baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=0)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(799, 601)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.graphWidget = PlotWidget(self.centralwidget)
        self.graphWidget.setGeometry(QtCore.QRect(10, 9, 781, 531))
        self.graphWidget.setObjectName("graphWidget")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 24))
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
from pyqtgraph import PlotWidget

class ControlMainWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super(ControlMainWindow, self).__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.ui.graphWidget.setBackground('w')
        # Set Title
        self.ui.graphWidget.setTitle("Kalman Filter", color="b", size="30pt")
        # Add Axis labels
        styles = {"color": "#f00", "font-size": "20px"}
        self.ui.graphWidget.setLabel("left", "Distance (cm)", **styles)
        self.ui.graphWidget.setLabel("bottom", "Time", **styles)
        # Add legend
        self.ui.graphWidget.addLegend()
        # Add grid
        self.ui.graphWidget.showGrid(x=True, y=True)
        # Set range
        #self.ui.graphWidget.setYRange(0, 60, padding=0)

        self.velocity = 0
        self.last_velocity = 0
        self.last_time_stamp = 0
        self.time_stamp = 0

        self.time_stamps = list(range(200))
        self.estimated_distances = [randint(0,100) for _ in range(200)]
        self.measured_distances = [randint(0,100) for _ in range(200)]
        self.estimated_line = self.ui.graphWidget.plot(self.time_stamps, self.estimated_distances, name="Estimated", pen=pg.mkPen(color='r'))
        self.measured_line = self.ui.graphWidget.plot(self.time_stamps, self.measured_distances, name="Measured", pen=pg.mkPen(color='b'))

        # Variance in sensors
        self.distance_var = 0.0032539285926189683
        self.accelX_var = 0.0002538491460462614
        self.delta_t = 0 # Time between each measurement

        # Initial state
        self.x = np.array([[20],
                      [0]])

        self.accelX = 0
        self.distance_meas = 0
        self.estimated_distance = 0

        ### Kalman filter for distance sensor ###
        self.A = np.array([[0, 0],
                      [0, 0]], dtype='float')
        self.B = np.array([[0],
                      [0]], dtype='float')
        self.Q = 0
        self.H = np.array([[1, 0]], dtype='float')
        self.H_a = np.array([[0, 1]], dtype='float')
        self.R = self.distance_var
        self.R_a = self.accelX_var
        self.P = np.array([[0, 0],
                      [0, 0]], dtype='float')

        self.s2_pos = 0.2 ** 2

        self.est_state = np.zeros(2)
        #est_cov = np.zeros((len(accelX), 2, 2))
        #velocities = np.zeros(len(accelX))

        self.kalman_filter_distance = KalmanFilter(self.A, self.B, self.H, self.Q, self.R, self.x, self.P, 0)
        self.kalman_filter_accel = KalmanFilter(self.A, self.B, self.H_a, self.Q, self.R_a, self.x, self.P, 0)

        # Starting a thread to listen for inputs from Arduino
        self.listenThread = Thread(target=self.readSerialInputs, args=())
        self.listenThread.daemon = True
        self.startListenThread()

        #self.plotThread = Thread(target=self.plotTheGraph, args=())
        #self.plotThread.daemon = True
        #self.startPlotThread()

        self.timer = QtCore.QTimer()
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.plotTheGraph)
        self.timer.start()

    def startListenThread(self):
        self.listenThread.start()
        return self

    def startPlotThread(self):
        self.plotThread.start()
        return(self)

    def readSerialInputs(self):
        while True:
            msg = arduino.readline() #Read everything in the input buffer
            msgVec = msg.decode('utf-8').split(',')
            if len(msgVec) == 5 and msgVec[0] and msgVec[1] and msgVec[2] and msgVec[3] and msgVec[4]:
                self.time_stamp = int(msgVec[0]) / 1000
                #print('Time Stamp: ', self.time_stamp)
                self.accelX = (float(msgVec[1])) * 100
                self.distance_meas = float(msgVec[4])
                #print('Acceleration X:')
                #print(self.accelX)
                #print('Distance X:')
                #print(self.distance_meas)
                self.delta_t = (self.time_stamp - self.last_time_stamp)
                #print('Delta T:')
                #print(self.delta_t)

                self.velocity = (float(self.accelX) * float(self.delta_t) + float(self.last_velocity))
                self.kalman_filter_accel.predict()
                self.kalman_filter_accel.update(self.velocity)
                (x, P) = self.kalman_filter_accel.get_state()
                print('Estimated velocity: ', x[1])
                if float(self.distance_meas) > 5:
                    y = float(self.distance_meas)
                    self.kalman_filter_distance.predict()
                    self.kalman_filter_distance.update(y)
                    (x, P) = self.kalman_filter_distance.get_state()
                    self.velocity = x[1]
                    #print('Estimated velocity: ', self.velocity)
                self.est_state = x.transpose()
                #print('Estimated state: ', self.est_state)
                #est_cov[i, ...] = P
                self.last_velocity = self.velocity

            self.A = np.array([[1, self.delta_t],
                               [0, 1]], dtype='float')

            self.B_a = np.array([[0.5*(self.delta_t**2)],
                            [self.delta_t]])

            self.base_sigma = np.array([[self.delta_t ** 3 / 3, self.delta_t ** 2 / 2],
                                   [self.delta_t ** 2 / 2, self.delta_t]])
            self.Q = self.s2_pos * self.base_sigma

            self.kalman_filter_distance.updateParameters(A=self.A, B=self.B, H=self.H, Q=self.Q, R=self.R, u=self.accelX)
            self.kalman_filter_accel.updateParameters(A=self.A, B=self.B_a, H=self.H_a, Q=self.Q, R=self.R_a, u=self.accelX)
            #print('Accel X: ', self.accelX)

            self.estimated_distance = self.est_state.transpose()[0]
            #print("Estimated distance: ", self.estimated_distance)

            self.last_time_stamp = self.time_stamp
            arduino.reset_input_buffer()
            time.sleep(0.05)

            #self.plotTheGraph()


    def plotTheGraph(self):
        self.time_stamps = self.time_stamps[1:] #Remove first element
        self.time_stamps.append(self.time_stamps[-1] + 1)

        self.estimated_distances = self.estimated_distances[1:] #Remove first element
        self.estimated_distances.append(float(self.estimated_distance))
        #print('Estimated Distance: ', float(self.estimated_distance))

        self.measured_distances = self.measured_distances[1:] #Remove first element
        self.measured_distances.append(self.distance_meas)
        #print('Measured Distance: ', self.distance_meas)

        self.estimated_line.setData(self.time_stamps, self.estimated_distances)
        self.measured_line.setData(self.time_stamps, self.measured_distances)


if __name__ == '__main__':
    import sys
    app = QtWidgets.QApplication(sys.argv)
    mySW = ControlMainWindow()
    mySW.show()

    sys.exit(app.exec_())

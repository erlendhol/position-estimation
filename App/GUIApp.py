#This is an application made for a bachelor project.
#The application reads IMU data from an Arduino board and filters the data
#using Kalman filters and Madgwick filter to estimate orientation.
#The application also includes a graphical user interface to plot and visualize
#the data, using the Qt framework.
#
#
#Written by Erlend Holseker and Arvin Khodabandeh.

from PyQt5 import QtCore, QtGui, QtWidgets
from pyqtgraph import PlotWidget
from threading import Thread
import time
import cv2
import pyqtgraph as pg
import numpy as np
from random import randint
import serial

from kalman_3 import KalmanFilter
from madgwick import MadgwickFilter
import orientation_conversion
import quaternion
import math
import homogeneous_transform


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1321, 822)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setStyleSheet("background-color: rgb(127, 127, 127);")
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout.setObjectName("gridLayout")
        self.WidgetContents = QtWidgets.QWidget(self.centralwidget)
        self.WidgetContents.setStyleSheet("background-color:lightgrey")
        self.WidgetContents.setObjectName("WidgetContents")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.WidgetContents)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.WidgetPages = QtWidgets.QStackedWidget(self.WidgetContents)
        self.WidgetPages.setObjectName("WidgetPages")
        self.RawAccPage = QtWidgets.QWidget()
        self.RawAccPage.setObjectName("RawAccPage")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.RawAccPage)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.label_2 = QtWidgets.QLabel(self.RawAccPage)
        self.label_2.setMaximumSize(QtCore.QSize(16777215, 50))
        font = QtGui.QFont()
        font.setFamily("Microsoft Sans Serif")
        font.setPointSize(20)
        self.label_2.setFont(font)
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.verticalLayout_3.addWidget(self.label_2)
        self.AccValGraphX = PlotWidget(self.RawAccPage)
        self.AccValGraphX.setObjectName("AccValGraphX")
        self.verticalLayout_3.addWidget(self.AccValGraphX)
        self.AccValGraphY = PlotWidget(self.RawAccPage)
        self.AccValGraphY.setObjectName("AccValGraphY")
        self.verticalLayout_3.addWidget(self.AccValGraphY)
        self.AccValGraphZ = PlotWidget(self.RawAccPage)
        self.AccValGraphZ.setObjectName("AccValGraphZ")
        self.verticalLayout_3.addWidget(self.AccValGraphZ)
        self.WidgetPages.addWidget(self.RawAccPage)
        self.RawGyroPage = QtWidgets.QWidget()
        self.RawGyroPage.setObjectName("RawGyroPage")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.RawGyroPage)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.label_3 = QtWidgets.QLabel(self.RawGyroPage)
        self.label_3.setMaximumSize(QtCore.QSize(16777215, 50))
        font = QtGui.QFont()
        font.setFamily("Microsoft Sans Serif")
        font.setPointSize(20)
        self.label_3.setFont(font)
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setObjectName("label_3")
        self.verticalLayout_4.addWidget(self.label_3)
        self.GyroValGraphX = PlotWidget(self.RawGyroPage)
        self.GyroValGraphX.setObjectName("GyroValGraphX")
        self.verticalLayout_4.addWidget(self.GyroValGraphX)
        self.GyroValGraphY = PlotWidget(self.RawGyroPage)
        self.GyroValGraphY.setObjectName("GyroValGraphY")
        self.verticalLayout_4.addWidget(self.GyroValGraphY)
        self.GyroValGraphZ = PlotWidget(self.RawGyroPage)
        self.GyroValGraphZ.setObjectName("GyroValGraphZ")
        self.verticalLayout_4.addWidget(self.GyroValGraphZ)
        self.WidgetPages.addWidget(self.RawGyroPage)
        self.RawMagPage = QtWidgets.QWidget()
        self.RawMagPage.setObjectName("RawMagPage")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout(self.RawMagPage)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.label_4 = QtWidgets.QLabel(self.RawMagPage)
        self.label_4.setMaximumSize(QtCore.QSize(16777215, 50))
        font = QtGui.QFont()
        font.setFamily("Microsoft Sans Serif")
        font.setPointSize(20)
        self.label_4.setFont(font)
        self.label_4.setAlignment(QtCore.Qt.AlignCenter)
        self.label_4.setObjectName("label_4")
        self.verticalLayout_5.addWidget(self.label_4)
        self.MagValGraphX = PlotWidget(self.RawMagPage)
        self.MagValGraphX.setObjectName("MagValGraphX")
        self.verticalLayout_5.addWidget(self.MagValGraphX)
        self.MagValGraphY = PlotWidget(self.RawMagPage)
        self.MagValGraphY.setObjectName("MagValGraphY")
        self.verticalLayout_5.addWidget(self.MagValGraphY)
        self.MagValGraphZ = PlotWidget(self.RawMagPage)
        self.MagValGraphZ.setObjectName("MagValGraphZ")
        self.verticalLayout_5.addWidget(self.MagValGraphZ)
        self.WidgetPages.addWidget(self.RawMagPage)
        self.EstValPage = QtWidgets.QWidget()
        self.EstValPage.setObjectName("EstValPage")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.EstValPage)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.label = QtWidgets.QLabel(self.EstValPage)
        self.label.setMaximumSize(QtCore.QSize(16777215, 50))
        font = QtGui.QFont()
        font.setFamily("Microsoft Sans Serif")
        font.setPointSize(20)
        self.label.setFont(font)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.verticalLayout_2.addWidget(self.label)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.EstValGraphX = PlotWidget(self.EstValPage)
        self.EstValGraphX.setMinimumSize(QtCore.QSize(500, 0))
        self.EstValGraphX.setObjectName("EstValGraphX")
        self.horizontalLayout_2.addWidget(self.EstValGraphX)
        self.verticalLayout_8 = QtWidgets.QVBoxLayout()
        self.verticalLayout_8.setObjectName("verticalLayout_8")
        self.dialX = QtWidgets.QDial(self.EstValPage)
        self.dialX.setStyleSheet("")
        self.dialX.setMinimum(-90)
        self.dialX.setMaximum(90)
        self.dialX.setWrapping(True)
        self.dialX.setNotchesVisible(True)
        self.dialX.setObjectName("dialX")
        self.verticalLayout_8.addWidget(self.dialX)
        self.label_7 = QtWidgets.QLabel(self.EstValPage)
        self.label_7.setMaximumSize(QtCore.QSize(16777215, 50))
        self.label_7.setMinimumSize(QtCore.QSize(150, 0))
        font = QtGui.QFont()
        font.setFamily("Microsoft Sans Serif")
        self.label_7.setFont(font)
        self.label_7.setAlignment(QtCore.Qt.AlignCenter)
        self.label_7.setObjectName("label_7")
        self.verticalLayout_8.addWidget(self.label_7)
        self.horizontalLayout_2.addLayout(self.verticalLayout_8)
        self.verticalLayout_2.addLayout(self.horizontalLayout_2)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.EstValGraphY = PlotWidget(self.EstValPage)
        self.EstValGraphY.setMinimumSize(QtCore.QSize(500, 0))
        self.EstValGraphY.setObjectName("EstValGraphY")
        self.horizontalLayout_3.addWidget(self.EstValGraphY)
        self.verticalLayout_7 = QtWidgets.QVBoxLayout()
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.dialY = QtWidgets.QDial(self.EstValPage)
        self.dialY.setMinimum(-180)
        self.dialY.setMaximum(180)
        self.dialY.setWrapping(True)
        self.dialY.setNotchesVisible(True)
        self.dialY.setObjectName("dialY")
        self.verticalLayout_7.addWidget(self.dialY)
        self.label_6 = QtWidgets.QLabel(self.EstValPage)
        self.label_6.setMaximumSize(QtCore.QSize(16777215, 50))
        self.label_6.setMinimumSize(QtCore.QSize(150, 0))
        font = QtGui.QFont()
        font.setFamily("Microsoft Sans Serif")
        self.label_6.setFont(font)
        self.label_6.setAlignment(QtCore.Qt.AlignCenter)
        self.label_6.setObjectName("label_6")
        self.verticalLayout_7.addWidget(self.label_6)
        self.horizontalLayout_3.addLayout(self.verticalLayout_7)
        self.verticalLayout_2.addLayout(self.horizontalLayout_3)
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.EstValGraphZ = PlotWidget(self.EstValPage)
        self.EstValGraphZ.setMinimumSize(QtCore.QSize(500, 0))
        self.EstValGraphZ.setObjectName("EstValGraphZ")
        self.horizontalLayout_4.addWidget(self.EstValGraphZ)
        self.verticalLayout_6 = QtWidgets.QVBoxLayout()
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.dialZ = QtWidgets.QDial(self.EstValPage)
        self.dialZ.setStyleSheet("")
        self.dialZ.setMinimum(0)
        self.dialZ.setMaximum(360)
        self.dialZ.setWrapping(True)
        self.dialZ.setNotchesVisible(True)
        self.dialZ.setObjectName("dialZ")
        self.verticalLayout_6.addWidget(self.dialZ)
        self.label_5 = QtWidgets.QLabel(self.EstValPage)
        self.label_5.setMaximumSize(QtCore.QSize(16777215, 50))
        self.label_5.setMinimumSize(QtCore.QSize(150, 0))
        font = QtGui.QFont()
        font.setFamily("Microsoft Sans Serif")
        self.label_5.setFont(font)
        self.label_5.setAlignment(QtCore.Qt.AlignCenter)
        self.label_5.setObjectName("label_5")
        self.verticalLayout_6.addWidget(self.label_5)
        self.horizontalLayout_4.addLayout(self.verticalLayout_6)
        self.verticalLayout_2.addLayout(self.horizontalLayout_4)
        self.WidgetPages.addWidget(self.EstValPage)
        self.horizontalLayout.addWidget(self.WidgetPages)
        self.gridLayout.addWidget(self.WidgetContents, 0, 1, 1, 1)
        self.WidgetMenu = QtWidgets.QWidget(self.centralwidget)
        self.WidgetMenu.setMaximumSize(QtCore.QSize(500, 16777215))
        self.WidgetMenu.setObjectName("WidgetMenu")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.WidgetMenu)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.FrameJustifier = QtWidgets.QFrame(self.WidgetMenu)
        self.FrameJustifier.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.FrameJustifier.setFrameShadow(QtWidgets.QFrame.Raised)
        self.FrameJustifier.setObjectName("FrameJustifier")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.FrameJustifier)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.CamFeedJustifier = QtWidgets.QWidget(self.FrameJustifier)
        self.CamFeedJustifier.setObjectName("CamFeedJustifier")
        self.gridLayout_4 = QtWidgets.QGridLayout(self.CamFeedJustifier)
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.CamFeedLabel = QtWidgets.QLabel(self.CamFeedJustifier)
        self.CamFeedLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.CamFeedLabel.setText("")
        self.CamFeedLabel.setObjectName("CamFeedLabel")
        self.gridLayout_4.addWidget(self.CamFeedLabel, 2, 0, 1, 1)
        self.TitleLabel = QtWidgets.QLabel(self.CamFeedJustifier)
        self.TitleLabel.setMaximumSize(QtCore.QSize(16777215, 50))
        font = QtGui.QFont()
        font.setFamily("Microsoft Sans Serif")
        font.setPointSize(22)
        self.TitleLabel.setFont(font)
        self.TitleLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.TitleLabel.setObjectName("TitleLabel")
        self.gridLayout_4.addWidget(self.TitleLabel, 1, 0, 1, 1)
        self.gridLayout_3.addWidget(self.CamFeedJustifier, 0, 0, 1, 1)
        self.PageSelectors = QtWidgets.QFrame(self.FrameJustifier)
        self.PageSelectors.setStyleSheet("background-color:rgb(153, 153, 153);\n"
                                        "border-style: outset;\n"
                                        "border-width: 2px;\n"
                                        "border-radius: 20px;\n"
                                        "border-color: grey;\n"
                                        "padding: 4px;")
        self.PageSelectors.setFrameShape(QtWidgets.QFrame.WinPanel)
        self.PageSelectors.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.PageSelectors.setObjectName("PageSelectors")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.PageSelectors)
        self.verticalLayout.setObjectName("verticalLayout")
        self.RawAccBtn = QtWidgets.QPushButton(self.PageSelectors)
        self.RawAccBtn.setMinimumSize(QtCore.QSize(0, 70))
        font = QtGui.QFont()
        font.setFamily("Microsoft Sans Serif")
        font.setPointSize(16)
        self.RawAccBtn.setFont(font)
        self.RawAccBtn.setStyleSheet("background-color: lightgrey;\n"
                                    "border-style: outset;\n"
                                    "border-color: grey;\n"
                                    "border-width: 2px;\n"
                                    "border-radius: 10px;\n"
                                    "padding: 4px;\n"
                                    "\n"
                                    "QPushbutton::pressed\n"
                                    "{\n"
                                    "background-color: grey;\n"
                                    "border-style: inset;\n"
                                    "border-color: grey;\n"
                                    "border-width: 2px;\n"
                                    "border-radius: 10px;\n"
                                    "padding: 4px;\n"
                                    "}")
        self.RawAccBtn.setFlat(False)
        self.RawAccBtn.setObjectName("RawAccBtn")
        self.verticalLayout.addWidget(self.RawAccBtn)
        self.RawGyroBtn = QtWidgets.QPushButton(self.PageSelectors)
        self.RawGyroBtn.setMinimumSize(QtCore.QSize(0, 70))
        font = QtGui.QFont()
        font.setFamily("Microsoft Sans Serif")
        font.setPointSize(16)
        self.RawGyroBtn.setFont(font)
        self.RawGyroBtn.setStyleSheet("background-color: lightgrey;\n"
                                    "border-style: outset;\n"
                                    "border-color: grey;\n"
                                    "border-width: 2px;\n"
                                    "border-radius: 10px;\n"
                                    "padding: 4px;\n"
                                    "\n"
                                    "QPushbutton::pressed\n"
                                    "{\n"
                                    "background-color: grey;\n"
                                    "border-style: inset;\n"
                                    "border-color: grey;\n"
                                    "border-width: 2px;\n"
                                    "border-radius: 10px;\n"
                                    "padding: 4px;\n"
                                    "}")
        self.RawGyroBtn.setObjectName("RawGyroBtn")
        self.verticalLayout.addWidget(self.RawGyroBtn)
        self.RawMagBtn = QtWidgets.QPushButton(self.PageSelectors)
        self.RawMagBtn.setMinimumSize(QtCore.QSize(0, 70))
        font = QtGui.QFont()
        font.setFamily("Microsoft Sans Serif")
        font.setPointSize(16)
        self.RawMagBtn.setFont(font)
        self.RawMagBtn.setStyleSheet("background-color: lightgrey;\n"
                                    "border-style: outset;\n"
                                    "border-color: grey;\n"
                                    "border-width: 2px;\n"
                                    "border-radius: 10px;\n"
                                    "padding: 4px;\n"
                                    "\n"
                                    "QPushbutton::pressed\n"
                                    "{\n"
                                    "background-color: grey;\n"
                                    "border-style: inset;\n"
                                    "border-color: grey;\n"
                                    "border-width: 2px;\n"
                                    "border-radius: 10px;\n"
                                    "padding: 4px;\n"
                                    "}")
        self.RawMagBtn.setObjectName("RawMagBtn")
        self.verticalLayout.addWidget(self.RawMagBtn)
        self.EstValBtn = QtWidgets.QPushButton(self.PageSelectors)
        self.EstValBtn.setMinimumSize(QtCore.QSize(0, 70))
        font = QtGui.QFont()
        font.setFamily("Microsoft Sans Serif")
        font.setPointSize(16)
        self.EstValBtn.setFont(font)
        self.EstValBtn.setStyleSheet("background-color: lightgrey;\n"
                                    "border-style: outset;\n"
                                    "border-color: grey;\n"
                                    "border-width: 2px;\n"
                                    "border-radius: 10px;\n"
                                    "padding: 4px;\n"
                                    "\n"
                                    "QPushbutton::pressed\n"
                                    "{\n"
                                    "background-color: grey;\n"
                                    "border-style: inset;\n"
                                    "border-color: grey;\n"
                                    "border-width: 2px;\n"
                                    "border-radius: 10px;\n"
                                    "padding: 4px;\n"
                                    "}")
        self.EstValBtn.setObjectName("EstValBtn")
        self.verticalLayout.addWidget(self.EstValBtn)
        self.gridLayout_3.addWidget(self.PageSelectors, 2, 0, 1, 1)
        self.MenuLabel = QtWidgets.QLabel(self.FrameJustifier)
        self.MenuLabel.setMaximumSize(QtCore.QSize(16777215, 30))
        font = QtGui.QFont()
        font.setFamily("Microsoft Sans Serif")
        font.setPointSize(20)
        self.MenuLabel.setFont(font)
        self.MenuLabel.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.MenuLabel.setFrameShadow(QtWidgets.QFrame.Plain)
        self.MenuLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.MenuLabel.setObjectName("MenuLabel")
        self.gridLayout_3.addWidget(self.MenuLabel, 1, 0, 1, 1)
        self.gridLayout_2.addWidget(self.FrameJustifier, 0, 0, 1, 1)
        self.gridLayout.addWidget(self.WidgetMenu, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        self.WidgetPages.setCurrentIndex(3)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label_2.setText(_translate("MainWindow", "Raw Accelerometer Values"))
        self.label_3.setText(_translate("MainWindow", "Raw Gyroscope Values"))
        self.label_4.setText(_translate("MainWindow", "Raw Magnetometer Values"))
        self.label.setText(_translate("MainWindow", "Estimated Attitude Values"))
        self.label_7.setText(_translate("MainWindow", "Estimated Roll:"))
        self.label_6.setText(_translate("MainWindow", "Estimated Pitch:"))
        self.label_5.setText(_translate("MainWindow", "Estimated Yaw:"))
        self.TitleLabel.setText(_translate("MainWindow", "Position and Orientation Estimation"))
        self.RawAccBtn.setText(_translate("MainWindow", "Raw Accelerometer Values"))
        self.RawGyroBtn.setText(_translate("MainWindow", "Raw Gyroscope Values"))
        self.RawMagBtn.setText(_translate("MainWindow", "Raw Magnetometer Values"))
        self.EstValBtn.setText(_translate("MainWindow", "Estimated Attitude Values"))
        self.MenuLabel.setText(_translate("MainWindow", "Menu"))

        # Graph showing estimated values around X-axis
        self.EstValGraphX.setBackground('w')
        self.EstValGraphX.setTitle("Angle X (Roll) in degrees", color="b", size="15pt")
        styles = {"color": "#f00", "font-size": "15px"}
        self.EstValGraphX.setLabel("left", "Roll Angle", **styles)
        self.EstValGraphX.setLabel("bottom", "Time steps", **styles)
        self.EstValGraphX.addLegend()
        self.EstValGraphX.showGrid(x=True, y=True)
        self.EstValGraphX.setYRange(-180, 180, padding=0)

        # Graph showing estimated values around Y-axis
        self.EstValGraphY.setBackground('w')
        self.EstValGraphY.setTitle("Angle Y (Pitch) in degrees", color="b", size="15pt")
        styles = {"color": "#f00", "font-size": "15px"}
        self.EstValGraphY.setLabel("left", "Pitch Angle", **styles)
        self.EstValGraphY.setLabel("bottom", "Time steps", **styles)
        self.EstValGraphY.addLegend()
        self.EstValGraphY.showGrid(x=True, y=True)
        self.EstValGraphY.setYRange(-90, 90, padding=0)

        # Graph showing estimated values around Z-axis
        self.EstValGraphZ.setBackground('w')
        self.EstValGraphZ.setTitle("Angle Z (Yaw) in degrees", color="b", size="15pt")
        styles = {"color": "#f00", "font-size": "15px"}
        self.EstValGraphZ.setLabel("left", "Yaw Angle", **styles)
        self.EstValGraphZ.setLabel("bottom", "Time steps", **styles)
        self.EstValGraphZ.addLegend()
        self.EstValGraphZ.showGrid(x=True, y=True)
        self.EstValGraphZ.setYRange(0, 360, padding=0)

        # Graph showing raw accelerometer data X
        self.AccValGraphX.setBackground('w')
        self.AccValGraphX.setTitle("Accelerometer X", color="b", size="15pt")
        styles = {"color": "#f00", "font-size": "15px"}
        self.AccValGraphX.setLabel("left", "m/s<sup>2</sup>", **styles)
        self.AccValGraphX.setLabel("bottom", "Time steps", **styles)
        self.AccValGraphX.addLegend()
        self.AccValGraphX.showGrid(x=True, y=True)

        # Graph showing raw accelerometer data Y
        self.AccValGraphY.setBackground('w')
        self.AccValGraphY.setTitle("Accelerometer Y", color="b", size="15pt")
        styles = {"color": "#f00", "font-size": "15px"}
        self.AccValGraphY.setLabel("left", "m/s<sup>2</sup>", **styles)
        self.AccValGraphY.setLabel("bottom", "Time steps", **styles)
        self.AccValGraphY.addLegend()
        self.AccValGraphY.showGrid(x=True, y=True)

        # Graph showing raw accelerometer data Z
        self.AccValGraphZ.setBackground('w')
        self.AccValGraphZ.setTitle("Accelerometer Z", color="b", size="15pt")
        styles = {"color": "#f00", "font-size": "15px"}
        self.AccValGraphZ.setLabel("left", "m/s<sup>2</sup>", **styles)
        self.AccValGraphZ.setLabel("bottom", "Time steps", **styles)
        self.AccValGraphZ.addLegend()
        self.AccValGraphZ.showGrid(x=True, y=True)

        # Graph showing raw gyroscope data X
        self.GyroValGraphX.setBackground('w')
        self.GyroValGraphX.setTitle("Gyroscope X", color="b", size="15pt")
        styles = {"color": "#f00", "font-size": "15px"}
        self.GyroValGraphX.setLabel("left", "degrees/second", **styles)
        self.GyroValGraphX.setLabel("bottom", "Time steps", **styles)
        self.GyroValGraphX.addLegend()
        self.GyroValGraphX.showGrid(x=True, y=True)

        # Graph showing raw gyroscope data Y
        self.GyroValGraphY.setBackground('w')
        self.GyroValGraphY.setTitle("Gyroscope Y", color="b", size="15pt")
        styles = {"color": "#f00", "font-size": "15px"}
        self.GyroValGraphY.setLabel("left", "degrees/second", **styles)
        self.GyroValGraphY.setLabel("bottom", "Time steps", **styles)
        self.GyroValGraphY.addLegend()
        self.GyroValGraphY.showGrid(x=True, y=True)

        # Graph showing raw gyroscope data Z
        self.GyroValGraphZ.setBackground('w')
        self.GyroValGraphZ.setTitle("Gyroscope Z", color="b", size="15pt")
        styles = {"color": "#f00", "font-size": "15px"}
        self.GyroValGraphZ.setLabel("left", "degrees/second", **styles)
        self.GyroValGraphZ.setLabel("bottom", "Time steps", **styles)
        self.GyroValGraphZ.addLegend()
        self.GyroValGraphZ.showGrid(x=True, y=True)

        # Graph showing raw magnetometer data X
        self.MagValGraphX.setBackground('w')
        self.MagValGraphX.setTitle("Magnetometer X", color="b", size="15pt")
        styles = {"color": "#f00", "font-size": "15px"}
        self.MagValGraphX.setLabel("left", "Micro Tesla (µT)", **styles)
        self.MagValGraphX.setLabel("bottom", "Time steps", **styles)
        self.MagValGraphX.addLegend()
        self.MagValGraphX.showGrid(x=True, y=True)

        # Graph showing raw magnetometer data Y
        self.MagValGraphY.setBackground('w')
        self.MagValGraphY.setTitle("Magnetometer Y", color="b", size="15pt")
        styles = {"color": "#f00", "font-size": "15px"}
        self.MagValGraphY.setLabel("left", "Micro Tesla (µT)", **styles)
        self.MagValGraphY.setLabel("bottom", "Time steps", **styles)
        self.MagValGraphY.addLegend()
        self.MagValGraphY.showGrid(x=True, y=True)

        # Graph showing raw magnetometer data Z
        self.MagValGraphZ.setBackground('w')
        self.MagValGraphZ.setTitle("Magnetometer Z", color="b", size="15pt")
        styles = {"color": "#f00", "font-size": "15px"}
        self.MagValGraphZ.setLabel("left", "Micro Tesla (µT)", **styles)
        self.MagValGraphZ.setLabel("bottom", "Time steps", **styles)
        self.MagValGraphZ.addLegend()
        self.MagValGraphZ.showGrid(x=True, y=True)


class ControlMainWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super(ControlMainWindow, self).__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.numberOfPlottingValues = 300

        # Connecting the menu buttons
        self.ui.RawAccBtn.clicked.connect(lambda : self.ui.WidgetPages.setCurrentIndex(0))
        self.ui.RawGyroBtn.clicked.connect(lambda : self.ui.WidgetPages.setCurrentIndex(1))
        self.ui.RawMagBtn.clicked.connect(lambda : self.ui.WidgetPages.setCurrentIndex(2))
        self.ui.EstValBtn.clicked.connect(lambda : self.ui.WidgetPages.setCurrentIndex(3))

        self.capture = cv2.VideoCapture('videoplayback.mp4')

        # Create variables for holding the last 200 plotting values
        self.time_stamps = list(range(self.numberOfPlottingValues))
        ## ESTIMATE GRAPHS ##
        ## Roll ##
        self.est_roll_list = [0 for _ in range(self.numberOfPlottingValues)]
        self.meas_roll_list = [0 for _ in range(self.numberOfPlottingValues)]
        self.est_roll_line = self.ui.EstValGraphX.plot(self.time_stamps, self.est_roll_list, name="Kalman Roll angle", pen=pg.mkPen(color='r'))
        self.meas_roll_line = self.ui.EstValGraphX.plot(self.time_stamps, self.meas_roll_list, name="Pre-Calculated Roll angle", pen=pg.mkPen(color='b'))
        ## Pitch ##
        self.est_pitch_list = [0 for _ in range(self.numberOfPlottingValues)]
        self.meas_pitch_list = [0 for _ in range(self.numberOfPlottingValues)]
        self.est_pitch_line = self.ui.EstValGraphY.plot(self.time_stamps, self.est_pitch_list, name="Kalman Pitch angle", pen=pg.mkPen(color='r'))
        self.meas_pitch_line = self.ui.EstValGraphY.plot(self.time_stamps, self.meas_pitch_list, name="Pre-Calculated Pitch angle", pen=pg.mkPen(color='b'))
        ## Yaw ##
        self.est_yaw_list = [0 for _ in range(self.numberOfPlottingValues)]
        self.meas_yaw_list = [0 for _ in range(self.numberOfPlottingValues)]
        self.est_yaw_line = self.ui.EstValGraphZ.plot(self.time_stamps, self.est_yaw_list, name="Estimated Yaw angle", pen=pg.mkPen(color='r'))
        self.meas_yaw_line = self.ui.EstValGraphZ.plot(self.time_stamps, self.meas_yaw_list, name="Madgwick Yaw angle", pen=pg.mkPen(color='b'))

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
        self.s2_roll = 0.1 ** 2
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
        self.s2_pitch = 0.1 ** 2
        self.est_state_pitch = np.zeros(2)
        self.est_angularVel_pitch = 0
        self.est_angle_pitch = 0
        self.meas_pitch = 0
        accelerometerY_variance = 0.0005460492817487228 #0.00025083176623752354
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
        self.beta = 0.041#0.001590805977558896#*math.pi/180 #0.0024
        self.madgwick = MadgwickFilter(beta=self.beta)
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

        # Connect a timer to the update GUI function to update the plots at a given interval
        self.timer = QtCore.QTimer()
        self.timer.setInterval(5)
        self.timer.timeout.connect(self.updateGUI)
        self.timer.start()

        # Starting a thread to listen for inputs from Arduino
        self.listenThread = Thread(target=self.readSerialInputs, args=())
        self.listenThread.daemon = True
        self.listenThread.start()

        # Starting a thread to capture camera video
        self.videoThread = Thread(target=self.showVideo, args=())
        self.videoThread.daemon = True
        self.videoThread.start()

    def readSerialInputs(self):
        # Initial Arduino reading
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
                #Gyro X Mean:  -0.028353774399723617
                #Gyro Y Mean:  -0.08471584038694076
                #Gyro Z Mean:  0.055643461737778545
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
        self.mad_roll, self.mad_pitch, self.mad_yaw = quaternion.quaternion_to_euler(self.madgwick.q, as_degrees=True)
        if self.mad_yaw < 0:
            self.mad_yaw = self.mad_yaw + 360

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

    def updateGUI(self):
        self.time_stamps = self.time_stamps[1:] #Remove first element
        self.time_stamps.append(self.time_stamps[-1] + 1)

        self.est_roll_list = self.est_roll_list[1:]
        self.est_roll_list.append(float(self.est_angle_roll))

        self.meas_roll_list = self.meas_roll_list[1:]
        self.meas_roll_list.append(float(self.meas_roll))

        self.est_pitch_list = self.est_pitch_list[1:]
        self.est_pitch_list.append(float(self.est_angle_pitch))

        self.meas_pitch_list = self.meas_pitch_list[1:]
        self.meas_pitch_list.append(float(self.meas_pitch))

        self.meas_yaw_list = self.meas_yaw_list[1:]
        self.meas_yaw_list.append(float(self.mad_yaw))

        self.est_yaw_list = self.est_yaw_list[1:]
        self.est_yaw_list.append(float(self.est_angle_yaw))

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
        self.ui.dialZ.setValue(int(self.est_angle_yaw))
        #self.ui.dialZ.setValue(int(self.mad_yaw))

        roll_text = 'Estimated Roll: ' + str(int(self.est_angle_roll)) + '°'
        #roll_text = ('Madg: ' + str(int(self.mad_roll)) + '°' + '\n' +
        #            'Kalman: ' + str(int(self.est_angle_roll))) + '°'
        pitch_text = 'Estimated Pitch: ' + str(int(self.est_angle_pitch)) + '°'
        #pitch_text = ('Madg: ' + str(int(self.mad_pitch)) + '°' + '\n' +
        #             'Kalman: ' + str(int(self.est_angle_pitch))) + '°'
        yaw_text = 'Estimated Yaw: ' + str(int(self.est_angle_yaw)) + '°'
        #yaw_text = 'Yaw: ' + str(int(self.mad_yaw)) + '°'

        self.ui.label_7.setText(roll_text)
        self.ui.label_6.setText(pitch_text)
        self.ui.label_5.setText(yaw_text)

        self.est_roll_line.setData(self.time_stamps, self.est_roll_list)
        self.meas_roll_line.setData(self.time_stamps, self.meas_roll_list)
        self.est_pitch_line.setData(self.time_stamps, self.est_pitch_list)
        self.meas_pitch_line.setData(self.time_stamps, self.meas_pitch_list)
        self.est_yaw_line.setData(self.time_stamps, self.est_yaw_list)
        self.meas_yaw_line.setData(self.time_stamps, self.meas_yaw_list)

        self.accX_line.setData(self.time_stamps, self.accX_list)
        self.accY_line.setData(self.time_stamps, self.accY_list)
        self.accZ_line.setData(self.time_stamps, self.accZ_list)
        self.gyroX_line.setData(self.time_stamps, self.gyroX_list)
        self.gyroY_line.setData(self.time_stamps, self.gyroY_list)
        self.gyroZ_line.setData(self.time_stamps, self.gyroZ_list)
        self.magX_line.setData(self.time_stamps, self.magX_list)
        self.magY_line.setData(self.time_stamps, self.magY_list)
        self.magZ_line.setData(self.time_stamps, self.magZ_list)

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
                time.sleep(0.02)
            else:
                self.ui.CamFeedLabel.setText('Never Gonna Give You Up <3')


if __name__ == '__main__':
    arduino = serial.Serial(port='/dev/cu.usbserial-DN041PFR', baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=0)
    import sys
    app = QtWidgets.QApplication(sys.argv)
    mySW = ControlMainWindow()
    mySW.show()

    sys.exit(app.exec_())

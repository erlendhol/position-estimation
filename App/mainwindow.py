
from PyQt5 import QtCore, QtGui, QtWidgets
from pyqtgraph import PlotWidget

"""
A class made for creating the GUI layout in an application made for a
bachelor project.

The class is made using the Qt framework.
"""
class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1275, 808)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setStyleSheet("background-color: rgb(127, 127, 127);")
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout.setObjectName("gridLayout")
        self.WidgetContents = QtWidgets.QWidget(self.centralwidget)
        self.WidgetContents.setStyleSheet("background-color:rgb(153, 153, 153)")
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
        self.verticalLayout_9 = QtWidgets.QVBoxLayout()
        self.verticalLayout_9.setObjectName("verticalLayout_9")
        self.label_8 = QtWidgets.QLabel(self.EstValPage)
        self.label_8.setMinimumSize(QtCore.QSize(150, 0))
        self.label_8.setMaximumSize(QtCore.QSize(16777215, 30))
        font = QtGui.QFont()
        font.setFamily("Microsoft Sans Serif")
        self.label_8.setFont(font)
        self.label_8.setAlignment(QtCore.Qt.AlignCenter)
        self.label_8.setObjectName("label_8")
        self.verticalLayout_9.addWidget(self.label_8)

        self.SensorFrameXBtn = QtWidgets.QCheckBox(self.EstValPage)
        self.SensorFrameXBtn.setObjectName("SensorFrameXBtn")
        self.verticalLayout_9.addWidget(self.SensorFrameXBtn)
        self.ShipFrameXBtn = QtWidgets.QCheckBox(self.EstValPage)
        self.ShipFrameXBtn.setObjectName("ShipFrameXBtn")
        self.verticalLayout_9.addWidget(self.ShipFrameXBtn)

        self.horizontalLayout_2.addLayout(self.verticalLayout_9)
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
        self.verticalLayout_10 = QtWidgets.QVBoxLayout()
        self.verticalLayout_10.setObjectName("verticalLayout_10")
        self.label_9 = QtWidgets.QLabel(self.EstValPage)
        self.label_9.setMinimumSize(QtCore.QSize(150, 0))
        self.label_9.setMaximumSize(QtCore.QSize(16777215, 30))
        font = QtGui.QFont()
        font.setFamily("Microsoft Sans Serif")
        self.label_9.setFont(font)
        self.label_9.setAlignment(QtCore.Qt.AlignCenter)
        self.label_9.setObjectName("label_9")
        self.verticalLayout_10.addWidget(self.label_9)
        self.SensorFrameYBtn = QtWidgets.QCheckBox(self.EstValPage)
        self.SensorFrameYBtn.setObjectName("SensorFrameYBtn")
        self.verticalLayout_10.addWidget(self.SensorFrameYBtn)
        self.ShipFrameYBtn = QtWidgets.QCheckBox(self.EstValPage)
        self.ShipFrameYBtn.setObjectName("ShipFrameYBtn")
        self.verticalLayout_10.addWidget(self.ShipFrameYBtn)
        self.label_12 = QtWidgets.QLabel(self.EstValPage)
        self.label_12.setMaximumSize(QtCore.QSize(16777215, 20))
        font = QtGui.QFont()
        font.setFamily("Microsoft Sans Serif")
        self.label_12.setFont(font)
        self.label_12.setAlignment(QtCore.Qt.AlignCenter)
        self.label_12.setObjectName("label_12")
        self.verticalLayout_10.addWidget(self.label_12)
        self.EncoderPitch = QtWidgets.QDoubleSpinBox(self.EstValPage)
        self.EncoderPitch.setMaximumSize(QtCore.QSize(200, 16777215))
        self.EncoderPitch.setMinimum(-90)
        self.EncoderPitch.setMaximum(90)
        self.EncoderPitch.setObjectName("EncoderPitch")
        self.verticalLayout_10.addWidget(self.EncoderPitch)
        self.horizontalLayout_3.addLayout(self.verticalLayout_10)
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
        self.verticalLayout_11 = QtWidgets.QVBoxLayout()
        self.verticalLayout_11.setObjectName("verticalLayout_11")
        self.label_10 = QtWidgets.QLabel(self.EstValPage)
        self.label_10.setMinimumSize(QtCore.QSize(150, 0))
        self.label_10.setMaximumSize(QtCore.QSize(16777215, 30))
        self.label_10.setAlignment(QtCore.Qt.AlignCenter)
        self.label_10.setObjectName("label_10")
        self.verticalLayout_11.addWidget(self.label_10)
        self.SensorFrameZBtn = QtWidgets.QCheckBox(self.EstValPage)
        self.SensorFrameZBtn.setObjectName("SensorFrameZBtn")
        self.verticalLayout_11.addWidget(self.SensorFrameZBtn)
        self.ShipFrameZBtn = QtWidgets.QCheckBox(self.EstValPage)
        self.ShipFrameZBtn.setObjectName("ShipFrameZBtn")
        self.verticalLayout_11.addWidget(self.ShipFrameZBtn)
        self.label_11 = QtWidgets.QLabel(self.EstValPage)
        self.label_11.setMaximumSize(QtCore.QSize(16777215, 20))
        font = QtGui.QFont()
        font.setFamily("Microsoft Sans Serif")
        self.label_11.setFont(font)
        self.label_11.setAlignment(QtCore.Qt.AlignCenter)
        self.label_11.setObjectName("label_11")
        self.verticalLayout_11.addWidget(self.label_11)
        self.EncoderYaw = QtWidgets.QDoubleSpinBox(self.EstValPage)
        self.EncoderYaw.setMaximumSize(QtCore.QSize(200, 16777215))
        self.EncoderYaw.setMinimum(0)
        self.EncoderYaw.setMaximum(180)
        self.EncoderYaw.setObjectName("EncoderYaw")
        self.verticalLayout_11.addWidget(self.EncoderYaw)
        self.horizontalLayout_4.addLayout(self.verticalLayout_11)
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
        self.CamFeedLabel.setText("")
        self.CamFeedLabel.setAlignment(QtCore.Qt.AlignCenter)
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
        self.RawAccBtn.setCheckable(False)
        self.RawAccBtn.setChecked(False)
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

        self.StartBtn = QtWidgets.QPushButton(self.FrameJustifier)
        #self.StartBtn.setMinimumSize(QtCore.QSize(0, 70))
        font = QtGui.QFont()
        font.setFamily("Microsoft Sans Serif")
        font.setPointSize(16)
        self.StartBtn.setFont(font)
        self.StartBtn.setStyleSheet("background-color: lightgrey;\n"
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
        self.StartBtn.setObjectName("StartBtn")
        self.gridLayout_3.addWidget(self.StartBtn)

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
        self.label.setText(_translate("MainWindow", "Estimated Attitude Values For Gangway and Ship"))
        self.label_8.setText(_translate("MainWindow", "Frame Representation"))
        self.SensorFrameXBtn.setText(_translate("MainWindow", "Sensor Roll to World"))
        self.ShipFrameXBtn.setText(_translate("MainWindow", "Ship Pitch to World"))
        self.label_7.setText(_translate("MainWindow", "Estimated Roll:"))
        self.label_9.setText(_translate("MainWindow", "Frame Representation"))
        self.SensorFrameYBtn.setText(_translate("MainWindow", "Sensor Pitch to World"))
        self.ShipFrameYBtn.setText(_translate("MainWindow", "Ship Roll to World"))
        self.label_12.setText(_translate("MainWindow", "Encoder Value Boom"))
        self.label_6.setText(_translate("MainWindow", "Estimated Pitch:"))
        self.label_10.setText(_translate("MainWindow", "Frame Representation"))
        self.SensorFrameZBtn.setText(_translate("MainWindow", "Sensor Yaw to Init Position"))
        self.ShipFrameZBtn.setText(_translate("MainWindow", "Ship Yaw to Init Position"))
        self.label_11.setText(_translate("MainWindow", "Encoder Value Slew"))
        self.label_5.setText(_translate("MainWindow", "Estimated Yaw:"))
        self.TitleLabel.setText(_translate("MainWindow", "Position and Orientation Estimation"))
        self.RawAccBtn.setText(_translate("MainWindow", "Raw Accelerometer Values"))
        self.RawGyroBtn.setText(_translate("MainWindow", "Raw Gyroscope Values"))
        self.RawMagBtn.setText(_translate("MainWindow", "Raw Magnetometer Values"))
        self.EstValBtn.setText(_translate("MainWindow", "Estimated Attitude Values"))
        self.StartBtn.setText(_translate("MainWindow", "Start Measuring"))
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
        self.EstValGraphZ.setYRange(-180, 180, padding=0)

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

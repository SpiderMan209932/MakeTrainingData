# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'MakeTrainingData.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1280, 784)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.SIzeHeight = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.SIzeHeight.setGeometry(QtCore.QRect(900, 20, 211, 31))
        self.SIzeHeight.setObjectName("SIzeHeight")
        self.SizeWidth = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.SizeWidth.setGeometry(QtCore.QRect(900, 50, 211, 31))
        self.SizeWidth.setObjectName("SizeWidth")
        self.SizeLength = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.SizeLength.setGeometry(QtCore.QRect(900, 80, 211, 31))
        self.SizeLength.setObjectName("SizeLength")
        self.PositionX = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.PositionX.setGeometry(QtCore.QRect(900, 110, 211, 31))
        self.PositionX.setObjectName("PositionX")
        self.PositionY = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.PositionY.setGeometry(QtCore.QRect(900, 140, 211, 31))
        self.PositionY.setObjectName("PositionY")
        self.PositionZ = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.PositionZ.setGeometry(QtCore.QRect(900, 170, 211, 31))
        self.PositionZ.setObjectName("PositionZ")
        self.AngleYaw = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.AngleYaw.setGeometry(QtCore.QRect(900, 200, 211, 31))
        self.AngleYaw.setObjectName("AngleYaw")
        self.TextWidth = QtWidgets.QLineEdit(self.centralwidget)
        self.TextWidth.setGeometry(QtCore.QRect(860, 50, 41, 31))
        self.TextWidth.setAlignment(QtCore.Qt.AlignCenter)
        self.TextWidth.setObjectName("TextWidth")
        self.lineEdit = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit.setGeometry(QtCore.QRect(860, 20, 41, 31))
        self.lineEdit.setAlignment(QtCore.Qt.AlignCenter)
        self.lineEdit.setObjectName("lineEdit")
        self.TextLength = QtWidgets.QLineEdit(self.centralwidget)
        self.TextLength.setGeometry(QtCore.QRect(860, 80, 41, 31))
        self.TextLength.setAlignment(QtCore.Qt.AlignCenter)
        self.TextLength.setObjectName("TextLength")
        self.TextX = QtWidgets.QLineEdit(self.centralwidget)
        self.TextX.setGeometry(QtCore.QRect(860, 110, 41, 31))
        self.TextX.setAlignment(QtCore.Qt.AlignCenter)
        self.TextX.setObjectName("TextX")
        self.TextY = QtWidgets.QLineEdit(self.centralwidget)
        self.TextY.setGeometry(QtCore.QRect(860, 140, 41, 31))
        self.TextY.setAlignment(QtCore.Qt.AlignCenter)
        self.TextY.setObjectName("TextY")
        self.TextZ = QtWidgets.QLineEdit(self.centralwidget)
        self.TextZ.setGeometry(QtCore.QRect(860, 170, 41, 31))
        self.TextZ.setAlignment(QtCore.Qt.AlignCenter)
        self.TextZ.setObjectName("TextZ")
        self.TextYaw = QtWidgets.QLineEdit(self.centralwidget)
        self.TextYaw.setGeometry(QtCore.QRect(860, 200, 41, 31))
        self.TextYaw.setAlignment(QtCore.Qt.AlignCenter)
        self.TextYaw.setObjectName("TextYaw")
        self.ObjecClass = QtWidgets.QComboBox(self.centralwidget)
        self.ObjecClass.setGeometry(QtCore.QRect(860, 240, 251, 31))
        self.ObjecClass.setObjectName("ObjecClass")
        self.ObjecClass.addItem("")
        self.ObjecClass.addItem("")
        self.ObjecClass.addItem("")
        self.ObjecClass.addItem("")
        self.ObjecClass.addItem("")
        self.ObjecClass.addItem("")
        self.NextFrame = QtWidgets.QPushButton(self.centralwidget)
        self.NextFrame.setGeometry(QtCore.QRect(360, 670, 121, 61))
        self.NextFrame.setObjectName("NextFrame")
        self.BeforeFrame = QtWidgets.QPushButton(self.centralwidget)
        self.BeforeFrame.setGeometry(QtCore.QRect(220, 670, 121, 61))
        self.BeforeFrame.setObjectName("BeforeFrame")
        self.graphicsView = QtWidgets.QGraphicsView(self.centralwidget)
        self.graphicsView.setGeometry(QtCore.QRect(10, 10, 711, 641))
        self.graphicsView.setObjectName("graphicsView")
        self.graphicsView_2 = QtWidgets.QGraphicsView(self.centralwidget)
        self.graphicsView_2.setGeometry(QtCore.QRect(730, 290, 531, 361))
        self.graphicsView_2.setObjectName("graphicsView_2")
        self.Before2DBox = QtWidgets.QPushButton(self.centralwidget)
        self.Before2DBox.setGeometry(QtCore.QRect(880, 670, 121, 61))
        self.Before2DBox.setObjectName("Before2DBox")
        self.NextFrame_2 = QtWidgets.QPushButton(self.centralwidget)
        self.NextFrame_2.setGeometry(QtCore.QRect(1020, 670, 121, 61))
        self.NextFrame_2.setObjectName("NextFrame_2")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1280, 28))
        self.menubar.setObjectName("menubar")
        self.menuFile = QtWidgets.QMenu(self.menubar)
        self.menuFile.setObjectName("menuFile")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.OpenFile = QtWidgets.QAction(MainWindow)
        self.OpenFile.setObjectName("OpenFile")
        self.SaveFile = QtWidgets.QAction(MainWindow)
        self.SaveFile.setObjectName("SaveFile")
        self.menuFile.addAction(self.OpenFile)
        self.menuFile.addAction(self.SaveFile)
        self.menubar.addAction(self.menuFile.menuAction())

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.TextWidth.setText(_translate("MainWindow", "W"))
        self.lineEdit.setText(_translate("MainWindow", "H"))
        self.TextLength.setText(_translate("MainWindow", "L"))
        self.TextX.setText(_translate("MainWindow", "X"))
        self.TextY.setText(_translate("MainWindow", "Y"))
        self.TextZ.setText(_translate("MainWindow", "Z"))
        self.TextYaw.setText(_translate("MainWindow", "Yaw"))
        self.ObjecClass.setItemText(0, _translate("MainWindow", "Class"))
        self.ObjecClass.setItemText(1, _translate("MainWindow", "Car"))
        self.ObjecClass.setItemText(2, _translate("MainWindow", "Pedestrian"))
        self.ObjecClass.setItemText(3, _translate("MainWindow", "Cyclist"))
        self.ObjecClass.setItemText(4, _translate("MainWindow", "Truck"))
        self.ObjecClass.setItemText(5, _translate("MainWindow", "Bus"))
        self.NextFrame.setText(_translate("MainWindow", "|▶"))
        self.BeforeFrame.setText(_translate("MainWindow", "◀|"))
        self.Before2DBox.setText(_translate("MainWindow", "◁|"))
        self.NextFrame_2.setText(_translate("MainWindow", "|▷"))
        self.menuFile.setTitle(_translate("MainWindow", "File"))
        self.OpenFile.setText(_translate("MainWindow", "開く"))
        self.OpenFile.setShortcut(_translate("MainWindow", "Ctrl+O"))
        self.SaveFile.setText(_translate("MainWindow", "保存"))
        self.SaveFile.setShortcut(_translate("MainWindow", "Ctrl+S"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())


# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'MakeTrainingData.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets, QtOpenGL
from PyQt5.QtWidgets import QFileDialog
from PyQt5.QtCore    import pyqtSlot
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import os
import cv2
import mayavi.mlab as mlab
import numpy as np
from viz_util import draw_lidar, draw_frustum_pc, draw_box2d, draw_gt_boxes3d
import trans_util
from file_util import *

class Ui_MainWindow(object):
    def __init__(self):
        self.frustum_number = 0
        self.FileOp = None
        self.fig = mlab.figure(figure=None, bgcolor=(0,0,0), fgcolor=None, engine=None, size=(1000, 1000))
        self.max_frustum_number = 0
        self.objects = None
        self.box3d = None
        self.type = None
        self.h = 0
        self.w = 0
        self.l = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.yaw = 0
        self.xmin = 0
        self.ymin = 0
        self.xmax = 0
        self.ymax = 0
        
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(272, 700)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")

         # 文字
        self.TextHeight = QtWidgets.QLabel(self.centralwidget)
        self.TextHeight.setGeometry(QtCore.QRect(10, 20, 41, 31))
        self.TextHeight.setAlignment(QtCore.Qt.AlignCenter)
        self.TextHeight.setObjectName("TextHeight")
        self.TextWidth = QtWidgets.QLabel(self.centralwidget)
        self.TextWidth.setGeometry(QtCore.QRect(10, 50, 41, 31))
        self.TextWidth.setAlignment(QtCore.Qt.AlignCenter)
        self.TextWidth.setObjectName("TextWidth")
        self.TextLength = QtWidgets.QLabel(self.centralwidget)
        self.TextLength.setGeometry(QtCore.QRect(10, 80, 41, 31))
        self.TextLength.setAlignment(QtCore.Qt.AlignCenter)
        self.TextLength.setObjectName("TextLength")
        self.TextX = QtWidgets.QLabel(self.centralwidget)
        self.TextX.setGeometry(QtCore.QRect(10, 110, 41, 31))
        self.TextX.setAlignment(QtCore.Qt.AlignCenter)
        self.TextX.setObjectName("TextX")
        self.TextY = QtWidgets.QLabel(self.centralwidget)
        self.TextY.setGeometry(QtCore.QRect(10, 140, 41, 31))
        self.TextY.setAlignment(QtCore.Qt.AlignCenter)
        self.TextY.setObjectName("TextY")
        self.TextZ = QtWidgets.QLabel(self.centralwidget)
        self.TextZ.setGeometry(QtCore.QRect(10, 170, 41, 31))
        self.TextZ.setAlignment(QtCore.Qt.AlignCenter)
        self.TextZ.setObjectName("TextZ")
        self.TextYaw = QtWidgets.QLabel(self.centralwidget)
        self.TextYaw.setGeometry(QtCore.QRect(10, 200, 41, 31))
        self.TextYaw.setAlignment(QtCore.Qt.AlignCenter)
        self.TextYaw.setObjectName("TextYaw")
        self.TextBox2DXMin = QtWidgets.QLabel(self.centralwidget)
        self.TextBox2DXMin.setGeometry(QtCore.QRect(10, 230, 41, 31))
        self.TextBox2DXMin.setAlignment(QtCore.Qt.AlignCenter)
        self.TextBox2DXMin.setObjectName("TextBox2DXMin")
        self.TextBox2DXMax = QtWidgets.QLabel(self.centralwidget)
        self.TextBox2DXMax.setGeometry(QtCore.QRect(10, 260, 41, 31))
        self.TextBox2DXMax.setAlignment(QtCore.Qt.AlignCenter)
        self.TextBox2DXMax.setObjectName("TextBox2DXMax")
        self.TextBox2DYMin = QtWidgets.QLabel(self.centralwidget)
        self.TextBox2DYMin.setGeometry(QtCore.QRect(10, 290, 41, 31))
        self.TextBox2DYMin.setAlignment(QtCore.Qt.AlignCenter)
        self.TextBox2DYMin.setObjectName("TextBox2DYMin")
        self.TextBox2DYMax = QtWidgets.QLabel(self.centralwidget)
        self.TextBox2DYMax.setGeometry(QtCore.QRect(10, 320, 41, 31))
        self.TextBox2DYMax.setAlignment(QtCore.Qt.AlignCenter)
        self.TextBox2DYMax.setObjectName("TextBox2DYMax")

        # H
        self.SizeHeight = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.SizeHeight.setMaximum(1000)
        self.SizeHeight.setMinimum(-1000)
        self.SizeHeight.setGeometry(QtCore.QRect(51, 20, 211, 31))
        self.SizeHeight.setObjectName("SizeHeight")
        self.SizeHeight.valueChanged.connect(self.GetValue)
        # W       
        self.SizeWidth = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.SizeWidth.setMaximum(1000)
        self.SizeWidth.setMinimum(-1000)
        self.SizeWidth.setGeometry(QtCore.QRect(51, 50, 211, 31))
        self.SizeWidth.setObjectName("SizeWidth")
        self.SizeWidth.valueChanged.connect(self.GetValue)
        # L
        self.SizeLength = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.SizeLength.setMaximum(1000)
        self.SizeLength.setMinimum(-1000)
        self.SizeLength.setGeometry(QtCore.QRect(51, 80, 211, 31))
        self.SizeLength.setObjectName("SizeLength")
        self.SizeLength.valueChanged.connect(self.GetValue)
        # X
        self.PositionX = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.PositionX.setMaximum(1000)
        self.PositionX.setMinimum(-1000)
        self.PositionX.setGeometry(QtCore.QRect(51, 110, 211, 31))
        self.PositionX.setObjectName("PositionX")
        self.PositionX.valueChanged.connect(self.GetValue)
        # Y
        self.PositionY = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.PositionY.setMaximum(1000)
        self.PositionY.setMinimum(-1000)
        self.PositionY.setGeometry(QtCore.QRect(51, 140, 211, 31))
        self.PositionY.setObjectName("PositionY")
        self.PositionY.valueChanged.connect(self.GetValue)
        #Z
        self.PositionZ = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.PositionZ.setMaximum(1000)
        self.PositionZ.setMinimum(-1000)
        self.PositionZ.setGeometry(QtCore.QRect(51, 170, 211, 31))
        self.PositionZ.setObjectName("PositionZ")
        self.PositionZ.valueChanged.connect(self.GetValue)
        # Yaw
        self.AngleYaw = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.AngleYaw.setMaximum(np.pi)
        self.AngleYaw.setMinimum(-np.pi)
        self.AngleYaw.setGeometry(QtCore.QRect(51, 200, 211, 31))
        self.AngleYaw.setObjectName("AngleYaw")
        self.AngleYaw.valueChanged.connect(self.GetValue)
        # XMin
        self.Box2DXMin = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.Box2DXMin.setMaximum(2000)
        self.Box2DXMin.setMinimum(0)
        self.Box2DXMin.setGeometry(QtCore.QRect(51, 230, 211, 31))
        self.Box2DXMin.setObjectName("Box2DXMin")
        self.Box2DXMin.valueChanged.connect(self.GetValue)
        # YMin
        self.Box2DYMin = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.Box2DYMin.setMaximum(2000)
        self.Box2DYMin.setMinimum(0)
        self.Box2DYMin.setGeometry(QtCore.QRect(51, 290, 211, 31))
        self.Box2DYMin.setObjectName("Box2DYMin")
        self.Box2DYMin.valueChanged.connect(self.GetValue)
        # XMax
        self.Box2DXMax = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.Box2DXMax.setMaximum(2000)
        self.Box2DXMax.setMinimum(0)
        self.Box2DXMax.setGeometry(QtCore.QRect(51, 260, 211, 31))
        self.Box2DXMax.setObjectName("Box2DXMax")
        self.Box2DXMax.valueChanged.connect(self.GetValue)
        # YMax
        self.Box2DYMax = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.Box2DYMax.setMaximum(2000)
        self.Box2DYMax.setMinimum(0)
        self.Box2DYMax.setGeometry(QtCore.QRect(51, 320, 211, 31))
        self.Box2DYMax.setObjectName("Box2DYMax")
        self.Box2DYMax.valueChanged.connect(self.GetValue)
       
        # クラス
        self.ObjectClass = QtWidgets.QComboBox(self.centralwidget)
        self.ObjectClass.setGeometry(QtCore.QRect(10, 350, 252, 31))
        self.ObjectClass.setObjectName("ObjectClass")
        self.ObjectClass.addItem("")
        self.ObjectClass.addItem("")
        self.ObjectClass.addItem("")
        self.ObjectClass.addItem("")
        self.ObjectClass.addItem("")
        self.ObjectClass.addItem("")
        self.ObjectClass.currentTextChanged.connect(self.GetValue)

        # ボタン
        self.BeforeFrame = QtWidgets.QPushButton(self.centralwidget)
        self.BeforeFrame.setGeometry(QtCore.QRect(10, 400, 121, 61))
        self.BeforeFrame.setObjectName("BeforeFrame")
        self.BeforeFrame.clicked.connect(self.OpenBeforeFrame)
        self.NextFrame = QtWidgets.QPushButton(self.centralwidget)
        self.NextFrame.setGeometry(QtCore.QRect(140, 400, 121, 61))
        self.NextFrame.setObjectName("NextFrame")
        self.NextFrame.clicked.connect(self.OpenNextFrame)
        self.Before2DBox = QtWidgets.QPushButton(self.centralwidget)
        self.Before2DBox.setGeometry(QtCore.QRect(10, 490, 121, 61))
        self.Before2DBox.setObjectName("Before2DBox")
        self.Before2DBox.clicked.connect(self.ViewBeforeFrustum)
        self.Next2DBox = QtWidgets.QPushButton(self.centralwidget)
        self.Next2DBox.setGeometry(QtCore.QRect(140, 490, 121, 61))
        self.Next2DBox.setObjectName("Next2DBox")
        self.Next2DBox.clicked.connect(self.ViewNextFrustum)
        self.SaveLabel = QtWidgets.QPushButton(self.centralwidget)
        self.SaveLabel.setGeometry(QtCore.QRect(10, 580, 251, 61))
        self.SaveLabel.setObjectName("SaveLabel")
        self.SaveLabel.clicked.connect(self.SaveBoxLabel)

        # メニューバー
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
        self.OpenFile.triggered.connect(self.OpenDialog)
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
        self.TextHeight.setText(_translate("MainWindow", "H"))
        self.TextLength.setText(_translate("MainWindow", "L"))
        self.TextX.setText(_translate("MainWindow", "X"))
        self.TextY.setText(_translate("MainWindow", "Y"))
        self.TextZ.setText(_translate("MainWindow", "Z"))
        self.TextYaw.setText(_translate("MainWindow", "Yaw"))
        self.TextBox2DXMin.setText(_translate("MainWindow", "Xmin"))
        self.TextBox2DXMax.setText(_translate("MainWindow", "Xmax"))
        self.TextBox2DYMin.setText(_translate("MainWindow", "Ymin"))
        self.TextBox2DYMax.setText(_translate("MainWindow", "Ymax"))
        self.ObjectClass.setItemText(0, _translate("MainWindow", "Class"))
        self.ObjectClass.setItemText(1, _translate("MainWindow", "Car"))
        self.ObjectClass.setItemText(2, _translate("MainWindow", "Pedestrian"))
        self.ObjectClass.setItemText(3, _translate("MainWindow", "Cyclist"))
        self.ObjectClass.setItemText(4, _translate("MainWindow", "Truck"))
        self.ObjectClass.setItemText(5, _translate("MainWindow", "Van"))
        self.NextFrame.setText(_translate("MainWindow", "|▶"))
        self.BeforeFrame.setText(_translate("MainWindow", "◀|"))
        self.Before2DBox.setText(_translate("MainWindow", "◁|"))
        self.Next2DBox.setText(_translate("MainWindow", "|▷"))
        self.SaveLabel.setText(_translate("MainWindoe", "Save"))
        self.menuFile.setTitle(_translate("MainWindow", "File"))
        self.OpenFile.setText(_translate("MainWindow", "開く"))
        self.OpenFile.setShortcut(_translate("MainWindow", "Ctrl+O"))
        self.SaveFile.setText(_translate("MainWindow", "保存"))
        self.SaveFile.setShortcut(_translate("MainWindow", "Ctrl+S"))

    def OpenDialog(self):
        # 画像ファイルの選択
        image_file_name = QFileDialog.getOpenFileName(caption='Open file', directory='/home/hiroki/KITTI', filter="Image(*.png *.jpg *.jpeg)")
        # FileOperationのインスタンス作成
        self.FileOp = FileOperation(image_file_name[0])
        # frustum_numberの初期化
        self.frustum_number = 0
        # objestの作成
        self.objects = self.FileOp.read_label_file()
        self.InputValue()
         # 画像の表示
        img , pts_box2d= self.ShowImage()
        # ポイントクラウドの表示
        self.ShowPointCloud(img, pts_box2d)
        return True
        
    def ShowPointCloud(self, img, pts_box2d):
        # ウィンドウの初期化
        mlab.clf(self.fig)
        # ポイントクラウドの読み込み
        pc_velo = self.FileOp.read_pc_file()
        pc_in_image = self.FileOp.get_pc_in_image(pc_velo, img.shape[1], img.shape[0])
        # calibの読み込み
        calib = self.FileOp.calib
        # 3Dボックスの頂点の取得
        _, pts_box3d = trans_util.compute_box_3d(self.objects[self.frustum_number] , calib.P, self.box3d)
        # ポイントクラウド + フラスタム
        draw_lidar(pc_in_image, fig=self.fig)
        draw_gt_boxes3d([pts_box3d], fig=self.fig)
        draw_frustum_pc(pc_in_image, pts_box2d, calib=calib, fig=self.fig)
        return True
    
    def ShowImage(self, update=False):
        # 画像ファイルの読み込み
        img = self.FileOp.read_image_file()
        # 2DBoxの読み込み
        pts_box2d, self.max_frustum_number = self.FileOp.read_label2d_file(frustum_number=self.frustum_number,objects=self.objects)
        # 2DBoxの更新
        if update:
            pts_box2d = [self.xmin, self.ymin, self.xmax, self.ymax]
        # 画像 + 2DBox
        draw_box2d(img, pts_box2d)
        return img, pts_box2d
    
    def SaveBoxLabel(self):
        # new_labelディレクトリの生成
        self.FileOp.make_save_dir()
        # new_labelへlabelを保存
        self.FileOp.write_save_label()
        return True

    def OpenNextFrame(self):
        # FileOperationの更新
        self.FileOp = self.FileOp.update(1)
        # frustum_numberの初期化
        self.frustum_number = 0
         # objestの作成
        self.objects = self.FileOp.read_label_file()
        self.InputValue()
        # 画像の表示
        img, pts_box2d = self.ShowImage()
        # ポイントクラウドの表示
        self.ShowPointCloud(img, pts_box2d)
        return True
        
    def OpenBeforeFrame(self):
        # FileOperationの更新
        self.FileOp = self.FileOp.update(-1)
        # frustum_numberの初期化
        self.frustum_number = 0
         # objestの作成
        self.objects = self.FileOp.read_label_file()
        self.InputValue()
        # 画像の表示
        img, pts_box2d = self.ShowImage()
        # ポイントクラウドの表示
        self.ShowPointCloud(img, pts_box2d)
        return True
    
    def ViewNextFrustum(self):
        # フラスタムの更新
        self.frustum_number += 1
        # 最大値以下
        if self.frustum_number >= self.max_frustum_number:
            self.frustum_number -= 1
            return True
        # 画像の表示
        img, pts_box2d = self.ShowImage()
        # ポイントクラウドの表示
        self.ShowPointCloud(img, pts_box2d)
        return True

    def ViewBeforeFrustum(self):
        # フラスタムの更新
        self.frustum_number -= 1
        # o以上
        if self.frustum_number < 0:
            print('0')
            self.frustum_number = 0
            return True
        # 画像の表示
        img, pts_box2d = self.ShowImage()
        # ポイントクラウドの表示
        self.ShowPointCloud(img, pts_box2d)
        return True

    def GetValue(self):
        if self.ObjectClass.currentText()=="Class":
            return True
        self.type = self.ObjectClass.currentText()
        self.h = self.SizeHeight.value()
        self.w = self.SizeWidth.value()
        self.l = self.SizeLength.value()
        self.x = self.PositionX.value()
        self.y = self.PositionY.value()
        self.z = self.PositionZ.value()
        self.yaw = self.AngleYaw.value()
        self.xmin = self.Box2DXMin.value()
        self.ymin = self.Box2DYMin.value()
        self.xmax = self.Box2DXMax.value()
        self.ymax = self.Box2DYMax.value()
        self.box3d = [self.l, self.w, self.h, self.yaw, self.x, self.y, self.z]
        # 画像の表示
        img , pts_box2d = self.ShowImage(update=True)
        # ポイントクラウドの表示
        self.ShowPointCloud(img , pts_box2d)
        return True

    def InputValue(self):
        # シグナルの無効化
        self.ObjectClass.blockSignals(True)
        self.SizeHeight.blockSignals(True)
        self.SizeWidth.blockSignals(True)
        self.SizeLength.blockSignals(True)
        self.PositionX.blockSignals(True)
        self.PositionY.blockSignals(True)
        self.PositionZ.blockSignals(True)
        self.AngleYaw.blockSignals(True)
        self.Box2DXMin.blockSignals(True)
        self.Box2DYMin.blockSignals(True)
        self.Box2DXMax.blockSignals(True)
        self.Box2DYMax.blockSignals(True)
        # 値の更新
        obj = self.objects[self.frustum_number]
        self.type = obj.type
        self.h = obj.h
        self.w = obj.w
        self.l = obj.l
        self.x = obj.t[0]
        self.y = obj.t[1]
        self.z = obj.t[2]
        self.yaw = obj.ry
        self.xmin = obj.xmin
        self.ymin = obj.ymin
        self.xmax = obj.xmax
        self.ymax = obj.ymax
        self.ObjectClass.setCurrentText(self.type)
        self.SizeHeight.setValue(self.h)
        self.SizeWidth.setValue(self.w)
        self.SizeLength.setValue(self.l)
        self.PositionX.setValue(self.x)
        self.PositionY.setValue(self.y)
        self.PositionZ.setValue(self.z)
        self.AngleYaw.setValue(self.yaw)
        self.Box2DXMin.setValue(self.xmin)
        self.Box2DYMin.setValue(self.ymin)
        self.Box2DXMax.setValue(self.xmax)
        self.Box2DYMax.setValue(self.ymax)
        # シグナルの有効化
        self.ObjectClass.blockSignals(False)
        self.SizeHeight.blockSignals(False)
        self.SizeWidth.blockSignals(False)
        self.SizeLength.blockSignals(False)
        self.PositionX.blockSignals(False)
        self.PositionY.blockSignals(False)
        self.PositionZ.blockSignals(False)
        self.AngleYaw.blockSignals(False)
        self.Box2DXMin.blockSignals(False)
        self.Box2DYMin.blockSignals(False)
        self.Box2DXMax.blockSignals(False)
        self.Box2DYMax.blockSignals(False)
        return True


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

# 残骸
# pixmap = QtGui.QPixmap(self.image_file_name[0])
# self.Image = QtGui.QImage(img)
# self.Image = QtGui.QImage(self.image_file_name[0])
# self.ImageView.setPixmap(QtGui.QPixmap.fromImage(self.Image))
    # View
# self.PointCloudView = QtOpenGL(self.centralwidget)
# # self.PointCloudView = QtWidgets.QGraphicsView(self.centralwidget)
# self.PointCloudView.setGeometry(QtCore.QRect(10, 10, 711, 641))
# self.PointCloudView.setObjectName("PointCloudView")
# self.ImageView = QtWidgets.QLabel(self.centralwidget)
# self.ImageView.setGeometry(QtCore.QRect(730, 290, 531, 361))
# self.ImageView.setObjectName("ImageView")
# pixmap = QtGui.QPixmap(self.image_file_name)
# self.Image = QtGui.QImage()
# self.ImageView.setPixmap(QtGui.QPixmap.fromImage(self.Image))
# # view.addItem(self._imageItem)
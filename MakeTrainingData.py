# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'MakeTrainingData.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets, QtOpenGL
from PyQt5.QtWidgets import QFileDialog, QMessageBox, QPlainTextEdit
from PyQt5.QtCore    import pyqtSlot
import os
import cv2
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as gl
import numpy as np
from viz_util import extract_pc_in_box3d, draw_box2d, NewGLAxis, get_box3d_pts
import trans_util
from file_util import *
import datetime

def ObjectColor(obj_type, mode):
    if mode=="2d":
        if obj_type=="Car":
            color = (0, 0, 255) 
        elif obj_type=="Pedestrian":
            color = (0, 255, 255)
        elif obj_type=="Cyclist":
            color = (0, 255, 0)
        elif obj_type=="Truck":
            color = (255, 255, 0)
        elif obj_type=="Van":
            color = (255, 0, 255)
        else:
            color = (255, 255, 255)
    elif mode=="3d":
        if obj_type=="Car":
            color = (1, 0, 0, 1) 
        elif obj_type=="Pedestrian":
            color = (1, 1, 0, 1)
        elif obj_type=="Cyclist":
            color = (0, 1, 0, 1)
        elif obj_type=="Truck":
            color = (0, 1, 1, 1)
        elif obj_type=="Van":
            color = (1, 0, 1, 1)
        else:
            color = (1, 1, 1, 1)
    return color

class Ui_MainWindow(object):
    def __init__(self):
        self.frustum_number = 0
        self.before_frustum_number = -1
        self.now_file = None
        self.FileOp = None
        # self.fig = mlab.figure(figure=None, bgcolor=(0,0,0), fgcolor=None, engine=None, size=(1000, 1000))
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
        self.viewx = 0
        self.viewy = 0
        self.viewz = 0
        self.yaw = 0
        self.xmin = 0
        self.ymin = 0
        self.xmax = 0
        self.ymax = 0
        self.save_label = str(self.type) + ' ' + '-1 -10 -10 ' + \
                str(self.xmin) + ' ' + str(self.ymin) + ' ' + str(self.xmax) + ' ' + str(self.ymax) + ' ' + \
                    str(self.h) + ' ' + str(self.w) + ' ' + str(self.h) + ' ' + \
                        str(self.x) + ' ' + str(self.y) +  ' ' + str(self.z) + ' ' + \
                            str(self.yaw)
        self.overwrite = False
        self.save_log = None
        self.line_scale = np.zeros((60, 3))
        self.log_file_name = date
        self.test_mode = False

    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1110, 940)
        # MainWindow.resize(272, 970)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")

         # 文字
        self.TextHeight = QtWidgets.QLabel(self.centralwidget)
        self.TextHeight.setGeometry(QtCore.QRect(10, 20, 41, 31))
        self.TextHeight.setAlignment(QtCore.Qt.AlignCenter)
        self.TextHeight.setObjectName("TextHeight")
        self.TextWidth = QtWidgets.QLabel(self.centralwidget)
        self.TextWidth.setGeometry(QtCore.QRect(10, 55, 41, 31))
        self.TextWidth.setAlignment(QtCore.Qt.AlignCenter)
        self.TextWidth.setObjectName("TextWidth")
        self.TextLength = QtWidgets.QLabel(self.centralwidget)
        self.TextLength.setGeometry(QtCore.QRect(10, 90, 41, 31))
        self.TextLength.setAlignment(QtCore.Qt.AlignCenter)
        self.TextLength.setObjectName("TextLength")
        self.TextX = QtWidgets.QLabel(self.centralwidget)
        self.TextX.setGeometry(QtCore.QRect(10, 140, 41, 31))
        self.TextX.setAlignment(QtCore.Qt.AlignCenter)
        self.TextX.setObjectName("TextX")
        self.TextY = QtWidgets.QLabel(self.centralwidget)
        self.TextY.setGeometry(QtCore.QRect(10, 175, 41, 31))
        self.TextY.setAlignment(QtCore.Qt.AlignCenter)
        self.TextY.setObjectName("TextY")
        self.TextZ = QtWidgets.QLabel(self.centralwidget)
        self.TextZ.setGeometry(QtCore.QRect(10, 210, 41, 31))
        self.TextZ.setAlignment(QtCore.Qt.AlignCenter)
        self.TextZ.setObjectName("TextZ")
        self.TextYaw = QtWidgets.QLabel(self.centralwidget)
        self.TextYaw.setGeometry(QtCore.QRect(10, 260, 41, 31))
        self.TextYaw.setAlignment(QtCore.Qt.AlignCenter)
        self.TextYaw.setObjectName("TextYaw")
        self.TextBox2DXMin = QtWidgets.QLabel(self.centralwidget)
        self.TextBox2DXMin.setGeometry(QtCore.QRect(10, 310, 41, 31))
        self.TextBox2DXMin.setAlignment(QtCore.Qt.AlignCenter)
        self.TextBox2DXMin.setObjectName("TextBox2DXMin")
        self.TextBox2DYMin = QtWidgets.QLabel(self.centralwidget)
        self.TextBox2DYMin.setGeometry(QtCore.QRect(10, 345, 41, 31))
        self.TextBox2DYMin.setAlignment(QtCore.Qt.AlignCenter)
        self.TextBox2DYMin.setObjectName("TextBox2DYMin")
        self.TextBox2DXMax = QtWidgets.QLabel(self.centralwidget)
        self.TextBox2DXMax.setGeometry(QtCore.QRect(10, 380, 41, 31))
        self.TextBox2DXMax.setAlignment(QtCore.Qt.AlignCenter)
        self.TextBox2DXMax.setObjectName("TextBox2DXMax")    
        self.TextBox2DYMax = QtWidgets.QLabel(self.centralwidget)
        self.TextBox2DYMax.setGeometry(QtCore.QRect(10, 415, 41, 31))
        self.TextBox2DYMax.setAlignment(QtCore.Qt.AlignCenter)
        self.TextBox2DYMax.setObjectName("TextBox2DYMax")
        self.TextCheckBoxTest = QtWidgets.QLabel(self.centralwidget)
        self.TextCheckBoxTest.setGeometry(QtCore.QRect(10, 505, 41, 31))
        self.TextCheckBoxTest.setAlignment(QtCore.Qt.AlignCenter)
        self.TextCheckBoxTest.setObjectName("CheckBoxTest")
        # self.TextViewY = QtWidgets.QLabel(self.centralwidget)
        # self.TextViewY.setGeometry(QtCore.QRect(10, 500, 41, 31))
        # self.TextViewY.setAlignment(QtCore.Qt.AlignCenter)
        # self.TextViewY.setObjectName("TextViewY")
        # self.TextViewZ = QtWidgets.QLabel(self.centralwidget)
        # self.TextViewZ.setGeometry(QtCore.QRect(10, 535, 41, 31))
        # self.TextViewZ.setAlignment(QtCore.Qt.AlignCenter)
        # self.TextViewZ.setObjectName("TextViewZ")

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
        self.SizeWidth.setGeometry(QtCore.QRect(51, 55, 211, 31))
        self.SizeWidth.setObjectName("SizeWidth")
        self.SizeWidth.valueChanged.connect(self.GetValue)
        # L
        self.SizeLength = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.SizeLength.setMaximum(1000)
        self.SizeLength.setMinimum(-1000)
        self.SizeLength.setGeometry(QtCore.QRect(51, 90, 211, 31))
        self.SizeLength.setObjectName("SizeLength")
        self.SizeLength.valueChanged.connect(self.GetValue)
        # X
        self.PositionX = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.PositionX.setMaximum(1000)
        self.PositionX.setMinimum(-1000)
        self.PositionX.setGeometry(QtCore.QRect(51, 140, 211, 31))
        self.PositionX.setObjectName("PositionX")
        self.PositionX.valueChanged.connect(self.GetValue)
        # Y
        self.PositionY = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.PositionY.setMaximum(1000)
        self.PositionY.setMinimum(-1000)
        self.PositionY.setGeometry(QtCore.QRect(51, 175, 211, 31))
        self.PositionY.setObjectName("PositionY")
        self.PositionY.valueChanged.connect(self.GetValue)
        #Z
        self.PositionZ = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.PositionZ.setMaximum(1000)
        self.PositionZ.setMinimum(-1000)
        self.PositionZ.setGeometry(QtCore.QRect(51, 210, 211, 31))
        self.PositionZ.setObjectName("PositionZ")
        self.PositionZ.valueChanged.connect(self.GetValue)
        # Yaw
        self.AngleYaw = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.AngleYaw.setMaximum(np.pi)
        self.AngleYaw.setMinimum(-np.pi)
        self.AngleYaw.setGeometry(QtCore.QRect(51, 260, 211, 31))
        self.AngleYaw.setObjectName("AngleYaw")
        self.AngleYaw.valueChanged.connect(self.GetValue)
        # XMin
        self.Box2DXMin = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.Box2DXMin.setMaximum(2000)
        self.Box2DXMin.setMinimum(0)
        self.Box2DXMin.setGeometry(QtCore.QRect(51, 310, 211, 31))
        self.Box2DXMin.setObjectName("Box2DXMin")
        self.Box2DXMin.valueChanged.connect(self.GetValue)
        # YMin
        self.Box2DYMin = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.Box2DYMin.setMaximum(2000)
        self.Box2DYMin.setMinimum(0)
        self.Box2DYMin.setGeometry(QtCore.QRect(51, 345, 211, 31))
        self.Box2DYMin.setObjectName("Box2DYMin")
        self.Box2DYMin.valueChanged.connect(self.GetValue)
        # XMax
        self.Box2DXMax = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.Box2DXMax.setMaximum(2000)
        self.Box2DXMax.setMinimum(0)
        self.Box2DXMax.setGeometry(QtCore.QRect(51, 380, 211, 31))
        self.Box2DXMax.setObjectName("Box2DXMax")
        self.Box2DXMax.valueChanged.connect(self.GetValue)
        # YMax
        self.Box2DYMax = QtWidgets.QDoubleSpinBox(self.centralwidget)
        self.Box2DYMax.setMaximum(2000)
        self.Box2DYMax.setMinimum(0)
        self.Box2DYMax.setGeometry(QtCore.QRect(51, 415, 211, 31))
        self.Box2DYMax.setObjectName("Box2DYMax")
        self.Box2DYMax.valueChanged.connect(self.GetValue)
        # # viewX
        # self.ViewX = QtWidgets.QDoubleSpinBox(self.centralwidget)
        # self.ViewX.setMaximum(1000)
        # self.ViewX.setMinimum(-1000)
        # self.ViewX.setGeometry(QtCore.QRect(51, 465, 211, 31))
        # self.ViewX.setObjectName("ViewX")
        # self.ViewX.valueChanged.connect(self.GetValue)
        # # viewY
        # self.ViewY = QtWidgets.QDoubleSpinBox(self.centralwidget)
        # self.ViewY.setMaximum(1000)
        # self.ViewY.setMinimum(-1000)
        # self.ViewY.setGeometry(QtCore.QRect(51, 500, 211, 31))
        # self.ViewY.setObjectName("ViewY")
        # self.ViewY.valueChanged.connect(self.GetValue)
        # #viewZ
        # self.ViewZ = QtWidgets.QDoubleSpinBox(self.centralwidget)
        # self.ViewZ.setMaximum(1000)
        # self.ViewZ.setMinimum(-1000)
        # self.ViewZ.setGeometry(QtCore.QRect(51, 535, 211, 31))
        # self.ViewZ.setObjectName("ViewZ")
        # self.ViewZ.valueChanged.connect(self.GetValue)
        # test check box
        self.CheckBoxTest = QtWidgets.QCheckBox(self.centralwidget)
        self.CheckBoxTest.setGeometry(QtCore.QRect(100, 505, 211, 31))
        self.CheckBoxTest.setObjectName("CheckBoxTest")
        self.CheckBoxTest.clicked.connect(self.ChangeSaveMode)

       
        # クラス
        self.ObjectClass = QtWidgets.QComboBox(self.centralwidget)
        self.ObjectClass.setGeometry(QtCore.QRect(10, 585, 252, 31))
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
        self.BeforeFrame.setGeometry(QtCore.QRect(10, 635, 121, 41))
        self.BeforeFrame.setObjectName("BeforeFrame")
        self.BeforeFrame.clicked.connect(self.OpenBeforeFrame)
        self.NextFrame = QtWidgets.QPushButton(self.centralwidget)
        self.NextFrame.setGeometry(QtCore.QRect(140, 635, 121, 41))
        self.NextFrame.setObjectName("NextFrame")
        self.NextFrame.clicked.connect(self.OpenNextFrame)
        self.Before2DBox = QtWidgets.QPushButton(self.centralwidget)
        self.Before2DBox.setGeometry(QtCore.QRect(10, 685, 121, 41))
        self.Before2DBox.setObjectName("Before2DBox")
        self.Before2DBox.clicked.connect(self.ViewBeforeFrustum)
        self.Next2DBox = QtWidgets.QPushButton(self.centralwidget)
        self.Next2DBox.setGeometry(QtCore.QRect(140, 685, 121, 41))
        self.Next2DBox.setObjectName("Next2DBox")
        self.Next2DBox.clicked.connect(self.ViewNextFrustum)
        # self.SaveLabel = QtWidgets.QPushButton(self.centralwidget)
        # self.SaveLabel.setGeometry(QtCore.QRect(10, 650, 251, 61))
        # self.SaveLabel.setObjectName("SaveLabel")
        # self.SaveLabel.clicked.connect(self.SaveBoxLabel)

        # now
        self.NowFile = QPlainTextEdit(self.centralwidget)
        self.NowFile.setGeometry(QtCore.QRect(10, 735, 251, 30))
        self.NowFile.setObjectName("NowFile")

        # log
        self.SaveLog = QPlainTextEdit(self.centralwidget)
        self.SaveLog.setGeometry(QtCore.QRect(10, 770, 251, 100))
        self.SaveLog.setObjectName("SaveLog")

        # ポイントクラウド
        self.ViewPointCloud = gl.GLViewWidget(self.centralwidget)
        self.ViewPointCloud.setGeometry(QtCore.QRect(300, 20, 800, 850))
        # self.ViewPointCloud.setWindowTitle('PointCloud')
        self.ViewPointCloud.show()
        self.grid = gl.GLGridItem() 
        self.grid.setSpacing(x=1, y=1, z=0)
        self.grid.setSize(300, 300, 0)
        self.ViewPointCloud.addItem(self.grid)
        self.ViewPointCloud.opts['distance'] = 30
        self.ViewPointCloud.opts['azimuth'] = 180
        self.ViewPointCloud.opts['elevation'] = 90
        
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
        self.SaveFile.triggered.connect(self.SaveBoxLabel)
        self.NewLabelFile = QtWidgets.QAction(MainWindow)
        self.NewLabelFile.setObjectName("NewLabelFile")
        self.NewLabelFile.triggered.connect(self.MakeLabel)
        self.menuFile.addAction(self.OpenFile)
        self.menuFile.addAction(self.SaveFile)
        self.menuFile.addAction(self.NewLabelFile)
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
        self.TextCheckBoxTest.setText(_translate("MainWindow", "Test"))
        # self.TextViewY.setText(_translate("MainWindow", "viewY"))
        # self.TextViewZ.setText(_translate("MainWindow", "viewZ"))
        self.ObjectClass.setItemText(0, _translate("MainWindow", "Class"))
        self.ObjectClass.setItemText(1, _translate("MainWindow", "Car"))
        self.ObjectClass.setItemText(2, _translate("MainWindow", "Pedestrian"))
        self.ObjectClass.setItemText(3, _translate("MainWindow", "Cyclist"))
        self.ObjectClass.setItemText(4, _translate("MainWindow", "Truck"))
        self.ObjectClass.setItemText(5, _translate("MainWindow", "Van"))
        self.ObjectClass.setItemText(6, _translate("MainWindow", "Misc"))
        self.NextFrame.setText(_translate("MainWindow", "|▶"))
        self.BeforeFrame.setText(_translate("MainWindow", "◀|"))
        self.Before2DBox.setText(_translate("MainWindow", "◁|"))
        self.Next2DBox.setText(_translate("MainWindow", "|▷"))
        # self.SaveLabel.setText(_translate("MainWindoe", "Save"))
        self.menuFile.setTitle(_translate("MainWindow", "File"))
        self.OpenFile.setText(_translate("MainWindow", "開く"))
        self.OpenFile.setShortcut(_translate("MainWindow", "Ctrl+O"))
        self.SaveFile.setText(_translate("MainWindow", "保存"))
        self.SaveFile.setShortcut(_translate("MainWindow", "Ctrl+S"))
        self.NewLabelFile.setText(_translate("MainWindow", "新規ラベル作成"))
        self.NewLabelFile.setShortcut(_translate("MainWindow", "Ctrl+n"))

    def OpenDialog(self):
        # 画像ファイルの選択
        image_file_name = QFileDialog.getOpenFileName(caption='Open file',  filter="Image(*.png *.jpg *.jpeg)") #directory='/home',
        # FileOperationのインスタンス作成
        self.FileOp = FileOperation(image_file_name[0])
        # frustum_numberの初期化
        self.frustum_number = 0
         # 現在のファイル
        self.MakeNowFile()
        self.NowFile.setPlainText(self.now_file)
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
        # mlab.clf(self.fig)
        # ポイントクラウドの読み込み
        pc_velo = self.FileOp.read_pc_file()
        pc_in_image = self.FileOp.get_pc_in_image(pc_velo, img.shape[1], img.shape[0])
        # calibの読み込み
        calib = self.FileOp.calib
        # 3Dボックスの頂点の取得
        _, pts_box3d = trans_util.compute_box_3d(self.objects[self.frustum_number] , calib.P, self.box3d)
        # frustum内の点群のとインデックスの取得
        pc_in_frustum, pc_in_frustum_inds = trans_util.get_frustum_pc(pc_in_image, pts_box2d, calib)
        for loop in range(len(pc_in_frustum_inds)):
            if pc_in_frustum_inds[loop]==True:
                pc_in_frustum_inds[loop]=False
            else:
                pc_in_frustum_inds[loop]=True
        # 3Dボックス内の点群とインデックスの取得
        pc_in_box3d, pc_in_box3d_inds = extract_pc_in_box3d(pc_in_frustum, pts_box3d)
        # ポイントクラウドに画像の色を追加
        # pc_with_color = trans_util.AddColor(pc_in_image, img, calib)
        # print(pc_with_color[:10])
        # print(pc_in_box3d)
        for loop in range(len(pc_in_box3d_inds)):
            if pc_in_box3d_inds[loop]==True:
                pc_in_box3d_inds[loop] = False
            else:
                pc_in_box3d_inds[loop] = True
        # 点群の色の設定
        color = np.ones_like(pc_in_image)
        color[:, :3] = pc_in_image[:, :3]
        # 距離によって色を変化
        # dist = np.zeros((np.shape(pc_in_image)[0]))
        # for loop in range(3):
        #     for loop_loop in range(np.shape(pc_in_image)[0]):
        #         dist[loop_loop] += color[loop_loop, loop]**2
        # for loop_loop in range(np.shape(pc_in_image)[0]):
        #         dist[loop_loop] = np.sqrt(dist[loop_loop])
        # _max =  np.max(dist)
        # dist /=  _max
        # one_line = np.ones_like(dist)
        # color[:, 0] = dist**2
        # color[:, 1] = dist
        # color[:, 2] = one_line-dist
        # 反射強度によって色を変化
        intensity = pc_in_image[:, 3]
        one_line = np.ones_like(intensity)
        color[:, 0] = 0
        color[:, 1] = 0
        # color[:, 0] = intensity
        # color[:, 1] = intensity
        color[:, 2] = one_line
        # color = np.zeros((np.shape(pc_with_color[pc_in_frustum_inds])[0], 4))
        # color[:, 0] = pc_with_color[pc_in_frustum_inds, 6]/255
        # color[:, 1] = pc_with_color[pc_in_frustum_inds, 5]/255
        # color[:, 2] = pc_with_color[pc_in_frustum_inds, 4]/255
        # color[:, 3] = pc_with_color[pc_in_frustum_inds, 3]
        i = 0
        for loop in range(len(pc_in_frustum_inds)):
            if pc_in_frustum_inds[loop]==True:
                color[i, 3] = 1
                i+=1
        # サイズの固定
        size = 3
        # 表示ウィンドウの初期化
        for item in self.ViewPointCloud.items:
            self.ViewPointCloud.removeItem(item)
        self.ViewPointCloud.items = []
        self.ViewPointCloud.update()
        # グリッドの生成
        self.ViewPointCloud.opts['center'] = QtGui.QVector3D(self.viewx, self.viewy, self.viewz)
        self.grid.setSize(300, 300, 0)
        self.ViewPointCloud.addItem(self.grid)
        # 原点と座標軸の描画
        axis = NewGLAxis(size=QtGui.QVector3D(100, 100, 10))
        self.ViewPointCloud.addItem(axis)
        origin = gl.GLScatterPlotItem(pos=np.zeros((1, 3)), color=(1, 1, 1, 1), size=10, pxMode=True)
        self.ViewPointCloud.addItem(origin)
        # 10メートルごとにボールドライン
        for loop in range(0, 30, 2):
            self.line_scale[loop] = [loop*10, 150 ,0]
            self.line_scale[loop+1] = [loop*10, -150 ,0]
        gl_lines = gl.GLLinePlotItem(pos=self.line_scale, color=(1, 1, 1, 1), width=5, antialias=True, mode='lines')
        self.ViewPointCloud.addItem(gl_lines)
        # 点群の描画
        gl_pc_in_image = gl.GLScatterPlotItem(pos=pc_in_image[pc_in_frustum_inds, :3], color=color, size=size, pxMode=True)
        self.ViewPointCloud.addItem(gl_pc_in_image)
        # gl_pc_in_image = gl.GLScatterPlotItem(pos=pc_with_color[pc_in_frustum_inds, :3], color=color, size=5, pxMode=True)
        # self.ViewPointCloud.addItem(gl_pc_in_image)
        gl_pc_in_frustum = gl.GLScatterPlotItem(pos=pc_in_frustum[pc_in_box3d_inds, :3], color=(1, 1, 1, 1), size=size, pxMode=True)
        self.ViewPointCloud.addItem(gl_pc_in_frustum)
        gl_pc_in_box3d = gl.GLScatterPlotItem(pos=pc_in_box3d[:, :3], color=ObjectColor(self.type, mode="3d"), size=size, pxMode=True)
        self.ViewPointCloud.addItem(gl_pc_in_box3d)
        # 3dボックスの描画
        pts = []
        gl_pts_box3d = []
        pts = get_box3d_pts(pts_box3d)
        for loop in range(3):
            if loop==2:
                gl_pts_box3d.append(gl.GLLinePlotItem(pos=pts[loop], color=ObjectColor(self.type, mode="3d"), width=3, mode='lines'))
            else:
                gl_pts_box3d.append(gl.GLLinePlotItem(pos=pts[loop], color=ObjectColor(self.type, mode="3d"), width=3, mode='line_strip'))
            self.ViewPointCloud.addItem(gl_pts_box3d[loop])
        return True
    
    def ShowImage(self, update=False):
        # 画像ファイルの読み込み
        img = self.FileOp.read_image_file()
        # 2DBoxの読み込み
        pts_box2d, self.max_frustum_number = self.FileOp.read_label2d_file(frustum_number=self.frustum_number, objects=self.objects, calib=self.FileOp.calib, box3d=self.box3d)
        # 2DBoxの更新
        if update:
            pts_box2d = [self.xmin, self.ymin, self.xmax, self.ymax]
        # 画像 + 2DBox
        draw_box2d(img, pts_box2d, ObjectColor(self.type, mode="2d"))
        return img, pts_box2d
    
    def MakeNowFile(self):
        self.now_file = str(os.path.basename(self.FileOp.new_label_file_name)) + ' : '+ str(self.frustum_number)
        return self.now_file

    def MakeSaveLabel(self):
        self.save_label = str(self.type) + ' ' + '-1 -10 -10 ' + \
                str(self.xmin) + ' ' + str(self.ymin) + ' ' + str(self.xmax) + ' ' + str(self.ymax) + ' ' + \
                    str(self.h) + ' ' + str(self.w) + ' ' + str(self.l) + ' ' + \
                        str(self.x) + ' ' + str(self.y) +  ' ' + str(self.z) + ' ' + \
                            str(self.yaw)
        return True

    def MakeSaveLog(self):
        self.save_log = str(os.path.basename(self.FileOp.new_label_file_name)) + '\nfrustum_number : ' + \
            str(self.frustum_number) + '\nClass : ' + str(self.type) + '\n'
        return self.save_log

    def SaveBoxLabel(self):
        # new_labelディレクトリの生成
        self.FileOp.make_save_dir(test=self.test_mode)
        # save_labelの作成
        self.MakeSaveLabel()
        # frustum_numberが一致したら保存しない
        if self.before_frustum_number==self.frustum_number:
            _dialog =  QMessageBox.information(None, "Caution", "すでに保存されています", QMessageBox.Ok)
            return True
        # frustum_numberが減少したら注意
        elif self.before_frustum_number>self.frustum_number:
            _dialog =  QMessageBox.information(None, "Caution", "すでに確認した物体です\n保存しますか?", QMessageBox.No, QMessageBox.Yes)
            if _dialog==QMessageBox.No:
                return True
        # new_labelへlabelを保存
        # print(self.log_file_name)
        if self.FileOp.write_save_label(self.save_label, self.overwrite)==False:
            save_dialog = QMessageBox.information(None, "Caution", ("すでにファイルが存在します\n" + "上書きしますか?"), QMessageBox.No, QMessageBox.Yes)
            if save_dialog == QMessageBox.Yes:
                self.overwrite = True
                self.FileOp.write_save_label(self.save_label, self.overwrite, delete=True)               
                self.SaveLog.appendPlainText(self.MakeSaveLog())
                self.before_frustum_number = self.frustum_number
                self.FileOp.write_log(self.save_log, '%s.txt'%self.log_file_name)
        else:
            self.SaveLog.appendPlainText(self.MakeSaveLog())
            self.before_frustum_number = self.frustum_number
            self.FileOp.write_log(self.save_log, '%s.txt'%self.log_file_name)
        return True

    def OpenNextFrame(self):
        # overwriteの初期化
        self.overwrite = False
        # FileOperationの更新
        self.FileOp = self.FileOp.update(1)
        # frustum_numberの初期化
        self.frustum_number = 0
        self.before_frustum_number = -1
        # 現在のファイル
        self.MakeNowFile()
        self.NowFile.setPlainText(self.now_file)
        # objestの作成
        self.objects = self.FileOp.read_label_file()
        self.InputValue()
        # 画像の表示
        img, pts_box2d = self.ShowImage()
        # ポイントクラウドの表示
        self.ShowPointCloud(img, pts_box2d)
        return True
        
    def OpenBeforeFrame(self):
        # overwriteの初期化
        self.overwrite = False
        # FileOperationの更新
        self.FileOp = self.FileOp.update(-1)
        # frustum_numberの初期化
        self.frustum_number = 0
        self.before_frustum_number = -1
        # 現在のファイル
        self.MakeNowFile()
        self.NowFile.setPlainText(self.now_file)
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
        # 現在のファイル
        self.MakeNowFile()
        self.NowFile.setPlainText(self.now_file)
        # frustumの更新
        self.InputValue()
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
            self.frustum_number = 0
            return True
        # 現在のファイル
        self.MakeNowFile()
        self.NowFile.setPlainText(self.now_file)
        # frustumの更新
        self.InputValue()
        # 画像の表示
        img, pts_box2d = self.ShowImage()
        # ポイントクラウドの表示
        self.ShowPointCloud(img, pts_box2d)
        return True

    def MakeLabel(self):
        # labelファイルに追加
        with open(self.FileOp.label_file_name, 'a') as f:
            f.write('Car -1 -10 -10 0 0 100 100 2 2 2 0 0 0 0\n')
        # frustum_numberを最大値に
        self.frustum_number = self.max_frustum_number
        # max_frustum_numberを更新
        self.max_frustum_number += 1
        # objestの作成
        self.objects = self.FileOp.read_label_file()
        # 現在のファイル
        self.MakeNowFile()
        self.NowFile.setPlainText(self.now_file)
        # frustumの更新
        self.InputValue()
        # 画像の表示
        img, pts_box2d = self.ShowImage()
        # ポイントクラウドの表示
        self.ShowPointCloud(img, pts_box2d)
        return True


    def GetValue(self):
        # 初期値では行わない
        if self.ObjectClass.currentText()=="Class":
            return True
        # 値の取得
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
        # self.viewx = self.ViewX.value()
        # self.viewy = self.ViewY.value()
        # self.viewz = self.ViewZ.value()
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
        # self.ViewX.blockSignals(True)
        # self.ViewY.blockSignals(True)
        # self.ViewZ.blockSignals(True)
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
        self.box3d = [self.l, self.w, self.h, self.yaw, self.x, self.y, self.z]
        self.viewx = obj.t[0]
        self.viewy = obj.t[1]
        self.viewz = obj.t[2]
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
        # self.ViewX.setValue(self.viewx)
        # self.ViewY.setValue(self.viewy)
        # self.ViewZ.setValue(self.viewz)
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
        # self.ViewX.blockSignals(False)
        # self.ViewY.blockSignals(False)
        # self.ViewZ.blockSignals(False)
        return True

    def ChangeSaveMode(self):
        self.test_mode = True
        return True 

if __name__ == "__main__":
    import sys
    global date
    date = datetime.datetime.now().date()
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
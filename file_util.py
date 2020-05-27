# -*- coding: utf-8 -*-
import os
import sys
import glob
import numpy as np
import cv2
import trans_util

class LabelObject(object):
    def __init__(self, label_file_line):
        data = label_file_line.split(' ')
        data[1:] = [float(x) for x in data[1:]]

        # extract label, truncation, occlusion
        self.type = data[0] # 'Car', 'Pedestrian', ...
        self.truncation = data[1] # truncated pixel ratio [0..1]
        self.occlusion = int(data[2]) # 0=visible, 1=partly occluded, 2=fully occluded, 3=unknown
        self.alpha = data[3] # object observation angle [-pi..pi]

        # extract 2d bounding box in 0-based coordinates
        self.xmin = data[4] # left
        self.ymin = data[5] # top
        self.xmax = data[6] # right
        self.ymax = data[7] # bottom
        self.box2d = np.array([self.xmin,self.ymin,self.xmax,self.ymax])

        # extract 3d bounding box information
        self.h = data[8] # box height
        self.w = data[9] # box width
        self.l = data[10] # box length (in meters)
        self.t = (data[11],data[12],data[13]) # location (x,y,z) in camera coord.
        self.ry = data[14] # yaw angle (around Y-axis in camera coordinates) [-pi..pi]

class FileOperation(object):
    def __init__(self, image_file_name):
        self.image_file_name        = image_file_name
        # 画像の拡張子の取得
        self.image_ext              = os.path.splitext(os.path.basename(self.image_file_name))[1]
        # データディレクトリの取得
        self.image_dir_name         = os.path.dirname(self.image_file_name)
        self.data_dir_name          = os.path.dirname(self.image_dir_name)
        self.pointcloud_dir_name    = os.path.join(self.data_dir_name, 'PointCloud')
        # self.label2d_dir_name       = os.path.join(self.data_dir_name, 'label2d')
        self.label_dir_name         = os.path.join(self.data_dir_name, 'label')
        self.calib_dir_name         = os.path.join(self.data_dir_name, 'calib')
        # ディレクトリ内のファイルのリストの取得
        self.image_file_list        = sorted(glob.glob(os.path.join(self.image_dir_name, '*')))
        self.pointcloud_file_list   = sorted(glob.glob(os.path.join(self.pointcloud_dir_name, '*')))
        self.label_file_list        = sorted(glob.glob(os.path.join(self.label_dir_name, '*')))
        # self.label2d_file_list      = sorted(glob.glob(os.path.join(self.label2d_dir_name, '*')))
        for i in range(len(self.image_file_list)):
            if os.path.basename(self.image_file_name)==os.path.basename(self.image_file_list[i]):
                self.image_file_idx = i
        # パケット時間に変更
        self.image_number_list = [i for i in range(len(self.image_file_list))]
        self.pointcloud_number_list = [i for i in range(len(self.pointcloud_file_list))]
        self.label_number_list = [i for i in range(len(self.label_file_list))]
        # self.label2d_number_list = [i for i in range(len(self.label2d_file_list))]
        for loop in range(len(self.image_file_list)):
            _file = os.path.splitext(os.path.basename(self.image_file_list[loop]))[0]
            self.image_number_list[loop] = int(_file[-12:])
        for loop in range(len(self.pointcloud_file_list)):
            _file = os.path.splitext(self.pointcloud_file_list[loop])[0]
            self.pointcloud_number_list[loop] = int(_file[-12:])
        for loop in range(len(self.label_file_list)):
            _file = os.path.splitext(self.label_file_list[loop])[0]
            self.label_number_list[loop] = int(_file[-12:])
        # for loop in range(len(self.label2d_file_list)):
        #     _file = os.path.splitext(self.label2d_file_list[loop])[0]
        #     self.label2d_number_list[loop] = int(_file[-12:])   
        # 各ファイル名の取得
        # self.base_file_name         = os.path.splitext(os.path.basename(self.image_file_name))[0]
        # self.base_file_number       = int(self.base_file_name)
        self.base_file_number         = self.image_number_list[self.image_file_idx]
        #　画像の時間に最も近いポイントクラウドおよびラベルを持ってくる
        diff = 100
        for loop in range(len(self.pointcloud_number_list)):
            pc_num  =  self.pointcloud_number_list[loop]
            _diff   = abs(pc_num - self.base_file_number) 
            if _diff<diff:
                diff = _diff
                self.pointcloud_file_name = self.pointcloud_file_list[loop]
                self.pointcloud_file_idx  = loop
        for loop in range(len(self.label_number_list)):
            if self.label_number_list[loop]==self.pointcloud_number_list[self.pointcloud_file_idx]:
                self.label_file_name = self.label_file_list[loop]
                break
            else:
                self.label_file_name = None
        if self.label_file_name==None:
            self.label_file_name = os.path.join(self.label_dir_name, os.path.splitext(os.path.basename(self.pointcloud_file_name))[0])+".txt"
            with open(self.label_file_name, 'w') as f:
                label_tmp = 'Misc ' + '-1 -10 -10 ' + '0 0 30 30 ' + '1 1 1 ' + '0 0 0 0\n'
                f.write(label_tmp)
        # for loop in range(len(self.label2d_number_list)):
        #     if self.label2d_number_list[loop]==self.pointcloud_number_list[self.pointcloud_file_idx]:
        #         self. label2d_file_name = self.label2d_file_list[loop]
        #         break
        # labelとcalibのファイル名は一致
        self.calib_file_name        = os.path.join(self.calib_dir_name, os.path.basename(self.label_file_name))
        self.calib                  = trans_util.Calibration(self.calib_file_name)
        self.new_label_dir_name     = os.path.join(self.data_dir_name, 'new_label')
        self.new_label_file_name    = os.path.join(self.new_label_dir_name, self.label_file_name)
        # self.pointcloud_file_name   = os.path.join(self.pointcloud_dir_name, self.base_file_name+ '.bin')
        # self.label2d_file_name      = os.path.join(self.label2d_dir_name, self.base_file_name+'.txt')
        # self.label_file_name        = os.path.join(self.label_dir_name, self.base_file_name+'.txt')
        
    def read_image_file(self):
        img = cv2.imread(self.image_file_name)
        return img
    
    def read_label_file(self):
        if os.path.exists(self.label_file_name)==False:
            return False
        lines = [line.rstrip() for line in open(self.label_file_name)]
        objects = [LabelObject(data) for data in lines]
        return objects

    def read_label2d_file(self, frustum_number, objects, calib=None, box3d=None):
        if objects[frustum_number].box2d[0]!=-100:
            num = len(objects)
            pts_box2d = objects[frustum_number].box2d
        else:
            # ３Dボックスを画像に投影し、２Dボックスを生成
            _, pts_box3d = trans_util.compute_box_3d(objects[frustum_number], calib.P, box3d)
            image_pts_box3d = calib.project_velo_to_image(pts_box3d)
            xmin = np.min(image_pts_box3d[:, 0])
            xmax = np.max(image_pts_box3d[:, 0])
            ymin = np.min(image_pts_box3d[:, 1])
            ymax = np.max(image_pts_box3d[:, 1])
            pts_box2d = [xmin, ymin, xmax, ymax]
            num = len(objects)
            # lines           = [line.rstrip() for line in open(self.label2d_file_name)]
            # list_pts_box2d  = [line.split() for line in lines]
            # num             = len(list_pts_box2d)
            # pts_box2d       = [float(pts) for pts in list_pts_box2d[frustum_number]]
        return pts_box2d, num
    
    def read_pc_file(self):
        pc = np.fromfile(self.pointcloud_file_name, dtype=np.float32)
        pc = pc.reshape((-1, 4))
        return pc

    def get_pc_in_image(self, pc, image_width, image_height):
        pc_rect     = self.calib.project_velo_to_rect(pc[:, :3])
        pc2d        = self.calib.project_velo_to_image(pc[:, :3])
        fov_inds    = (pc2d[:, 0] > 0) & (pc2d[:, 1] > 0) &\
            (pc2d[:, 0] < image_width) & (pc2d[:, 1] < image_height) & (pc_rect[:, 2]>0)
        pc_in_image = pc[fov_inds, :]
        return pc_in_image

    def get_frustum(self, pts_box2d):
        pts_box2d_frustum = np.ones((4, 3))
        loop = 0
        for x_loop in range(0, 3, 2):
            for y_loop in range(1, 4, 2):
                pts_box2d_frustum[loop, 0] = pts_box2d[x_loop]
                pts_box2d_frustum[loop, 1] = pts_box2d[y_loop]
                loop += 1
        return pts_box2d_frustum
    
    def write_log(self, log, log_file_name):
        with open(os.path.join(self.data_dir_name, log_file_name),"a") as f:
            f.write('%s\n'%log)
        return True

    def make_save_dir(self):
        if os.path.exists(self.new_label_dir_name)==False:
            os.mkdir(self.new_label_dir_name)        
        return True
    
    def write_save_label(self, labels, overwrite=False, delete=False):
        if (os.path.exists(self.new_label_file_name)==True) and (overwrite==False):
            return False
        if delete==True:
            os.remove(self.new_label_file_name)
        with open(self.new_label_file_name, "a") as f:
            f.write('%s\n'%labels)
        return True

    def update(self, num):
        # base_numberの追加
        # self.base_file_number       += num
        # if self.base_file_number<0:
        #     self.base_file_number   = 0
        # image_file_idxの更新
        self.image_file_idx         += num
        if self.image_file_idx<0:
            self.image_file_idx     = 0
        # ファイル名の更新
        self.image_file_name        = self.image_file_list[self.image_file_idx]
        image_file_name             = self.image_file_name
        # self.base_file_name         = '%06d'%self.base_file_number
        # self.image_file_name        = os.path.join(self.image_dir_name, self.base_file_name + self.image_ext)
        # image_file_name             = self.image_file_name
        return FileOperation(image_file_name)
        
if __name__=="__main__":
    print("OK")
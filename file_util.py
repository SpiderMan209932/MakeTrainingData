# -*- coding: utf-8 -*-
import os
import sys
import glob
import numpy as np
import cv2
import trans_util

class FileOperation(object):
    def __init__(self, image_file_name):
        self.image_file_name        = image_file_name
        # 画像の拡張子の取得
        self.image_ext              = os.path.splitext(os.path.basename(self.image_file_name))[1]
        # データディレクトリの取得
        self.image_dir_name         = os.path.dirname(self.image_file_name)
        self.data_dir_name          = os.path.dirname(self.image_dir_name)
        self.pointcloud_dir_name    = os.path.join(self.data_dir_name, 'PointCloud')
        self.label2d_dir_name       = os.path.join(self.data_dir_name, 'label2d')
        self.calib_dir_name         = os.path.join(self.data_dir_name, 'calib')
        # 各ファイル名の取得
        self.base_file_name         = os.path.splitext(os.path.basename(self.image_file_name))[0]
        self.base_file_number       = int(self.base_file_name)
        self.pointcloud_file_name   = os.path.join(self.pointcloud_dir_name, self.base_file_name+ '.bin')
        self.label2d_file_name      = os.path.join(self.label2d_dir_name, self.base_file_name+'.txt')
        self.calib_file_name        = os.path.join(self.calib_dir_name, self.base_file_name+'.txt')
        self.calib                  = trans_util.Calibration(self.calib_file_name)
        self.new_label_file_name    = os.path.join(self.data_dir_name, 'new_label', self.base_file_name+'.txt')

    def read_image_file(self):
        img = cv2.imread(self.image_file_name)
        return img
    
    def read_label_file(self):
        if os.path.exist(self.labal_file_name)==False:
            return False
        lines = [line.rstrip() for line in open(self.label2d_file_name)]
        datas  = [line.split() for line in lines]
        objects = [0 for _ in range(datas)]
        for loop in range(len(datas)):
            data = datas[loop].split(' ')
            data  = [float(x) for x in data[1:]]
            objects[loop] = { 'type':data[0], 'box2d':(data[4] ,data[5], data[6], data[7]), 'center':(data[11], data[12], data[13]),
            'size':(data[8], data[9], data[10]), 'yaw':(data[14])}   
        return objects

    def read_label2d_file(self, frustum_number):
        lines           = [line.rstrip() for line in open(self.label2d_file_name)]
        list_pts_box2d  = [line.split() for line in lines]
        num             = len(list_pts_box2d)
        pts_box2d       = [float(pts) for pts in list_pts_box2d[frustum_number]]
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

    def make_save_dir(self):
        if os.path.exists(self.new_label_file_name)==False:
            os.mkdir(self.new_label_file_name)        
        return True
    
    def write_save_label(self, labels):
        with open(self.new_label_file_name, "a") as f:
            for label in labels:
                if label.isdigit()==False:
                    f.write('%s '%label)
                else :
                    f.write('%.6f '%label)
                    if label==labels[15]:
                        f.write('\n')
        return True
    

    def update(self, num):
        # base_numberの追加
        self.base_file_number       += num
        # ファイル名の更新
        self.base_file_name         = '%06d'%self.base_file_number
        self.image_file_name        = os.path.join(self.image_dir_name, self.base_file_name + self.image_ext)
        image_file_name             = self.image_file_name
        return FileOperation(image_file_name)
        
if __name__=="__main__":
    print("OK")
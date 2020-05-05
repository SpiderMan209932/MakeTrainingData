''' Visualization code for point clouds and 3D bounding boxes with mayavi.

Modified by Charles R. Qi
Date: September 2017

Ref: https://github.com/hengck23/didi-udacity-2017/blob/master/baseline-04/kitti_data/draw.py
'''

import numpy as np
# import mayavi.mlab as mlab
import cv2
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as gl
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

try:
    raw_input          # Python 2
except NameError:
    raw_input = input  # Python 3

class NewGLAxis(gl.GLAxisItem):
    def paint(self):
        self.setupGLState()
        
        if self.antialias:
            glEnable(GL_LINE_SMOOTH)
            glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
            
        glLineWidth(10)

        glBegin( GL_LINES )        

        x,y,z = self.size()
        glColor4f( 1, 1, 0, 1)  # z is yellow
        glVertex3f(0, 0, 0)
        glVertex3f(0, 0, z)

        glColor4f(0, 1, 0, 1)  # y is green
        glVertex3f(0, 0, 0)
        glVertex3f(0, y, 0)

        glColor4f(1, 0, 0, 1)  # x is red
        glVertex3f(0, 0, 0)
        glVertex3f(x, 0, 0)
        glEnd()

def in_hull(p, hull):
    from scipy.spatial import Delaunay
    if not isinstance(hull,Delaunay):
        hull = Delaunay(hull)
    return hull.find_simplex(p)>=0

def extract_pc_in_box3d(pc, box3d):
    ''' pc: (N,3), box3d: (8,3) '''
    box3d_roi_inds = in_hull(pc[:,0:3], box3d)
    return pc[box3d_roi_inds,:], box3d_roi_inds

def draw_frustum_pc(pc, pts_2d, calib, fig):    
    pc2d = calib.project_velo_to_image(pc[:, :3])
    box2d_inds = (pc2d[:, 0] < pts_2d[2]) &\
        (pc2d[:, 0] > pts_2d[0]) &\
            (pc2d[:, 1] < pts_2d[3]) &\
                (pc2d[:, 1] > pts_2d[1])
    pc_in_box2d = pc[box2d_inds, :]
    mlab.points3d(pc_in_box2d[:,0], pc_in_box2d[:,1], pc_in_box2d[:,2], color=(1, 1, 1), mode='point', scale_factor=1, figure=fig)
    return fig

def draw_box2d(img, pts_box2d, color):
    cv2.rectangle(img, (int(pts_box2d[0]), int(pts_box2d[1])), (int(pts_box2d[2]), int(pts_box2d[3])), color, thickness=2)
    # cv2.rectangle(img, (pts_box2d[0], pts_box2d[1]), (pts_box2d[2], pts_box2d[3]), (0, 0, 255), thickness=2)
    cv2.imshow("image", img)
    return True

def draw_box3d_pc(pc, pts_3d, color, fig):
    pc_in_box3d, _ = extract_pc_in_box3d(pc, pts_3d)
    mlab.points3d(pc_in_box3d[:,0], pc_in_box3d[:,1], pc_in_box3d[:,2], color=color, mode='point', scale_factor=1, figure=fig)
    return fig

def get_box3d_pts(pts_box3d):
    pts_box3d_top = np.zeros((5, 3))
    pts_box3d_piller = np.zeros((8, 3))
    pts_box3d_bottom = np.zeros((5, 3))
    l = 0
    for k in range(0,4):
        # top
        pts_box3d_top[k] = pts_box3d[k]
        # aspect
        pts_box3d_piller[l] = pts_box3d[k]
        pts_box3d_piller[l+1] = pts_box3d[k+4]
        # bottom
        pts_box3d_bottom[k] = pts_box3d[k+4] 
        l += 2
    pts_box3d_top[4] = pts_box3d_top[0]
    pts_box3d_bottom[4] = pts_box3d_bottom[0]
    return pts_box3d_top, pts_box3d_bottom, pts_box3d_piller

def draw_lidar_simple(pc, color=None):
    ''' Draw lidar points. simplest set up. '''
    fig = mlab.figure(figure=None, bgcolor=(0,0,0), fgcolor=None, engine=None, size=(1600, 1000))
    if color is None: color = pc[:,2]
    #draw points
    mlab.points3d(pc[:,0], pc[:,1], pc[:,2], color, color=None, mode='point', colormap = 'gnuplot', scale_factor=1, figure=fig)
    #draw origin
    mlab.points3d(0, 0, 0, color=(1,1,1), mode='sphere', scale_factor=0.2)
    #draw axis
    axes=np.array([
        [2.,0.,0.,0.],
        [0.,2.,0.,0.],
        [0.,0.,2.,0.],
    ],dtype=np.float64)
    mlab.plot3d([0, axes[0,0]], [0, axes[0,1]], [0, axes[0,2]], color=(1,0,0), tube_radius=None, figure=fig)
    mlab.plot3d([0, axes[1,0]], [0, axes[1,1]], [0, axes[1,2]], color=(0,1,0), tube_radius=None, figure=fig)
    mlab.plot3d([0, axes[2,0]], [0, axes[2,1]], [0, axes[2,2]], color=(0,0,1), tube_radius=None, figure=fig)
    mlab.view(azimuth=180, elevation=70, focalpoint=[ 12.0909996 , -1.04700089, -2.03249991], distance=62.0, figure=fig)
    return fig

def draw_lidar(pc, color=None, fig=None, bgcolor=(0,0,0), pts_scale=1, pts_mode='point', pts_color=None):
    ''' Draw lidar points
    Args:
        pc: numpy array (n,3) of XYZ
        color: numpy array (n) of intensity or whatever
        fig: mayavi figure handler, if None create new one otherwise will use it
    Returns:
        fig: created or used fig
    '''
    if fig is None: fig = mlab.figure(figure=None, bgcolor=bgcolor, fgcolor=None, engine=None, size=(1600, 1000))
    if color is None: color = pc[:,2]
    mlab.points3d(pc[:,0], pc[:,1], pc[:,2], color, color=pts_color, mode=pts_mode, colormap = 'gnuplot', scale_factor=pts_scale, figure=fig)
    
    #draw origin
    mlab.points3d(0, 0, 0, color=(1,1,1), mode='sphere', scale_factor=0.2)
    
    #draw axis
    axes=np.array([
        [2.,0.,0.,0.],
        [0.,2.,0.,0.],
        [0.,0.,2.,0.],
    ],dtype=np.float64)
    mlab.plot3d([0, axes[0,0]], [0, axes[0,1]], [0, axes[0,2]], color=(1,0,0), tube_radius=None, figure=fig)
    mlab.plot3d([0, axes[1,0]], [0, axes[1,1]], [0, axes[1,2]], color=(0,1,0), tube_radius=None, figure=fig)
    mlab.plot3d([0, axes[2,0]], [0, axes[2,1]], [0, axes[2,2]], color=(0,0,1), tube_radius=None, figure=fig)

    # draw fov (todo: update to real sensor spec.)
    fov=np.array([  # 45 degree
        [20., 20., 0.,0.],
        [20.,-20., 0.,0.],
    ],dtype=np.float64)
    
    mlab.plot3d([0, fov[0,0]], [0, fov[0,1]], [0, fov[0,2]], color=(1,1,1), tube_radius=None, line_width=1, figure=fig)
    mlab.plot3d([0, fov[1,0]], [0, fov[1,1]], [0, fov[1,2]], color=(1,1,1), tube_radius=None, line_width=1, figure=fig)
   
    # draw square region
    TOP_Y_MIN=-20
    TOP_Y_MAX=20
    TOP_X_MIN=0
    TOP_X_MAX=40
    TOP_Z_MIN=-2.0
    TOP_Z_MAX=0.4
    
    x1 = TOP_X_MIN
    x2 = TOP_X_MAX
    y1 = TOP_Y_MIN
    y2 = TOP_Y_MAX
    mlab.plot3d([x1, x1], [y1, y2], [0,0], color=(0.5,0.5,0.5), tube_radius=0.1, line_width=1, figure=fig)
    mlab.plot3d([x2, x2], [y1, y2], [0,0], color=(0.5,0.5,0.5), tube_radius=0.1, line_width=1, figure=fig)
    mlab.plot3d([x1, x2], [y1, y1], [0,0], color=(0.5,0.5,0.5), tube_radius=0.1, line_width=1, figure=fig)
    mlab.plot3d([x1, x2], [y2, y2], [0,0], color=(0.5,0.5,0.5), tube_radius=0.1, line_width=1, figure=fig)
    
    #mlab.orientation_axes()
    mlab.view(azimuth=180, elevation=70, focalpoint=[ 12.0909996 , -1.04700089, -2.03249991], distance=62.0, figure=fig)
    return fig

def draw_gt_boxes3d(gt_boxes3d, fig, obj_type, color=(1,1,1), line_width=1, draw_text=True, text_scale=(1,1,1), color_list=None):
    ''' Draw 3D bounding boxes
    Args:
        gt_boxes3d: numpy array (n,8,3) for XYZs of the box corners
        fig: mayavi figure handler
        color: RGB value tuple in range (0,1), box line color
        line_width: box line width
        draw_text: boolean, if true, write box indices beside boxes
        text_scale: three number tuple
        color_list: a list of RGB tuple, if not None, overwrite color.
    Returns:
        fig: updated fig
    ''' 
    num = len(gt_boxes3d)
    for n in range(num):
        b = gt_boxes3d[n]
        if color_list is not None:
            color = color_list[n] 
        mlab.text3d(b[4,0], b[4,1], b[4,2], '%s'%obj_type, scale=text_scale, color=color, figure=fig)
        # if draw_text: mlab.text3d(b[4,0], b[4,1], b[4,2], '%d'%n, scale=text_scale, color=color, figure=fig)
        for k in range(0,4):
            #http://docs.enthought.com/mayavi/mayavi/auto/mlab_helper_functions.html
            i,j=k,(k+1)%4
            mlab.plot3d([b[i,0], b[j,0]], [b[i,1], b[j,1]], [b[i,2], b[j,2]], color=color, tube_radius=None, line_width=line_width, figure=fig)

            i,j=k+4,(k+1)%4 + 4
            mlab.plot3d([b[i,0], b[j,0]], [b[i,1], b[j,1]], [b[i,2], b[j,2]], color=color, tube_radius=None, line_width=line_width, figure=fig)

            i,j=k,k+4
            mlab.plot3d([b[i,0], b[j,0]], [b[i,1], b[j,1]], [b[i,2], b[j,2]], color=color, tube_radius=None, line_width=line_width, figure=fig)
    #mlab.show(1)
    #mlab.view(azimuth=180, elevation=70, focalpoint=[ 12.0909996 , -1.04700089, -2.03249991], distance=62.0, figure=fig)
    return fig


if __name__=='__main__':
    pc = np.loadtxt('mayavi/kitti_sample_scan.txt')
    fig = draw_lidar(pc)
    mlab.savefig('pc_view.jpg', figure=fig)
    raw_input()

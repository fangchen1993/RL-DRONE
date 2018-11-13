"""
author: Russel FC
email: 2655463370@qq.com
license: BSD
Please feel free to use and modify this, but keep the above information. Thanks!
"""
# -*- coding:utf-8
import numpy as np
import math
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as Axes3D
import sys
import rotation

class GUI():
    # 'quad_list' is a dictionary of format: quad_list = {'quad_1_name':{'position':quad_1_position,'orientation':quad_1_orientation,'arm_span':quad_1_arm_span}, ...}
    def __init__(self, quads):
        self.quads = quads
        self.fig = plt.figure()
        #self.fig1 = plt.figure()#Angular Velocity
        #self.fig2 = plt.figure()#Angular Displacement
        self.ax = Axes3D.Axes3D(self.fig)
        self.ax.set_xlim3d([-4.0, 4.0])
        self.ax.set_xlabel('X')
        self.ax.set_ylim3d([-4.0, 4.0])
        self.ax.set_ylabel('Y')
        self.ax.set_zlim3d([0, 5.0])
        self.ax.set_zlabel('Z')
        self.ax.set_title('Quadcopter Simulation')
        self.init_plot()
        self.fig.canvas.mpl_connect('key_press_event', self.keypress_routine)#返回按键被按下事件

    # def rotation_matrix(self,angles):#旋转矩阵
    #     ct = math.cos(angles[0])
    #     cp = math.cos(angles[1])
    #     cg = math.cos(angles[2])
    #     st = math.sin(angles[0])
    #     sp = math.sin(angles[1])
    #     sg = math.sin(angles[2])
    #     R_x = np.array([[1,0,0],[0,ct,-st],[0,st,ct]])
    #     R_y = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    #     R_z = np.array([[cg,-sg,0],[sg,cg,0],[0,0,1]])
    #     R = np.dot(R_z, np.dot( R_y, R_x ))
    #     return R

    def init_plot(self):#此处为画出我们的无人机,两条直线加一个点(两点成一直线)

        self.quads['l1'], = self.ax.plot([],[],[],color='blue',linewidth=3,antialiased=False)
        self.quads['l2'], = self.ax.plot([],[],[],color='red',linewidth=3,antialiased=False)
        self.quads['hub'], = self.ax.plot([],[],[],marker='o',color='green', markersize=6,antialiased=False)

    def update(self):

        R = rotation.rotation_DCM(self.quads['orientation'])
        L = self.quads['L']
        points = np.mat([ [-L,0,0], [L,0,0], [0,-L,0], [0,L,0], [0,0,0], [0,0,0] ]).T
        points = np.dot(R,points)
        points[0,:] += self.quads['position'][0,0]
        points[1,:] += self.quads['position'][0,1]
        points[2,:] += self.quads['position'][0,2]
        points = np.array(points)#这里转成array类型是为了后面的set_dat需要是以为数据,若是mat则会报错,参见我的博客:https://mp.csdn.net/mdeditor/80475947
        self.quads['l1'].set_data(points[0,0:2],points[1,0:2])
        self.quads['l1'].set_3d_properties(points[2,0:2])
        self.quads['l2'].set_data(points[0,2:4],points[1,2:4])
        self.quads['l2'].set_3d_properties(points[2,2:4])
        self.quads['hub'].set_data(points[0,5],points[1,5])
        self.quads['hub'].set_3d_properties(points[2,5])
        plt.pause(0.00001)
        #plt.show()

    def keypress_routine(self,event):
        sys.stdout.flush()
        if event.key == 'x':
            y = list(self.ax.get_ylim3d())
            y[0] += 0.2
            y[1] += 0.2
            self.ax.set_ylim3d(y)
        elif event.key == 'w':
            y = list(self.ax.get_ylim3d())
            y[0] -= 0.2
            y[1] -= 0.2
            self.ax.set_ylim3d(y)
        elif event.key == 'd':
            x = list(self.ax.get_xlim3d())
            x[0] += 0.2
            x[1] += 0.2
            self.ax.set_xlim3d(x)
        elif event.key == 'a':
            x = list(self.ax.get_xlim3d())
            x[0] -= 0.2
            x[1] -= 0.2
            self.ax.set_xlim3d(x)

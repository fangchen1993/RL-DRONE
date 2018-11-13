"""
author: Russel FC
email: 2655463370@qq.com
license: BSD
Please feel free to use and modify this, but keep the above information. Thanks!
This file is for rotation
INPUT:Angle[roll,pitch,yaw]
OUTPUT:The DCM matrix R(b_e)
"""
# -*- coding:utf-8
import  numpy as np
import math
def rotation_DCM(Angle):#array也可以进行乘法
        c_roll,s_roll,c_pitch,s_pitch,c_yaw,s_yaw = Trigonometric(Angle)
        R_x=np.mat([[1,0,0],[0,c_roll,s_roll],[0,-s_roll,c_roll]])
        R_y=np.mat([[c_pitch,0,-s_pitch],[0,1,0],[s_pitch,0,c_pitch]])
        R_z=np.mat([[c_yaw,s_yaw,0],[s_yaw,c_yaw,0],[0,0,1]])
        R_e_b = np.dot(np.dot(R_x,R_y),R_z)
        R_b_e = R_e_b.T
        return R_b_e



def Quaternion(self):#TODO
    pass
def Trigonometric(Angle):#[roll,pitch,yaw]
    c_roll = math.cos(Angle[0,0])
    s_roll = math.sin(Angle[0,0])
    c_pitch = math.cos(Angle[0,1])
    s_pitch = math.sin(Angle[0,1])
    c_yaw = math.cos(Angle[0,2])
    s_yaw = math.sin(Angle[0,2])
    return c_roll,s_roll,c_pitch,s_pitch,c_yaw,s_yaw



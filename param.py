"""
author: Russel FC
email: 2655463370@qq.com
license: BSD
Please feel free to use and modify this, but keep the above information. Thanks!
"""
# -*- coding: utf-8 -*-
import numpy as np

# """
# g:重力加速度
# m:飞机质量
# L:螺旋桨到飞机重心距离
# k:推力系数,推力T和电机转速omega的平方成正比
# b:扭矩系数
# I:惯性矩(inertia matrix)
# kd:阻力FD系数
# """
Control_Params={'name':'PID','value':[0.4,0.2,0.1]}

quadcopter_params={'position':np.mat([[1,0,1]]),
                   'orientation':np.mat([[0,0,0]]),
                   'm':3.1,'g':9.81,'L':0.23,
                   'k':0.00029248,'b':1e-7,
                   'I':np.mat([[0.00021,0,0],[0,0.00021,0],[0,0,0.0000705]]),
                   'kd':0.25,
                   }
#3-6-1的网络 setpoint,rate,rate_prev
BP_params={'eta':0.05,
           'alpha':0.05,
           'w1roll':np.mat(np.random.random((2,6))),
           'w2roll':np.mat(np.random.random((6,1))),
           'w1pitch':np.mat(np.random.random((2,6))),
           'w2pitch':np.mat(np.random.random((6,1))),
           'w1yaw':np.mat(np.random.random((2,6))),
           'w2yaw':np.mat(np.random.random((6,1)))
           }

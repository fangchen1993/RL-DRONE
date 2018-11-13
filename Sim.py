#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    :  2018/5
# @Author  : FC
# @Site    : 2655463370@qq.com
# @license : BSD

###
#程序的整个原理是在平稳状态(roll,pitch,yaw为0)下,无人机受到一个扰动,最终稳定到初始状态的过程
###
import param,rotation,gui
import numpy as np
import math
import QuadcopterFunction
import Controller
import sys
import signal

PI = np.pi
#Note:在最终的显示中,theta的单位是度,在计算过程中为了求角速度omega用的都是弧度
#所有的计算都是矩阵形式,主要由于array和matrix的乘法是不同的,参见我的`博客:https://mp.csdn.net/mdeditor/80475947

def esc(signum,frame):
    print('Stop the program')
    _task_should_exit =1

    sys.exit()
def simulate(*args):
    """
    :params:dt
    """
    signal.signal(signal.SIGINT,esc)
    signal.signal(signal.SIGTERM, esc)
    quadcopter_params = param.quadcopter_params
    controller_params = param.Control_Params
    if len(args)<1 :
        dt = 0.05
    else:
        dt = args[0]

    #Initial system
    x=quadcopter_params['position']
    xdot = np.mat(np.zeros((3,1)))
    theta = quadcopter_params['orientation']

    #Set the Disturbance thetadot
    thetadot = np.mat(np.random.rand(3,1))*100/180#这里的radians是为了把角度转弧度,因为角速度是用弧度定义的

    #GUI的句柄
    quad_gui = gui.GUI(quadcopter_params)

#Start the task
    while not _task_should_exit:
        #数据可视化
        quad_gui.update()
        # 初始化输入和参数更新,实际上后面的计算只需要i,而返回controller_parms是为了积分项始终计算
        i, controller_params = Controller.controller(controller_params, quadcopter_params,thetadot,dt)
        # 计算角速度,加速度,角加速度
        omega = QuadcopterFunction.thetadot2omega(thetadot, theta)
        a = QuadcopterFunction.acceleration(i, theta, xdot,quadcopter_params)
        omegadot = QuadcopterFunction.angular_acceleration(i, omega, quadcopter_params)

        # 更新系统
        omega = omega + dt * omegadot
        thetadot = QuadcopterFunction.omega2thetadot(omega, theta)
        theta = theta + dt * thetadot
        xdot = xdot + dt * a
        x = x + dt * xdot
        #更新quadcopters的'position'和'orientation'
        #Note:把theta的单位转换为度
        quadcopter_params['position'] = x
        quadcopter_params['orientation'] = theta*180/PI



if '__main__':
    _task_should_exit = 0
    dt = 0.0001
    simulate(dt)#TODO如何读取终端的输入dt,如何ctrl+C 结束程序
















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
import BP
from matplotlib import pyplot as plt
from matplotlib.ticker import MultipleLocator, FormatStrFormatter


omegadata=[]
thetadotdata=[]
omegadataBP=[]
PI = np.pi
loop = 10000
mu=0.01#训练误差

#Note:在最终的显示中,theta的单位是度,在计算过程中为了求角速度omega用的都是弧度
#所有的计算都是矩阵形式,主要由于array和matrix的乘法是不同的,参见我的`博客:https://mp.csdn.net/mdeditor/80475947


def simulate(*args):
    """
    :params:dt
    """

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

#Start the task
    for i in range(loop):
        # 初始化输入和参数更新,实际上后面的计算只需要i,而返回controller_parms是为了积分项始终计算
        i, controller_params = Controller.controller(controller_params, quadcopter_params,thetadot,dt)
        # 计算角速度,加速度,角加速度
        omega = QuadcopterFunction.thetadot2omega(thetadot, theta)
        omegadata.append(omega)
        thetadotdata.append(thetadot)

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

def BPfunction():
    #[u,y]=[thetadotdata,omegadata]
    #BP的参数(roll,pitch,yaw)

    for i in range(loop):
        w1roll = param.BP_params['w1roll']
        w2roll = param.BP_params['w2roll']
        w1pitch = param.BP_params['w1pitch']
        w2pitch = param.BP_params['w2pitch']
        w1yaw = param.BP_params['w1yaw']
        w2yaw = param.BP_params['w2yaw']
        Outputroll=BP.front(thetadotdata[i][0],omegadata[i][0],w1roll,w2roll)
        Outputpitch=BP.front(thetadotdata[i][1],omegadata[i][1],w1pitch,w2pitch)
        Outputyaw=BP.front(thetadotdata[i][2],omegadata[i][2],w1yaw,w2yaw)
        omegadataBP.append([Outputroll,Outputpitch,Outputyaw])
        w1roll_new,w2roll_new=BP.back(Outputroll,omegadata[i][0],thetadotdata[i][0],w1roll,w2roll)
        w1pitch_new, w2pitch_new = BP.back(Outputpitch,omegadata[i][1], thetadotdata[i][1], w1pitch, w2pitch)
        w1yaw_new, w2yaw_new = BP.back(Outputyaw, omegadata[i][2], thetadotdata[i][2], w1yaw, w2yaw)
        #更新权重
        param.BP_params['w1roll']=w1roll_new
        param.BP_params['w2roll']=w2roll_new
        param.BP_params['w1pitch'] = w1pitch_new
        param.BP_params['w2pitch'] = w2pitch_new
        param.BP_params['w1yaw'] = w1yaw_new
        param.BP_params['w2yaw'] = w2yaw_new


def plot(dt):
    # xmajorLocator = MultipleLocator(10)  # 将x主刻度标签设置为20的倍数
    # xmajorFormatter = FormatStrFormatter('%1.1f')  # 设置x轴标签文本的格式
    # xminorLocator = MultipleLocator(5)  # 将x轴次刻度标签设置为5的倍数
    #
    # ymajorLocator = MultipleLocator(0.5)  # 将y轴主刻度标签设置为0.5的倍数
    # ymajorFormatter = FormatStrFormatter('%1.1f')  # 设置y轴标签文本的格式
    # yminorLocator = MultipleLocator(0.1)  # 将此y轴次刻度标签设置为0.1的倍数

    x=[];y1=[];y2=[];y3=[]
    y1_bp=[];y2_bp=[];y3_bp=[]
    y1error=[];y2error=[];y3error=[]
    for i in range(loop):
        x.append(i*dt)
        y1.append(float(omegadata[i][0]))
        y2.append(float(omegadata[i][1]))
        y3.append(float(omegadata[i][2]))
        y1_bp.append(float(omegadataBP[i][0]))
        y2_bp.append(float(omegadataBP[i][1]))
        y3_bp.append(float(omegadataBP[i][2]))
        y1error.append(float(omegadata[i][0])-float(omegadataBP[i][0]))
        y2error.append(float(omegadata[i][1]) - float(omegadataBP[i][1]))
        y3error.append(float(omegadata[i][2]) - float(omegadataBP[i][2]))

    fig = plt.figure(figsize=(8, 4))
    ax = fig.add_subplot(1, 1, 1)
    ax.xaxis.set_major_locator(MultipleLocator(5))
    ax.xaxis.grid(True)
    ax.yaxis.grid(True)
    ax.plot(x, y1, label="roll_rate", color="red", linewidth=2)
    ax.plot(x, y1_bp, label="roll_rate_bp", color="blue", linewidth=2)
    # ax.plot(x, y2, label="theta_rate", color="blue", linewidth=2)
    # ax.plot(x, y3, label="yaw_rate", color="green", linewidth=2)
    # plt.plot(x, z, "b--", label="$cos(x^2)$")
    plt.xlabel("Time(s)")
    plt.ylabel("omega")
    plt.title('Roll Angular velocity variation')
    plt.legend()
    fig = plt.figure(figsize=(8, 4))
    ax = fig.add_subplot(1, 1, 1)
    ax.xaxis.set_major_locator(MultipleLocator(5))
    ax.xaxis.grid(True)
    ax.yaxis.grid(True)
    ax.plot(x, y1error, label="roll_error", color="red", linewidth=2)
    ax.plot(x, y2error, label="pitch_error", color="blue", linewidth=2)
    ax.plot(x, y3error, label="yaw_error", color="black", linewidth=2)
    plt.xlabel("Time(s)")
    plt.ylabel("error")
    plt.title('BP error')
    plt.legend()
    fig = plt.figure(figsize=(8, 4))
    ax = fig.add_subplot(1, 1, 1)
    ax.xaxis.set_major_locator(MultipleLocator(5))
    ax.xaxis.grid(True)
    ax.yaxis.grid(True)
    ax.plot(x, y2, label="pitch_rate", color="red", linewidth=2)
    ax.plot(x, y2_bp, label="pitch_rate_bp", color="blue", linewidth=2)
    # ax.plot(x, y2, label="theta_rate", color="blue", linewidth=2)
    # ax.plot(x, y3, label="yaw_rate", color="green", linewidth=2)
    # plt.plot(x, z, "b--", label="$cos(x^2)$")
    plt.xlabel("Time(s)")
    plt.ylabel("omega")
    plt.title('Pitch Angular velocity variation')
    plt.legend()
    fig = plt.figure(figsize=(8, 4))
    ax = fig.add_subplot(1, 1, 1)
    ax.xaxis.set_major_locator(MultipleLocator(5))
    ax.xaxis.grid(True)
    ax.yaxis.grid(True)
    ax.plot(x, y3, label="yaw_rate", color="red", linewidth=2)
    ax.plot(x, y3_bp, label="yaw_rate_bp", color="blue", linewidth=2)
    # ax.plot(x, y2, label="theta_rate", color="blue", linewidth=2)
    # ax.plot(x, y3, label="yaw_rate", color="green", linewidth=2)
    # plt.plot(x, z, "b--", label="$cos(x^2)$")
    plt.xlabel("Time(s)")
    plt.ylabel("omega")
    plt.title('Yaw Angular velocity variation')
    plt.legend()
    plt.show()
if '__main__':
    dt = 0.01
    simulate(dt)#TODO如何读取终端的输入dt,如何ctrl+C 结束程序
    BPfunction()
    plot(dt)
    print("roll w1:",param.BP_params['w1roll'])
    print("roll w2:", param.BP_params['w2roll'])
    print("pitch w1:", param.BP_params['w1pitch'])
    print("pitch w2:", param.BP_params['w2pitch'])
    print("yaw w1:", param.BP_params['w1yaw'])
    print("yaw w2:", param.BP_params['w2yaw'])















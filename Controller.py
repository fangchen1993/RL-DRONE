#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    :  //
# @Author  : FC
# @Site    : 2655463370@qq.com
# @license : BSD
#The implement of PID Controller
#RETURN :i qcontroller_params(积分项累积)
import numpy as np
import math



def controller(control_params,quadcopter_params,thetadot,dt):
    if control_params['name'] == 'PID':
        return PID_Control(control_params,quadcopter_params,thetadot,dt)
    if control_params['name'] == 'PD':
        return PD_Control(control_params,quadcopter_params,thetadot,dt)


def PID_Control(control_params,quadcopter_params,thetadot,dt):#TODO

    Kp = control_params['value'][0]
    Ki = control_params['value'][1]
    Kd = control_params['value'][2]
    m = quadcopter_params['m']
    g = quadcopter_params['g']
    k = quadcopter_params['k']
    #先初始化积分器,当然也可在param.py文件中直接初始化
    #此处计分器积分的是角度,由于初始状态角度为0,所以初始化为0,角度大小即为误差大小
    if 'integral' not in control_params and 'integral1' not in control_params:
        control_params['integral'] = np.mat(np.zeros((3,1)))
        control_params['integral1'] = np.mat(np.zeros((3,1)))
    #防止积分饱和
    for i in range(3):
        if abs(control_params['integral1'][i,0])>0.01:
            control_params['integral1'][i,0] = 0


    #计算合力 (Note:计算的准则是合力在z轴的投影和重力相等)\
    Total = m * g / k / (math.cos(control_params['integral'][0, 0]) * math.cos(control_params['integral'][1, 0]))

    #计算误差和输入
    err = Kp*control_params['integral'] + Ki*control_params['integral1'] + Kd *thetadot
    inputs = err2inputs(err,Total,quadcopter_params)
    #更新状态和积分器
    control_params['integral'] += thetadot*dt
    control_params['integral1'] += control_params['integral']*dt
    return inputs,control_params





def PD_Control(control_params,quadcopter_params,thetadot,dt):
    Kp = control_params['value'][0]
    Kd = control_params['value'][2]
    m = quadcopter_params['m']
    g = quadcopter_params['g']
    k = quadcopter_params['k']
    if 'integral' not in control_params:
        control_params['integral'] = np.mat(np.zeros((3,1)))
    #计算合力
    Total = m*g/k/(math.cos(control_params['integral'][0,0])*math.cos(control_params['integral'][1,0]))
    #计算误差和输入
    err = Kp*control_params['integral'] + Kd*thetadot
    input = err2inputs(err,Total,quadcopter_params)
    #Kp的角度积分得到当前新的角度
    control_params['integral'] += thetadot*dt
    return input,control_params

#通过误差得到输入的四个角速度平方
def err2inputs(error,total,quadcopter_params):
    err1 = error[0,0]
    err2 = error[1,0]
    err3 = error[2,0]
    Ix = quadcopter_params['I'][0,0]
    Iy = quadcopter_params['I'][1,1]
    Iz = quadcopter_params['I'][2,2]
    k = quadcopter_params['k']
    L = quadcopter_params['L']
    b = quadcopter_params['b']
    inputs = np.mat(np.zeros((4,1)))
    inputs[0,0] = total/4 - (2*b*err1*Ix+err3*Iz*k*L)/(4*b*k*L)
    inputs[1,0] = total/4 + (err3*Iz)/(4*b) - (err2*Iy)/(2*k*L)
    inputs[2,0] = total/4 - (-2*b*err1*Ix+err3*Iz*k*L)/(4*b*k*L)
    inputs[3,0] = total/4 + (err3*Iz)/(4*b) + (err2*Iy)/(2*k*L)
    return inputs


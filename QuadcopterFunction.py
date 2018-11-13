#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    : 2018/5/22
# @Author  : FC
# @Site    : 2655463370@qq.com
# @license : BSD
#Note:All params are matrix
import numpy as np
import math
import rotation

def thetadot2omega(thetadot,theta):
    """
    通过roll,pitch,yaw的一阶微分计算角速度
    :param thetadot:
    :param theta:[roll pitch yaw]
    """
    rollAngle = theta[0,0]
    pitchAngle = theta[0,1]
    yawAngle = theta[0,2]
    W = np.mat([[1,0,-math.sin(pitchAngle)],
            [0,math.cos(rollAngle),math.cos(pitchAngle)*math.sin(rollAngle)],
            [0,-math.sin(rollAngle),math.cos(pitchAngle)*math.cos(rollAngle)]])
    omega = W*thetadot
    return omega


def omega2thetadot(omega,theta):
    """

    通过角速度计算roll,pitch,yaw的一阶微分
    :param omega:
    :param Angles:
    """
    rollAngle = theta[0,0]
    pitchAngle = theta[0,1]
    yawAngle = theta[0,2]
    W = np.mat([[1,0,-math.sin(pitchAngle)],
            [0,math.cos(rollAngle),math.cos(pitchAngle)*math.sin(rollAngle)],
            [0,-math.sin(rollAngle),math.cos(pitchAngle)*math.cos(rollAngle)]])
    thetadot = W.I * omega
    return thetadot



def acceleration(inputs,theta,xdot,quadparams):
    """
计算惯性参考系的加速度
% Parameters:
%   g: gravity acceleration
%   m: mass of quadcopter
%   k: thrust coefficient
%   kd: global drag coefficient
    :param theta:
    :param xdot:
    :param quadparams:
    """
    m = quadparams['m']
    g = quadparams['g']
    k = quadparams['k']
    kd = quadparams['kd']
    gravity = np.mat([[0],[0],[-g]])
    R = rotation.rotation_DCM(theta)
    T = R*thrust(inputs,k)
    Fd = -kd*xdot
    a = gravity + 1/m * T + Fd/m
    return  a



def thrust(inputs,k):
    """
    根据输入和推力系数计算推力
    :param iuputs:
    :param k:
    :return: T
    """
    t=inputs.sum()
    T = np.mat([[0,0,t]]).T
    return T


def angular_acceleration(inputs,omega,quadparams):
    """
    计算机体系的角加速度
    :param inputs:
    :param omega:
    :param quadparams:
    :return:
    """
    L = quadparams['L']
    b = quadparams['b']
    k = quadparams['k']
    InertiaMatrix = quadparams['I']
    tau = torques(inputs,quadparams)
    omegadot = InertiaMatrix.I*(tau-(np.cross(omega.T,(InertiaMatrix*omega).T)).T)
    return omegadot

def torques(inputs,quadparams):
    """
    计算扭矩
    :param inputs:4x1
    :param quadparams:
    :return: tau 3x1
    """
    L = quadparams['L']
    k = quadparams['k']
    b = quadparams['b']
    tau = np.mat([[L*k*(inputs[0,0]-inputs[2,0])],
                  [L*k*(inputs[1,0]-inputs[3,0])],
                  [b*(inputs[0,0]-inputs[1,0]+inputs[2,0]-inputs[3,0])]])
    return tau







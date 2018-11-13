#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    :  //
# @Author  : FC
# @Site    : 2655463370@qq.com
# @license : BSD
import param,BP
import ReadData
from matplotlib import pyplot as plt
from matplotlib.ticker import MultipleLocator, FormatStrFormatter

def BPfunction(loop):
    #[u,y]=[thetadotdata,omegadata]
    #BP的参数(roll,pitch,yaw)
    for i in range(loop):
        w1roll = param.BP_params['w1roll']
        w2roll = param.BP_params['w2roll']
        w1pitch = param.BP_params['w1pitch']
        w2pitch = param.BP_params['w2pitch']
        w1yaw = param.BP_params['w1yaw']
        w2yaw = param.BP_params['w2yaw']
        Outputroll=BP.front(roll[i],roll_ctr[i],w1roll,w2roll)
        Outputpitch=BP.front(pitch[i],pitch_ctr[i],w1pitch,w2pitch)
        Outputyaw=BP.front(yaw[i],yaw_ctr[i],w1yaw,w2yaw)
        roll_bp.append(Outputroll)
        pitch_bp.append(Outputpitch)
        yaw_bp.append(Outputyaw)
        w1roll_new,w2roll_new=BP.back(Outputroll,roll_ctr[i],roll[i],w1roll,w2roll)
        w1pitch_new, w2pitch_new = BP.back(Outputpitch,pitch_ctr[i],pitch[i], w1pitch, w2pitch)
        w1yaw_new, w2yaw_new = BP.back(Outputyaw, yaw_ctr[i], yaw[i], w1yaw, w2yaw)
        #更新权重
        param.BP_params['w1roll']=w1roll_new
        param.BP_params['w2roll']=w2roll_new
        param.BP_params['w1pitch'] = w1pitch_new
        param.BP_params['w2pitch'] = w2pitch_new
        param.BP_params['w1yaw'] = w1yaw_new
        param.BP_params['w2yaw'] = w2yaw_new
#可视化
def plot(loop,dt):
    time=[];err_roll=[];err_pitch=[];err_yaw=[]
    for i in range(loop):
        time.append(i*dt)
        err_roll.append((roll[i]-roll_bp[i]))
        err_pitch.append((pitch[i] - pitch_bp[i]))
        err_yaw.append((yaw[i] - yaw_bp[i]))
    fig = plt.figure(figsize=(8, 4))
    ax = fig.add_subplot(1, 1, 1)
    ax.xaxis.set_major_locator(MultipleLocator(5))
    ax.xaxis.grid(True)
    ax.yaxis.grid(True)
    ax.plot(time, roll, label="actuator_control_roll", color="red", linewidth=2)
    ax.plot(time, roll_bp, label="actuator_control_roll_bp", color="blue", linewidth=2)
    plt.xlabel("Time(s)")
    plt.ylabel("roll angle")
    plt.title('Roll Angular velocity variation')
    plt.legend()
    fig = plt.figure(figsize=(8, 4))
    ax = fig.add_subplot(1, 1, 1)
    ax.xaxis.set_major_locator(MultipleLocator(5))
    ax.xaxis.grid(True)
    ax.yaxis.grid(True)
    ax.plot(time, err_roll, label="err_roll", color="red", linewidth=2)
    ax.plot(time, err_pitch, label="err_pitch", color="green", linewidth=2)
    ax.plot(time, err_yaw, label="err_yaw", color="blue", linewidth=2)
    plt.xlabel("Time(s)")
    plt.ylabel("error")
    plt.title(' Angular velocity variation')
    plt.legend()
    plt.show()



# BP OUT
roll_bp = [];pitch_bp = [];yaw_bp=[]
# get data
actuator, rates_setpoint, rates_ctr, sensor_combine, attitude_setpoint = ReadData.readData()

# data processing
roll = [];pitch = [];yaw = []
roll_ctr = [];pitch_ctr = [];yaw_ctr = []
m = len(actuator)
for i in range(2, m):
    roll_ctr.append(float(actuator[i][2]))
    pitch_ctr.append(float(actuator[i][3]))
    yaw_ctr.append(float(actuator[i][4]))

n = len(attitude_setpoint)
for i in range(1, n):
    roll.append(float(attitude_setpoint[i][1]))
    pitch.append(float(attitude_setpoint[i][2]))
    yaw.append(float(attitude_setpoint[i][3]))
BPfunction(len(roll))
plot(len(roll),0.1)
print("roll w1:", param.BP_params['w1roll'])
print("roll w2:", param.BP_params['w2roll'])
print("pitch w1:", param.BP_params['w1pitch'])
print("pitch w2:", param.BP_params['w2pitch'])
print("yaw w1:", param.BP_params['w1yaw'])
print("yaw w2:", param.BP_params['w2yaw'])


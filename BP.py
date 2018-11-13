#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    :  //
# @Author  : FC
# @Site    : 2655463370@qq.com
# @license : BSD
# BP神经网络控制器的实现
#input:[rate_pre,rate,rate_set]  1x3
import param
import numpy as np

eta = param.BP_params['eta']
alpha = param.BP_params['alpha']

#input:(theta_pre,theta,theta_set) | (phi_pre,phi,phi_set) | (psi_pre,psi,psi_set)



def front(u,y,w1,w2):
    x=[]
    yy=[]
    for i in range(6):
        x.append(sigmod(w1[0,i]*float(u)+w1[1,i]*float(y)))
    for i in range(6):
        yy.append(x[i]*w2[i,0])

    Output=sum(yy)
    return Output

def back(Output,setpoint,u,wi1,wi2): #output:1x1 setpoint:1x1 inputNum:(u,y)
    x=[]
    err=setpoint-Output
    for i in range(6):
        x.append(sigmod(wi1[0,i]*float(u)+wi1[1,i]*float(setpoint)))

    w2_gra = np.mat(np.random.random((6,1)))
    w1_gra=np.mat(np.random.random((2, 6)))
    for i in range(6):
        w2_gra[i,0]=eta*err*x[i]

    for j in range(6):
        w1_gra[0,j]=wi2[j,0]*x[j]*(1-x[j])*u*err*eta
        w1_gra[1,j] = wi2[j, 0] * x[j] * (1 - x[j]) *setpoint* err * eta
    w1_new = wi1+w1_gra
    w2_new = wi2+w2_gra
    return w1_new,w2_new



def sigmod(x):
    return 1/(1+np.exp(-x))




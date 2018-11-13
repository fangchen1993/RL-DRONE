#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    :  //
# @Author  : FC
# @Site    : 2655463370@qq.com
# @license : BSD
import param
import numpy as np
def sigmod(x):
    m,n=x.shape
    a=np.mat(np.zeros((m,n)))
    for i in range(n):
        a[0,i]=float(1/(1+np.exp(-x[0,i])))
    return a[0,0]
y=np.mat([[1,2,3]])
b=np.mat([[1,2,3,4],[4,5,6,4],[7,8,9,4]])
c=y*b
m,n=c.shape
a=np.mat(np.zeros((m,n)))
for i in range(n):
    a[0,i]=float(1/(1+np.exp(-c[0,i])))
for i in range(n):
    print(a[0,i])





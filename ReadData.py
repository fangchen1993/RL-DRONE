#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    :  //
# @Author  : FC
# @Site    : 2655463370@qq.com
# @license : BSD
import csv
import os
import numpy as np
import random
import requests
# name of data file
#得到CSV数据有pyulog:command "ulog2csv xxx.ulg"
# 数据集名称
def readData():
    actuator = []
    rates_setpoint = []
    rates_ctr = []
    sensor_combine = []
    attitude_setpoint = []
    file_name1 = '09_21_31_actuator_controls_0_0.csv'
    file_name2 = '09_21_31_vehicle_rates_setpoint_0.csv'
    file_name3 = '09_21_31_rate_ctrl_status_0.csv'
    file_name4 = '09_21_31_sensor_combined_0.csv'
    file_name5 = '09_21_31_vehicle_attitude_setpoint_0.csv'
    csv_reader1 = csv.reader(open(file_name1))
    for row in csv_reader1:
        actuator.append(row)

    csv_reader2 = csv.reader(open(file_name2))
    for row in csv_reader2:
        rates_setpoint.append(row)

    csv_reader3 = csv.reader(open(file_name3))
    for row in csv_reader3:
        rates_ctr.append(row)

    csv_reader4 = csv.reader(open(file_name4))
    for row in csv_reader4:
        sensor_combine.append(row)
    csv_reader5 = csv.reader(open(file_name5))
    for row in csv_reader5:
        attitude_setpoint.append(row)

    return actuator,rates_setpoint,rates_ctr,sensor_combine,attitude_setpoint



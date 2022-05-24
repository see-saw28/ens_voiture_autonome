#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May 24 16:58:18 2022

@author: student
"""
import numpy as np
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt


angles = np.linespace(-np.pi,np.pi,360)

decimation_number = 90

def constant_angle_decimation(angles):
    decimated_angles = 
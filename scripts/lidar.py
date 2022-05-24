#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May 24 16:42:43 2022

@author: student
"""
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt

import lidar_tools as lt


def callback(data):
    angle_min = data.angle_min
    angle_max = data.angle_max
    angle_increment = data.angle_increment
    ranges = data.ranges
    print(angle_min, angle_max)
    
    angles = np.linspace(angle_min,angle_max,len(ranges))
    x=[]
    y=[]
    for i,angle in enumerate(angles):
        x.append(ranges[i]*np.cos(angle))
        y.append(ranges[i]*np.sin(angle))
        
    print(x[0],x[180])
    
    


def main():

   

    # init node
    rospy.init_node('lidar')
    rate = rospy.Rate(100) # hz
    
       
    rospy.Subscriber('scan', LaserScan, callback, queue_size=10)
    rospy.spin()


    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
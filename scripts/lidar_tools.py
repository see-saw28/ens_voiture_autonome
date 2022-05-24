#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May 24 16:58:18 2022

@author: student
"""
import numpy as np
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt

box_x=4
box_y=1

def angle_to_box(angle):
    box_angle=np.arcsin(box_y/np.sqrt(box_x**2+box_y**2))
    
    abs_angle=abs(angle)
    if abs_angle<box_angle:
        x=box_x/2
        y=x*np.tan(angle)
    elif abs_angle>np.pi-box_angle: 
        # print('hre')
        x=-box_x/2
        y=abs(x)*np.tan(angle)
    else :
        y=box_y/2*np.sign(angle)
        x=abs(y)/np.tan(angle)*np.sign(angle)
        
    return(x,y)

def box_to_angle(x,y):
    return np.atan2(y,x)

def constant_angle_decimation(angles,angle_min,angle_max,angle_increment,decimation_number):
    initial_number = len(angles)
    new_angle_increment = initial_number/decimation_number*angle_increment
    new_angles = np.linspace(angle_min,angle_max,decimation_number)
    return new_angles,new_angle_increment

def mask(angles,new_min_angle,new_max_angle):
    new_angles = []
    for angle in angles:
        if angle>new_min_angle and angle<new_max_angle:
            new_angles.append(angle)
    return new_angles

def laser_plot(ranges,angle_min,angle_max,angle_increment):
    angles = np.linspace(angle_min,angle_max,80)
    x=[]
    y=[]
    
    fig, (ax1, ax2,ax3,ax4) = plt.subplots(1, 4)
    fig.suptitle('Horizontally stacked subplots')
    

    
    for i,angle in enumerate(angles):
        xi,yi = angle_to_box(angle)
        x.append(xi)
        y.append(yi)
        if i==len(angles)/2:
            c='g'
            # print(angle)
        else:
            c='r'
        ax1.plot([0,yi],[0,xi],c)
    ax1.plot([-box_y/2,-box_y/2,box_y/2,box_y/2,-box_y/2],[box_x/2,-2,-2,2,2],'b')
    
    angles_truncated = mask(angles,-np.pi*2/3,np.pi*2/3)
    
    for i,angle in enumerate(angles_truncated):
        xi,yi = angle_to_box(angle)
        x.append(xi)
        y.append(yi)
        if i==len(angles_truncated)/2:
            c='g'
            # print(angle)
        else:
            c='r'
        ax2.plot([0,yi],[0,xi],c)
    ax2.plot([-box_y/2,-box_y/2,box_y/2,box_y/2,-box_y/2],[box_x/2,-2,-2,2,2],'b')
    
    decimation_number = 20
    
    angles_decimated,_ = constant_angle_decimation(angles_truncated,-np.pi*2/3,np.pi*2/3,2*np.pi/360,decimation_number)
    
    for i,angle in enumerate(angles_decimated):
        xi,yi = angle_to_box(angle)
        x.append(xi)
        y.append(yi)
        if i==len(angles_decimated)/2:
            c='g'
            # print(angle)
        else:
            c='r'
        ax3.plot([0,yi],[0,xi],c)
    ax3.plot([-box_y/2,-box_y/2,box_y/2,box_y/2,-box_y/2],[box_x/2,-2,-2,2,2],'b')
    
    decimation_number = 20
    
    # angles_decimated,_ = constant_angle_decimation(angles_truncated,-np.pi*2/3,np.pi*2/3,2*np.pi/360,decimation_number)
    
    # for i,angle in enumerate(angles_decimated):
    #     xi,yi = angle_to_box(angle)
    #     x.append(xi)
    #     y.append(yi)
    #     if i==len(angles_decimated)/2:
    #         c='g'
    #         # print(angle)
    #     else:
    #         c='r'
    #     ax3.plot([0,yi],[0,xi],c)
    # ax3.plot([-box_y/2,-box_y/2,box_y/2,box_y/2,-box_y/2],[box_x/2,-2,-2,2,2],'b')
    
    plt.show()
    
    
    
# laser_plot([],-np.pi*2/3,np.pi*2/3,2*np.pi/360)
laser_plot([],-np.pi,np.pi,2*np.pi/360)
decimation_number = 90


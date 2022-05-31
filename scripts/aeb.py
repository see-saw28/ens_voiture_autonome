#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May 31 10:37:32 2022

@author: student
"""
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np
from nav_msgs.msg import Path, Odometry

aeb_distance = 0.5
TTC_threshold = 0.7

wb = 0.257
alpha = 0
velocity = 0.0


    
def cmd_callback(data):
    global alpha
    steer = data.angular.z
    
    alpha = np.arcsin(aeb_distance*np.tan(steer)/(2*wb))
    
def odom_callback(data):
    
    global velocity
    velocity = -data.twist.twist.linear.x

def main():

    global pub

    # init node
    rospy.init_node('lidar')
    rate = rospy.Rate(100) # hz
    
    pub = rospy.Publisher('AEB', Bool, queue_size=1)  
    rospy.Subscriber('scan', LaserScan, lidar_callback, queue_size=10)
    rospy.Subscriber('cmd_vel', Twist, cmd_callback, queue_size=10)
    rospy.Subscriber('camera/odom/sample', Odometry, odom_callback, queue_size=10)
    
    rospy.spin()

def lidar_callback(data):
    
    angle_min = data.angle_min
    angle_max = data.angle_max
    angle_increment = data.angle_increment
    ranges = data.ranges
    # print(angle_min, angle_max)
    laser_number = int(alpha/angle_increment)
    
        
    # print(ranges[laser_number],alpha,laser_number)
    b = Bool()
    TTC = ranges[laser_number]/velocity
    # print(TTC)
    if TTC<TTC_threshold and TTC >0:
        print('break',TTC)
        b.data = True
    else :
        b.data = False
        
    pub.publish(b)
        
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jun 15 10:32:15 2022

@author: student
"""
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jun  5 21:33:32 2022

@author: student
"""
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jun  2 10:58:27 2022

@author: student
"""
"""
Mobile robot motion planning sample with Dynamic Window Approach
author: Atsushi Sakai (@Atsushi_twi), Göktuğ Karakaşlı
"""



import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry


stuck = False





def odom_callback(data):
    global stuck
    velocity = -data.twist.twist.linear.x
    
    if abs(velocity)<0.1:
        stuck = True
        
    elif stuck :
        stuck = False
    


def lidar_callback(data):
    global config
    global stuck
    angle_min = data.angle_min
    angle_max = data.angle_max
    angle_increment = data.angle_increment
    ranges = data.ranges
    angles = np.linspace(angle_min,angle_max,len(ranges))
    xy=[]
    if ranges[0]<0.2:
        stuck = True
    elif ranges[0]<10 and stuck :
        stuck = False
        
    pub_stuck.publish(Bool(stuck))
    
    
    # print(config.ob)
    





    
   


if __name__ == '__main__':
    rospy.init_node('stuck_detection')
    
    
    pub_stuck = rospy.Publisher('stuck', Bool, queue_size=10)
    
    rospy.Subscriber('scan', LaserScan, lidar_callback, queue_size=10)
    rospy.Subscriber('camera/odom/sample', Odometry, odom_callback, queue_size=10)
    # main(robot_type=RobotType.circle)
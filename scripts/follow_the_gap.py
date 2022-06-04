#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jun  4 15:43:34 2022

@author: student
"""
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May 31 10:37:32 2022

@author: student
"""
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose, Point, Vector3, Quaternion, PoseStamped
from std_msgs.msg import Bool
import numpy as np
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA, String
from tf.transformations import quaternion_from_euler


    
def odom_callback(data):
    
    global velocity
    velocity = -data.twist.twist.linear.x

def main():

    global pub
    global pub_marker
    global pub_marker1

    # init node
    rospy.init_node('follow_the_gap')
    rate = rospy.Rate(100) # hz
    
    
    rospy.Subscriber('scan', LaserScan, lidar_callback, queue_size=10)
    # rospy.Subscriber('cmd_vel', Twist, cmd_callback, queue_size=10)
    # rospy.Subscriber('camera/odom/sample', Odometry, odom_callback, queue_size=10)
    pub_marker = rospy.Publisher('follow_the_gap_marker', Marker, queue_size=10)
    pub_marker1 = rospy.Publisher('follow_the_gap_obstacle_marker', Marker, queue_size=10)
    pub = rospy.Publisher('follow_the_gap_cmd', Twist, queue_size=10)
    rospy.spin()
    
def publish_marker(pub, x, y, theta, collide=False):
    marker = Marker()     
    marker.header=Header(frame_id='laser')
    marker.type=Marker.CUBE
    marker.scale=Vector3(0.2, 0.2, 0.2)
    a,b,c,d = quaternion_from_euler(0,0,theta)
    marker.pose=Pose(Point(x,y,0), Quaternion(a,b,c,d))
    if collide :
        marker.color = ColorRGBA(1,0,0,1)
    else :
        marker.color = ColorRGBA(0,1,0,1)
    marker.lifetime = rospy.Duration(100)
    
    pub.publish(marker)

def lidar_callback(data):
    global pub_marker
    angle_min = data.angle_min
    angle_max = data.angle_max
    angle_increment = data.angle_increment
    ranges = np.roll(np.array(data.ranges),180)
    # print(angle_min, angle_max)
    n = len(ranges)
    angles = np.linspace(angle_min,angle_max,n)
    xy=[]
    
    ranges[ranges>12]=0
    
    max_distance = 4
    
    ranges[ranges>max_distance]=max_distance
    l=2
    cut_angle = np.pi/4
    new_ranges=np.zeros(n)
    for i,angle in enumerate(angles):
        if abs(angle)<cut_angle:
            #convolution
            new_ranges[i] = np.mean(ranges[max(0,i-l):min(i+l+1,n)])
    
    
    closest_dist = min(ranges[ranges!=0])
    closest_dist_index = list(ranges).index(closest_dist)
    closest_dist_angle = angle_increment * closest_dist_index + angle_min
    
    x = -closest_dist * np.cos(closest_dist_angle)
    y = -closest_dist * np.sin(closest_dist_angle)
    publish_marker(pub_marker1, x, y, closest_dist_angle, collide=True)
    # print(closest_dist, closest_dist_index,closest_dist_angle)
    
    # eliminate all points in the bubble
    radius = 0.15
    
    bubble_number = int(np.ceil(radius / (closest_dist * angle_increment)))
          
    for i in range(bubble_number):
        new_ranges[closest_dist_index+i] = 0
        new_ranges[closest_dist_index-i] = 0
        
    # find the max gap in free space
    
    max_idx = int(abs(cut_angle - angle_min) / angle_increment)
    min_idx = n - max_idx
    
    start = min_idx
    end = min_idx
    current_start = min_idx -1
    duration = 0
    longest_duration = 0


    for i in range(min_idx, max_idx):
        if current_start < min_idx :
            if new_ranges[i]>0:
                current_start = i
        elif new_ranges[i]<=0:
            duration = i - current_start
            if duration > longest_duration :
                longest_duration = duration
                start = current_start
                end = i-1
            current_start = min_idx - 1
            
    if current_start >= min_idx :
        duration = max_idx + 1 - current_start
        if duration > longest_duration:
            longest_duration = duration
            start = current_start
            end = max_idx
      
    current_max = 0
    for i in range(start,end):
        if new_ranges[i]>current_max:
            current_max = new_ranges[i]
            angle = angle_increment * i + angle_min
            
        elif new_ranges[i]==current_max:
            if abs(angle_increment * i + angle_min)< abs(angle):
                angle = angle_increment * i + angle_min
    
    # display furthest point 
    x = -current_max * np.cos(angle)
    y = -current_max * np.sin(angle)
    publish_marker(pub_marker, x, y, angle)
    
    
    wheelbase_length = 0.257
    max_steering_angle = 0.3
    
    # publish steering command
    msg = Twist()
    
    # Raw angle from the point
    delta = np.clip(angle, -max_steering_angle, max_steering_angle)
    msg.angular.z = delta
    
   
    # Steering as PP
    delta = math.atan2(2.0 * wheelbase_length * math.sin(angle) / current_max, 1.0)
    delta = np.clip(delta, -max_steering_angle, max_steering_angle)
    msg.angular.z = delta
    
    
    
    if current_max < 1.0 :
        speed = 0.5
        
    elif current_max < 3.0 :
        speed = current_max
        
    else :
        speed = 3.0
        
    msg.linear.x = speed
    
    pub.publish(msg)
    
        
 
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
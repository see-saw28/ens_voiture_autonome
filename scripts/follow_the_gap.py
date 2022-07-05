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
from dynamic_reconfigure.server import Server
from ens_voiture_autonome.cfg import FTGConfig

max_distance = 2.0
cut_angle = np.pi/2
conv_width = 2
wheelbase_length = 0.257
max_steering_angle = 0.3
radius = 0.40
max_velocity=3.0
obstacle_velocity=0.5
near_distance=0.7
max_velocity_distance=3.0
direct_steering=False
infinite_to_max=True

def callback(config, level):
    global max_distance
    global cut_angle
    global conv_width
    global wheelbase_length
    global max_steering_angle
    global radius
    global max_velocity
    global obstacle_velocity
    global near_distance
    global max_velocity_distance
    global direct_steering
    global infinite_to_max

    max_distance=config['max_distance']
    cut_angle=config['cut_angle']
    conv_width=config['conv_width']
    wheelbase_length=config['wheelbase_length']
    max_steering_angle=config['max_steering_angle']
    radius=config['radius']
    max_velocity=config['max_velocity']
    obstacle_velocity=config['obstacle_velocity']
    near_distance=config['near_distance']
    max_velocity_distance=config['max_velocity_distance']
    direct_steering=config['direct_steering']
    infinite_to_max=config['infinite_to_max']


    return config

def main():

    global pub
    global pub_marker
    global pub_marker1
    global pub_scan

    # init node
    rospy.init_node('follow_the_gap')
    rate = rospy.Rate(100) # hz


    rospy.Subscriber('scan', LaserScan, lidar_callback, queue_size=10)
    pub_marker = rospy.Publisher('follow_the_gap_marker', Marker, queue_size=10)
    pub_marker1 = rospy.Publisher('follow_the_gap_obstacle_marker', Marker, queue_size=10)
    pub = rospy.Publisher('follow_the_gap_cmd', Twist, queue_size=10)
    pub_scan = rospy.Publisher('ftg_scan', LaserScan, queue_size=10)
    srv = Server(FTGConfig, callback)
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
    range_max = data.range_max
    ranges = np.roll(np.array(data.ranges),180)
    # print(angle_min, angle_max)
    n = len(ranges)
    angles = np.linspace(angle_min,angle_max,n)


    if infinite_to_max :
        ranges[ranges>range_max]=max_distance

    else :
        ranges[ranges>range_max] = 0





    ranges[ranges>max_distance]=max_distance


    new_ranges=np.zeros(n)
    for i,angle in enumerate(angles):
        if abs(angle)<cut_angle:
            #convolution
            new_ranges[i] = np.mean(ranges[max(0,i-conv_width):min(i+conv_width+1,n)])


    closest_dist = min(new_ranges[new_ranges!=0])
    closest_dist_index = list(new_ranges).index(closest_dist)
    closest_dist_angle = angle_increment * closest_dist_index + angle_min

    x = -closest_dist * np.cos(closest_dist_angle)
    y = -closest_dist * np.sin(closest_dist_angle)
    publish_marker(pub_marker1, x, y, closest_dist_angle, collide=True)
    # print(closest_dist, closest_dist_index,closest_dist_angle)



    bubble_number = int(np.ceil(radius / (closest_dist * angle_increment)))

    for i in range(bubble_number):
        new_ranges[(closest_dist_index+i)%n] = 0
        new_ranges[(closest_dist_index-i)%n] = 0

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




    # publish steering command
    msg = Twist()

    if direct_steering:
    # Raw angle from the point
        delta = np.clip(angle, -max_steering_angle, max_steering_angle)
        msg.angular.z = delta

    else :
    # Steering as PP
        delta = math.atan2(2.0 * wheelbase_length * math.sin(angle) / current_max, 1.0)
        delta = np.clip(delta, -max_steering_angle, max_steering_angle)
        msg.angular.z = delta



    if current_max < near_distance :
        speed = obstacle_velocity

    elif current_max < max_velocity_distance :
        speed = current_max

    else :
        speed = max_velocity

    msg.linear.x = speed

    pub.publish(msg)

    new_ranges = np.roll(np.array(new_ranges),-180)
    data.ranges = new_ranges
    # data.header.stamp = rospy.Time.now()
    pub_scan.publish(data)




if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
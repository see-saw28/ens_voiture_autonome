#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jun  4 15:31:05 2022

@author: student
"""
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May 23 16:31:31 2022

@author: student
"""
#!/usr/bin/env python

'''
* ROS Pure Pursuit node ****************************
 
 Pure Pursuit path tracker for following a path.
 
 Reference: 
 https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf 
 
 Pure Pursuit algorithm adapted from: 
 https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathTracking/pure_pursuit
 By Jon Eivind Stranden @ NTNU 2019
****************************************************
'''

import rospy
import math
import os 
import numpy as np
import time
from std_msgs.msg import Float64, String, Bool
from geometry_msgs.msg import PointStamped, Twist, PoseWithCovarianceStamped, Pose, Point, Vector3, Quaternion, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA, String
from sensor_msgs.msg import LaserScan
# from ens_voiture_autonome.msg import DS4
import tf
import rospkg
import pickle
from dynamic_reconfigure.server import Server
from ens_voiture_autonome.cfg import PurePursuitConfig

import path_tools

### Tuning settings #############################

k = 0.4  # Look forward gain (Dynamic look-ahead)
look_ahead_dist = 0.6  # Look-ahead distance

k_obstacle = 0.4  # Look forward gain (Dynamic look-ahead)
look_ahead_dist_obstacle = 0.6  # Look-ahead distance

lidar_look_ahead_dist = 1.0
steering_angle = 0.0

wheelbase_length = 0.257  # wheel base of vehicle [m]
max_steering_angle = 0.30 # rad
radius = 0.15 # m

detect_collision = False
#################################################

# Path point storage
course_x = []
course_y = []

collision = False



class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

# initial state
state = State(x=0.0, y=0.0, yaw=0.0, v=0.0)

def normalize_angle(angle):
    # adapted from: https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathTracking/stanley_controller

    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle

def pure_pursuit_control(state, course_x, course_y):
    # adapted from: https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathTracking/pure_pursuit

    global lidar_look_ahead_dist
    # Get waypoint target index
    ind, ind_car = calc_target_index(state, course_x, course_y)

    if ind < len(course_x):
        target_x = course_x[ind%len(course_x)]
        target_y = course_y[ind%len(course_x)]
    

    # Calc angle alpha between the vehicle heading vector and the look-ahead vector
    alpha = math.atan2(target_y - state.y, target_x - state.x) - (state.yaw)

    if state.v < 0:  # back
        alpha = math.pi - alpha

    # Dynamic look-ahead distance
    dyn_look_ahead_dist = k * abs(state.v) + look_ahead_dist
    
    
    lidar_look_ahead_dist = look_ahead_dist_obstacle + k_obstacle * state.v - 0.235*np.cos(alpha)

    # Calc steering wheel angle delta
    delta = math.atan2(2.0 * wheelbase_length * math.sin(alpha) / dyn_look_ahead_dist, 1.0)

    # Cap max steering wheel angle
    delta = np.clip(delta, -max_steering_angle, max_steering_angle)
    
    # Calc turning radius
    R = dyn_look_ahead_dist / (2*np.sin(abs(alpha)))
    
    speed = course_speed[ind_car%len(course_speed)]
    
    return speed, delta, ind, normalize_angle(alpha), R


def calc_target_index(state, course_x, course_y):
    # adapted from: https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathTracking/pure_pursuit
    global dyn_look_ahead_dist
    global k
    global look_ahead_dist
    

    dyn_look_ahead_dist = k * state.v + look_ahead_dist
    # search nearest point index
    
    dx = [state.x - icx for icx in course_x]
    dy = [state.y - icy for icy in course_y]
    d = [math.sqrt(idx ** 2 + idy ** 2)for (idx, idy) in zip(dx, dy)]
    ind_car = d.index(min(d))
    ind = ind_car
    distance = 0
    while distance < dyn_look_ahead_dist :
        ind += 1
        distance = d[ind%len(course_x)] 
        
        if ind >len(course_x)*2:
            ind = ind_car
            rospy.loginfo('too far away')
            break
          
    
    return ind%len(course_x), ind_car


def callback(config, level):
    global k
    global look_ahead_dist
    global k_obstacle
    global look_ahead_dist_obstacle
    global wheelbase_length
    global max_steering_angle
    global radius
    global detect_collision
    
    
    k = config['k']  # Look forward gain (Dynamic look-ahead)
    look_ahead_dist = config['look_ahead_dist']  # Look-ahead distance

    k_obstacle = config['k_obstacle'] # Look forward gain (Dynamic look-ahead)
    look_ahead_dist_obstacle = config['look_ahead_dist_obstacle']  # Look-ahead distance

    wheelbase_length = config['wheelbase_length']  # wheel base of vehicle [m]
    max_steering_angle = config['max_steering_angle'] # rad
    radius = config['radius'] # m

    detect_collision = config['detect_collision']
    
    
            
    
    return config

def update_state_callback(data):

    global state

    state.x = data.pose.pose.position.x
    state.y = data.pose.pose.position.y

    # Convert quaternions to euler to get yaw
    orientation_list = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    _, _, yaw = euler_from_quaternion(orientation_list)
    state.yaw = yaw


def load_path_callback(msg):
    
    global course_x
    global course_y
    global course_speed
    global pub_full_path
    global msg_path1

    msg=msg.data.split(" ")
    
    """
    if (msg[0]=="load" and len(msg)>1):
        path_x = []
        path_y = []
        course_speed = []
        
        # display the path to the look ahead point
        msg_path1 = Path()
        msg_path1.header.frame_id = 'map'
        
        
            
        if 'traj' in msg[1]:   
        
       
            f = open(rospack.get_path('ens_vision')+f'/paths/{msg[1]}.pckl', 'rb')
            marker,speeds,orientations,cmd_speeds = pickle.load(f)
            f.close()
            for i, pose1 in enumerate(marker.points):
                path_x.append(pose1.x)
                path_y.append(pose1.y)
                course_speed.append(cmd_speeds[i])
                
                pose = PoseStamped()
                
                pose.header.frame_id = "map"
                pose.header.seq = i
                
                pose.pose.position.x = pose1.x
                pose.pose.position.y = pose1.y
                
                msg_path1.poses.append(pose)
                
        elif 'mcp' in msg[1]:   
        
       
            f = open(rospack.get_path('ens_voiture_autonome')+f'/paths/{msg[1]}.npy', 'rb')
            raceline = np.load(f)
            f.close()
            for i, position in enumerate(raceline):
                path_x.append(position[0])
                path_y.append(position[1])
                course_speed.append(2)
                
                pose = PoseStamped()
            
                pose.header.frame_id = "map"
                pose.header.seq = i
                
                
                pose.pose.position.x = position[0]
                pose.pose.position.y = position[1]
                
                msg_path1.poses.append(pose)
            
        pub_full_path.publish(msg_path1)

        course_x = path_x
        course_y = path_y
        rospy.loginfo('traj loaded')"""
        
    if (msg[0]=="load"):
        if len(msg)>1:  
            if 'mcp' in msg[1]:
                mcp = path_tools.load_mcp(msg[1])
                path = path_tools.mcp_to_path(mcp)
                msg_path1 = path
            else :
                msg_path1 = path_tools.load_path(msg[1])
            pub_full_path.publish(msg_path1)

            path_x = []
            path_y = []
            path_yaw = []
            course_speed = []
            
            for i, pose in enumerate(msg_path1.poses):
                path_x.append(pose.pose.position.x)
                path_y.append(pose.pose.position.y)
                orientation_list = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
                _, _, yaw = euler_from_quaternion(orientation_list)
                path_yaw.append(yaw)
                course_speed.append(1)
            
            course_x = path_x
            course_y = path_y
            course_yaw = path_yaw
        



def mcp_callback(msg):
    
    global course_x
    global course_y
    global course_speed
    global pub_full_path
    global msg_path

    
   
    path_x = []
    path_y = []
    course_speed = []
    
   
        

    for pose in msg.poses:
        path_x.append(pose.pose.position.x)
        path_y.append(pose.pose.position.y)
        course_speed.append(1)
        
    course_x = path_x
    course_y = path_y
        
    # rospy.loginfo('loaded mcp path : ',len(path_x), ' poses')
    

    
def odom_callback(data):
    
    global state
    state.v = -data.twist.twist.linear.x
    # rospy.loginfo(state.v)
    


def lidar_callback(data):
    global pub_marker
    global pub
    global alpha
    global collision
    global pub_collision
    
    angle_min = data.angle_min
    angle_max = data.angle_max
    angle_increment = data.angle_increment
    ranges = data.ranges
    # rospy.loginfo(angle_min, angle_max)
    
    if detect_collision:
    
        alpha = np.arcsin(lidar_look_ahead_dist*np.tan(steering_angle)/(2*wheelbase_length))
        
        try :
            laser_number = int(alpha/angle_increment)
            
        except:
            laser_number = 0
        
        
        number_of_check = int(radius / (0.5*angle_increment))
    
        for i in range(laser_number-number_of_check, laser_number+number_of_check):
            # if ranges[i]*abs(i-laser_number)<radius:
                # rospy.loginfo(ranges[i], lidar_look_ahead_dist)
            if ranges[i]<lidar_look_ahead_dist and ranges[i]*abs(i-laser_number)*angle_increment<radius:
                collision = True
                break
                
            else :
                collision = False
            
    
    
   
        
    
    
def publish_marker(pub, x, y, theta, collide=False):
    marker = Marker()     
    marker.header=Header(frame_id='map')
    marker.type=Marker.SPHERE
    marker.scale=Vector3(0.15, 0.15, 0.15)
    a,b,c,d = quaternion_from_euler(0,0,theta)
    marker.pose=Pose(Point(x,y,0), Quaternion(a,b,c,d))
    if collide :
        marker.color = ColorRGBA(1,0,0,1)
    else :
        marker.color = ColorRGBA(0,0,1,1)
    marker.lifetime = rospy.Duration(100)
    
    pub.publish(marker)
    
def publish_collision_marker(pub, alpha, collide=False):
    marker = Marker()     
    marker.header=Header(frame_id='laser')
    marker.type=Marker.LINE_STRIP
    marker.scale=Vector3(0.05, 0.2, 0.2)
    
    marker.pose=Pose(Point(0,0,0), Quaternion(0,0,0,1))
    marker.points.append(Point(0,0,0))
    x = -lidar_look_ahead_dist * np.cos(alpha)
    y = -lidar_look_ahead_dist * np.sin(alpha)
    marker.points.append(Point(x,y,0))
    if collide :
        marker.color = ColorRGBA(1,0,0,1)
    else :
        marker.color = ColorRGBA(0,1,0,1)
    marker.lifetime = rospy.Duration(100)
    
    pub.publish(marker)

def main():
    global steering_angle
    global state
    global pub_full_path
    global msg_path1
    
    msg_path1 = Path()

    # Publish
    pub = rospy.Publisher('pure_pursuit_cmd', Twist, queue_size=100)
    pub_marker = rospy.Publisher('pure_pursuit_look_ahead', Marker, queue_size=10)
    pub_path = rospy.Publisher('pure_pursuit_path', Path, queue_size=10)
    pub_full_path = rospy.Publisher('loaded_path', Path, queue_size=10)
    
    pub_collision = rospy.Publisher('collision', Bool, queue_size=10)
    pub_collision_marker = rospy.Publisher('collision_check', Marker, queue_size=10)

    # Get current state of truck
    #rospy.Subscriber('pf/viz/inferred_pose', PoseStamped, update_state_callback, queue_size=10)
    rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, update_state_callback, queue_size=100)
    rospy.Subscriber('syscommand', String, load_path_callback, queue_size=10)
    rospy.Subscriber('camera/odom/sample', Odometry, odom_callback, queue_size=10)
    rospy.Subscriber('scan', LaserScan, lidar_callback, queue_size=10)
    rospy.Subscriber('mcp_path', Path, mcp_callback, queue_size=10)
    
    srv = Server(PurePursuitConfig, callback)
    
    
   
    
    while not rospy.is_shutdown():
        pub_collision.publish(Bool(collision))
           
        if (not collision and detect_collision) or not(detect_collision):
            # Get steering angle
            if len(course_x) != 0 :
                speed, steering_angle, target_ind, alpha, R = pure_pursuit_control(state, course_x, course_y)
                if target_ind is not None:
                    
                    # display look ahead point
                    publish_marker(pub_marker, course_x[target_ind], course_y[target_ind],alpha+state.yaw, collide=collision)
                    publish_collision_marker(pub_collision_marker, alpha, collide=collision)
                    
                    
                   
                    # display the path to the look ahead point
                    msg_path = Path()
                    msg_path.header.frame_id = 'base_link'
                    
                    angles = np.linspace(0,2*abs(alpha),50)
                    for i,angle in enumerate(angles) :
                        pose = PoseStamped()
                        
                        pose.header.frame_id = "base_link"
                        pose.header.seq = i
                        
                        pose.pose.position.x = R * np.sin(angle)
                        pose.pose.position.y = R * (1 - np.cos(angle)) * np.sign(alpha)
                        
                        msg_path.poses.append(pose)
                        
                    pub_path.publish(msg_path)
            else:
                steering_angle = 0.0
                target_ind = 0
                speed = 0

            # Publish steering msg
            msg = Twist()
            msg.angular.z = steering_angle
            msg.linear.x = speed
            pub.publish(msg)
            
            pub_full_path.publish(msg_path1)

        rate.sleep()

    
if __name__ == '__main__':
    try:
        # init node
        rospy.init_node('pure_pursuit_with_avoidance')
        rate = rospy.Rate(15) # hz
        rospack = rospkg.RosPack()
       
        main()
    except rospy.ROSInterruptException:
        pass
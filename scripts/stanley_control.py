#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May 24 09:24:04 2022

@author: student
"""
'''
* ROS Stanley Steering node ************************

 Stanley Steering path tracker for following a path.

 References:
 http://isl.ecst.csuchico.edu/DOCS/darpa2005/DARPA%202005%20Stanley.pdf
 https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf

 Stanley Steering algorithm adapted from:
 https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathTracking/stanley_controller
 By Jon Eivind Stranden @ NTNU 2019
****************************************************
'''

import rospy
import math
import os
import numpy as np
from std_msgs.msg import Float64, String
from geometry_msgs.msg import PointStamped, Twist, PoseWithCovarianceStamped, Pose, Point, Vector3, Quaternion, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA, String
# from ens_voiture_autonome.msg import DS4
import tf
import rospkg
import pickle
from dynamic_reconfigure.server import Server
from ens_voiture_autonome.cfg import StanleyControllerConfig
from ens_vision import path_tools


### Tuning settings #################

k = 0.1  # Cross track error gain
k_soft = 0.5
k_steer = 0.5
wheelbase_length = 0.257  # [m]
max_steering_angle = 0.30 # [rad]

#####################################

# Path points with yaw storage
course_x = []
course_y = []
course_yaw = []
course_speed = []

old_steer = 0

def callback(config, level):
    global k
    global k_soft
    global k_steer
    global k_obstacle
    global look_ahead_dist_obstacle
    global wheelbase_length
    global max_steering_angle
    global radius
    global detect_collision


    k = config['k_sc']  # Cross track error gain
    k_soft = config['k_soft']  # softning constant, permitting control to be arbitrarily soft at low speeds
    k_steer = config['k_steer']  # servomotor correction (derivative term)

    k_obstacle = config['k_obstacle'] # Look forward gain (Dynamic look-ahead)
    look_ahead_dist_obstacle = config['look_ahead_dist_obstacle']  # Look-ahead distance

    wheelbase_length = config['wheelbase_length']  # wheel base of vehicle [m]
    max_steering_angle = config['max_steering_angle'] # rad
    radius = config['radius'] # m

    detect_collision = config['detect_collision']




    return config

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

# initial state
state = State(x=0.0, y=0.0, yaw=0.0, v=0.0)


def stanley_control(state, course_x, course_y, course_yaw):
    # adapted from: https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathTracking/stanley_controller

    global old_steer



    current_target_ind, error_front_axle = calc_target_index(state, course_x, course_y)

    # theta_e corrects the heading error
    theta_e = normalize_angle(course_yaw[current_target_ind] - state.yaw)


    # theta_d corrects the cross track error
    theta_d = np.arctan2(k * error_front_axle, state.v + k_soft)

    # Steering control
    delta = theta_e + theta_d


    # Add stability
    delta += k_steer*(delta - old_steer)

    # Cap max steering wheel angle
    delta = np.clip(delta, -max_steering_angle, max_steering_angle)

    old_steer = delta

    speed = course_speed[current_target_ind]

    return speed, delta, current_target_ind


def normalize_angle(angle):
    # adapted from: https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathTracking/stanley_controller

    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


def calc_target_index(state, course_x, course_y):
    # adapted from: https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathTracking/stanley_controller

    # Calc front axle position
    front_x = state.x + wheelbase_length * np.cos(state.yaw)
    front_y = state.y + wheelbase_length * np.sin(state.yaw)

    # Search nearest point index
    dx = [front_x - icx for icx in course_x]
    dy = [front_y - icy for icy in course_y]
    d = [np.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
    error_front_axle = min(d)
    target_idx = d.index(error_front_axle)
    # print(target_idx)

    target_yaw = normalize_angle(np.arctan2(
        front_y - course_y[target_idx], front_x - course_x[target_idx]) - state.yaw)
    if target_yaw > 0.0:
        error_front_axle = - error_front_axle


    return target_idx, error_front_axle


def update_state_callback(data):

    global state

    state.x = data.pose.pose.position.x
    state.y = data.pose.pose.position.y

    # print(state.x, state.y)

    # Convert quaternions to euler to get yaw
    orientation_list = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    _, _, yaw = euler_from_quaternion(orientation_list)
    state.yaw = yaw
'''
def load_path_callback(msg):

    global course_x
    global course_y
    global course_speed
    global course_yaw
    global pub_full_path
    global msg_path1

    msg=msg.data.split(" ")
    if (msg[0]=="load" and len(msg)>1):
        path_x = []
        path_y = []
        course_speed = []
        path_yaw = []

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
                orientation_list = orientations[i]
                _, _, yaw = euler_from_quaternion(orientation_list)
                path_yaw.append(yaw)

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
            n = len(raceline)
            for i, position in enumerate(raceline):
                path_x.append(position[0])
                path_y.append(position[1])
                yaw = np.arctan2(
                    raceline[(i+1)%n,1] - raceline[i,1], raceline[(i+1)%n,0] - raceline[i,0])
                path_yaw.append(yaw)
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
        course_yaw = path_yaw
        rospy.loginfo('traj loaded')
'''

def load_path_callback(msg):

    global course_x
    global course_y
    global course_yaw
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

def path_callback(data):

    global course_x
    global course_y
    global course_yaw
    global course_speed

    path_x = []
    path_y = []
    path_yaw = []

    for i, pose in enumerate(data.poses):
        path_x.append(data.poses[i].pose.position.x)
        path_y.append(data.poses[i].pose.position.y)
        orientation_list = [data.poses[i].pose.orientation.x, data.poses[i].pose.orientation.y, data.poses[i].pose.orientation.z, data.poses[i].pose.orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        path_yaw.append(yaw)

    course_x = path_x
    course_y = path_y
    course_yaw = path_yaw
    course_speed = np.ones(len(course_x))


def vel_callback(data):

    global state

    state.v = data.linear.x


def odom_callback(data):

    global state

    state.v = -data.twist.twist.linear.x
    # print(state.v)

def publish_marker(pub, x, y, theta):
    marker = Marker()
    marker.header=Header(frame_id='map')
    marker.type=Marker.CUBE
    marker.scale=Vector3(0.3, 0.1, 0.1)
    a,b,c,d = quaternion_from_euler(0,0,theta)
    marker.pose=Pose(Point(x,y,0), Quaternion(a,b,c,d))
    marker.color = ColorRGBA(1,0,1,1)
    marker.lifetime = rospy.Duration(100)

    pub.publish(marker)

def main():
    global pub_full_path
    # init node
    rospy.init_node('stanley_controller')
    rate = rospy.Rate(15) # hz

    # Publish
    pub = rospy.Publisher('stanley_control_cmd', Twist, queue_size=10)
    pub_full_path = rospy.Publisher('loaded_path', Path, queue_size=10)

    # Get current state of truck
    rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, update_state_callback, queue_size=10)
    # rospy.Subscriber('trajectory', Path, path_callback, queue_size=10)
    rospy.Subscriber('syscommand', String, load_path_callback, queue_size=10)
    # rospy.Subscriber('vel', Twist, vel_callback, queue_size=10)
    rospy.Subscriber('camera/odom/sample', Odometry, odom_callback, queue_size=10)

    srv = Server(StanleyControllerConfig, callback)

    pub_marker = rospy.Publisher('stanley_controller_look_ahead', Marker, queue_size=10)

    while not rospy.is_shutdown():

        # Get steering angle
        if len(course_x) != 0:
            speed, steering_angle, target_ind = stanley_control(state, course_x, course_y, course_yaw)
            publish_marker(pub_marker, course_x[target_ind], course_y[target_ind], course_yaw[target_ind])
        else:
            steering_angle = 0.0
            target_ind = 0
            speed = 0.0

        # Publish steering msg
        msg = Twist()
        msg.angular.z = steering_angle
        msg.linear.x = speed
        pub.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        rospack = rospkg.RosPack()
        main()
    except rospy.ROSInterruptException:
        pass
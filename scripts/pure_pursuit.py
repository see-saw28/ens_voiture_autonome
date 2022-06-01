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
from std_msgs.msg import Float64, String
from geometry_msgs.msg import PointStamped, Twist, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Path, Odometry
# from ens_voiture_autonome.msg import DS4
import tf
import rospkg
import pickle

### Tuning settings #############################

k = 0.4  # Look forward gain (Dynamic look-ahead)
look_ahead_dist = 0.6  # Look-ahead distance
wheelbase_length = 0.257  # wheel base of vehicle [m]
max_steering_angle = 0.30 # rad
vitesse_max=2

#################################################

# Path point storage
course_x = []
course_y = []

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

# initial state
state = State(x=0.0, y=0.0, yaw=0.0, v=0.0)


def pure_pursuit_control(state, course_x, course_y):
    # adapted from: https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathTracking/pure_pursuit

    # Get waypoint target index
    ind, ind_car = calc_target_index(state, course_x, course_y)

    if ind < len(course_x):
        target_x = course_x[ind]
        target_y = course_y[ind]
    else:
        target_x = course_x[-1]
        target_y = course_y[-1]
        ind = len(course_x) - 1

    # Calc angle alpha between the vehicle heading vector and the look-ahead vector
    alpha = math.atan2(target_y - state.y, target_x - state.x) - (state.yaw)

    if state.v < 0:  # back
        alpha = math.pi - alpha

    # Dynamic look-ahead distance
    dyn_look_ahead_dist = k * abs(state.v) + look_ahead_dist

    # Calc steering wheel angle delta
    delta = math.atan2(2.0 * wheelbase_length * math.sin(alpha) / dyn_look_ahead_dist, 1.0)

    # Cap max steering wheel angle
    delta = np.clip(delta, -max_steering_angle, max_steering_angle)
    
  
    speed = course_speed[ind_car]
    
    return speed, delta, ind


def calc_target_index(state, course_x, course_y):
    # adapted from: https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathTracking/pure_pursuit

    k = rospy.get_param('joy_to_cmd_vel/k_pp')
    look_ahead_dist = rospy.get_param('joy_to_cmd_vel/look_ahead_dist')

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
            print('too far away')
            break
          
    
    return ind%len(course_x), ind_car


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

    msg=msg.data.split(" ")
    if (msg[0]=="load" and len(msg)>1):
        path_x = []
        path_y = []
        course_speed = []
        
    
       
        f = open(rospack.get_path('ens_vision')+f'/paths/{msg[1]}.pckl', 'rb')
        marker,speeds,orientations,cmd_speeds = pickle.load(f)
        f.close()
        for i, pose in enumerate(marker.points):
            path_x.append(pose.x)
            path_y.append(pose.y)
            course_speed.append(cmd_speeds[i])

        course_x = path_x
        course_y = path_y
        
        

def vel_callback(data):

    global state
    global vitesse_max
    
    state.v = data.linear.x
    
def odom_callback(data):
    
    global state
    state.v = -data.twist.twist.linear.x
    # print(state.v)


def main():
   

    # Publish
    pub = rospy.Publisher('pure_pursuit_cmd', Twist, queue_size=100)

    # Get current state of truck
    #rospy.Subscriber('pf/viz/inferred_pose', PoseStamped, update_state_callback, queue_size=10)
    rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, update_state_callback, queue_size=100)
    rospy.Subscriber('syscommand', String, load_path_callback, queue_size=10)
    # rospy.Subscriber('vel', Twist, vel_callback, queue_size=10)
    rospy.Subscriber('camera/odom/sample', Odometry, odom_callback, queue_size=10)

    # Start a TF broadcaster
    tf_br = tf.TransformBroadcaster()
    
    while not rospy.is_shutdown():

        # Get steering angle
        if len(course_x) != 0:
            speed, steering_angle, target_ind = pure_pursuit_control(state, course_x, course_y)
            if target_ind is not None:
                tf_br.sendTransform((course_x[target_ind], course_y[target_ind], 0.0), quaternion_from_euler(0.0, 0.0, 3.1415), rospy.Time.now() , "look_ahead_point", "map")
        else:
            steering_angle = 0.0
            target_ind = 0
            speed = 0

        # Publish steering msg
        msg = Twist()
        msg.angular.z = steering_angle
        msg.linear.x = speed
        pub.publish(msg)

        rate.sleep()

    
if __name__ == '__main__':
    try:
        # init node
        rospy.init_node('pure_pursuit')
        rate = rospy.Rate(100) # hz
        rospack = rospkg.RosPack()
       
        main()
    except rospy.ROSInterruptException:
        pass
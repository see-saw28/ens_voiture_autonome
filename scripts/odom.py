#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat May 21 20:47:08 2022


Inspired by Jon Eivind Stranden @ NTNU 2019

@author: student
"""

import rospy
from math import tan, cos, sin
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf

# variables
velocity = 0
angular_velocity = 0
init_x_pos = 0.0
init_y_pos = 0.0
init_yaw = 0.0
delta_yaw = 0.0
yaw = 0.0
new_init_pose = False


def vel_callback(data):
    
    global velocity

    # Get speed and steering values from car controller
    velocity = data.linear.x


def imu_callback(data):

    global yaw
    global angular_velocity

    # Get angular velocity in z-direction
    roll, pitch, yaw = tf.transformations.euler_from_quaternion((data.orientation.x, data.orientation.y, data.orientation.z,data.orientation.w))
    angular_velocity = data.angular_velocity.z
    

def initpose_callback(data):

    global init_x_pos
    global init_y_pos
    global delta_yaw
    global new_init_pose

    # Get position
    init_x_pos = data.pose.pose.position.x
    init_y_pos = data.pose.pose.position.y

    # Get yaw
    orientation_list = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    roll, pitch, yaw_ = tf.transformations.euler_from_quaternion(orientation_list)   
    delta_yaw = yaw - yaw_

    new_init_pose = True


def main():

    global velocity
    global angular_velocity
    global init_x_pos
    global init_y_pos
    global delta_yaw
    global new_init_pose

    print ("Running odom_esc node...")

    # Publisher
    odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
    tf_br = tf.TransformBroadcaster()

    # Subscriptions
    rospy.Subscriber("vel", Twist, vel_callback)
    rospy.Subscriber("imu/data", Imu, imu_callback)    
    # rospy.Subscriber("initialpose", PoseWithCovarianceStamped, initpose_callback)

    odom_msg = Odometry()

    # Set up node
    rospy.init_node('ackermann_odom', anonymous=True)
    rate = rospy.Rate(30) # 10hz

    last_stamp = rospy.Time.now()

    x_ = 0.0
    y_ = 0.0
    global yaw
    
    

    while not rospy.is_shutdown():

        if new_init_pose is True:
            # Update init pose
            x_ = init_x_pos
            y_ = init_y_pos
            yaw = init_yaw
            new_init_pose = False

       

        # Delta time
        dt = rospy.Time.now() - last_stamp

        yaw_ = yaw + delta_yaw

        # Calculate speed in x and y direction
        x_dot = velocity * cos(yaw_)
        y_dot = velocity * sin(yaw_)

        # Calculate x and y position
        x_ += x_dot * dt.to_sec()
        y_ += y_dot * dt.to_sec()

        # Save timestamp
        last_stamp = rospy.Time.now()

        # Make odometry msg
        odom_msg.header.frame_id = "odom"
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.child_frame_id = "base_link"

        # Position and orientation
        odom_msg.pose.pose.position.x = x_
        odom_msg.pose.pose.position.y = y_
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = sin(yaw_/2.0)
        odom_msg.pose.pose.orientation.w = cos(yaw_/2.0)

        # Uncertainty in position
        odom_msg.pose.covariance[0] = 0.5 # <x
        odom_msg.pose.covariance[7]  = 0.5 # <y
        odom_msg.pose.covariance[35] = 0.4 # <yaw

        # Velocity
        odom_msg.twist.twist.linear.x = velocity
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = angular_velocity
        odom_msg.twist.covariance[0] = 0.5 # <x
        odom_msg.twist.covariance[7]  = 0.5 # <y
        odom_msg.twist.covariance[35] = 0.4 # <yaw

        # Publish msgs
        odom_pub.publish(odom_msg)

        # Odom transform to
        tf_br.sendTransform((x_, y_, 0.0), (0.0, 0.0, sin(yaw_/2.0), cos(yaw_/2.0)), rospy.Time.now() , "base_link", "odom")
        tf_br.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now() , "odom", "map")

        rate.sleep()
        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
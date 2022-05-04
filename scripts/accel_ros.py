#!/usr/bin/env python3

import time
import rospy
import board
import adafruit_bno055
import math
import numpy as np
import os
os.environ['ROS_MASTER_URI']='http://172.20.10.9:11311'
os.environ['ROS_IP']='172.20.10.8'

from sensor_msgs.msg import Imu

i2c = board.I2C()
sensor = adafruit_bno055.BNO055_I2C(i2c)

def talker():
	pub = rospy.Publisher('imu/data', Imu, queue_size=10)
	rospy.init_node('ros_imu', anonymous=False)
	rate = rospy.Rate(10)

	
	
	msg = Imu()

	while not rospy.is_shutdown():

		gy,gx,gz=sensor.gyro
		q1,q0,q2,q3=sensor.quaternion
		ay,ax,az=sensor.linear_acceleration
		gy=-gy
		ay=-ay
		q1=-q1
		msg.header.stamp = rospy.get_rostime()

		


		
	

		#Fill message
		msg.orientation.x = q1
		msg.orientation.y = q2
		msg.orientation.z = q3
		msg.orientation.w = q0
		msg.orientation_covariance[0] = q1 * q1
		msg.orientation_covariance[0] = q2 * q2
		msg.orientation_covariance[0] = q3 * q3		

		msg.angular_velocity.x = gx
		msg.angular_velocity.y = gy
		msg.angular_velocity.z = gz
		msg.angular_velocity_covariance[0] = gx*gx
		msg.angular_velocity_covariance[4] = gy*gy
		msg.angular_velocity_covariance[8] = gz*gz
		msg.linear_acceleration.x = ax
		msg.linear_acceleration.y = ay
		msg.linear_acceleration.z = az
		msg.linear_acceleration_covariance[0] = ax*ax
		msg.linear_acceleration_covariance[4] = ay*ay
		msg.linear_acceleration_covariance[8] = az*az
		
		pub.publish(msg)

		rate.sleep()

if __name__ == '__main__':
	try:
        	talker()
	except rospy.ROSInterruptException:
        	pass

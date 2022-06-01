#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May 31 13:29:53 2022

@author: student
"""
#!/usr/bin/env python3



import time
import rospy
import numpy as np
import os
from getkey import getkey, keys


os.environ['ROS_MASTER_URI']='http://192.168.1.174:11311'
os.environ['ROS_IP']='192.168.1.149'

import RPi.GPIO as GPIO
import sys
dir=33
prop=32


GPIO.setmode(GPIO.BOARD)
GPIO.setup(prop, GPIO.OUT)
GPIO.output(prop, GPIO.LOW)
GPIO.setup(dir, GPIO.OUT)
GPIO.output(dir, GPIO.LOW)



p = GPIO.PWM(dir,50)
q = GPIO.PWM(prop,50)

p.start(5)
q.start(7.5)

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Path, Odometry

x = 0.0
z = 0.0

velocity_mes = 0.0

old_x=1
def callback(msg):
#	print('test')
	global old_x
	global x
	global z
	x = msg.linear.x
	z = msg.angular.z
#	print(x,z)
	
def odom_callback(data):
    
    global velocity_mes
    velocity_mes = -data.twist.twist.linear.x
    msg = Float32()
    msg.data = velocity_mes
    pub_vel.publish(msg)
	

def asservissement():
	global pub_vel
	error_integral = 0
	old_error = 0
	
	Kp = 3
	Ki = 1
	Kd = 1
        
	
	rospy.init_node('command', anonymous=False)

	pub = rospy.Publisher('/pwm',Float32,queue_size=5)
	pub_vel = rospy.Publisher('/vel',Float32,queue_size=5)
	rospy.Subscriber("/cmd_vel", Twist, callback) #/cmd_vel /key_vel /ps3_vel /joy
	rospy.Subscriber('camera/odom/sample', Odometry, odom_callback, queue_size=10)
	
	frequency = 1000

	rate = rospy.Rate(frequency) 
    
    
    #msg.pose = Pose(Point(x, y, 0.),Quaternion(*tf.transformations.quaternion_from_euler(roll, pitch, yaw)))
    
	while not rospy.is_shutdown():
		try : 
			
			error = x - velocity_mes
			
			error_integral += error/frequency
			
			error_derivative = (error-old_error)/frequency
			
			speed = Kp*error + Ki*error_integral + Kd*error_derivative
			
			old_error = error
			

			if (speed>=0 and speed<2.4):
			    pwm=6.99-(0.1569*speed + 0.0032)
		#	elif (x<1.4): 

		#	    pwm=6.95-(0.1569*1.4+0.0032)
			elif (speed>2.4):
			    pwm=6.95-(0.4037*speed-0.5757)


			elif (x<0):
			    pwm=7.5-0.5*x

		#	print(z)
			p.ChangeDutyCycle(5.2-z/0.30)
			q.ChangeDutyCycle(pwm)
			msg = Float32()     
			msg.data = pwm
			pub.publish(msg)
			
			rate.sleep()
			
		except rospy.ROSInterruptException:
		    
			pass
			
	
if __name__ == '__main__':
	try:
		asservissement()
		q.stop()
		p.stop()
		GPIO.cleanup()
		print("out")


	except  rospy.ROSInterruptException:
		q.stop()
		p.stop()
		GPIO.cleanup()
		print("out")
		pass

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

pub_freq = 25.0
rospy.init_node('command', anonymous=False)
rate = rospy.Rate(pub_freq)
pub = rospy.Publisher('/pwm',Float32,queue_size=5)

pub_vel = rospy.Publisher('/vel',Float32,queue_size=5)

def odom_callback(data):
    
    global velocity_mes
    velocity_mes = -data.twist.twist.linear.x
    msg = Float32()
    msg.data = velocity_mes
    pub_vel.publish(msg)

wheelbase=0.3
print(wheelbase)
old_x=1
def callback(msg):
#	print('test')
	global old_x
	x = msg.linear.x
	z = msg.angular.z
#	print(x,z)
	vit=0.5
	x=x/np.cos(z)

	if x>=0:
		pwm = 6.88-0.20*x
# 	if (x>=0 and x<2.4):
# 	    pwm=6.95-(0.1569*x + 0.0032)
# #	elif (x<1.4): 

# #	    pwm=6.95-(0.1569*1.4+0.0032)
# 	elif (x>2.4):
# 	    pwm=6.95-(0.4037*x-0.5757)
# 	elif (x<0 and old_x>=0):
# 		pwm=7.52
# 		print('back')
# 		for i in range(10):
# 			q.ChangeDutyCycle(pwm+0.02*i)
# 			time.sleep(0.01)
# 	   



	elif (x<0):
        
		if velocity_mes>0.1:
			pwm = 6.5
		elif velocity_mes>0:
			pwm = 7.52
		else:

			pwm=7.5-vit*x
	old_x=x
#	print(z)
	p.ChangeDutyCycle(5.2-z/0.30)
	q.ChangeDutyCycle(pwm)
	msg = Float32()     
	msg.data = pwm
	pub.publish(msg)

def listener_and_pub():
	rospy.Subscriber("/cmd_vel", Twist, callback) #/cmd_vel /key_vel /ps3_vel /joy
	rospy.Subscriber('camera/odom/sample', Odometry, odom_callback, queue_size=10)
	rospy.spin()
	
if __name__ == '__main__':
	try:
		listener_and_pub()
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

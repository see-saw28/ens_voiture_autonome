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


x = 0.0
z = 0.0

pub_freq = 25.0
rospy.init_node('command', anonymous=False)
rate = rospy.Rate(pub_freq)

wheelbase=0.3
print(wheelbase)
old_x=1
def callback(msg):
#	print('test')
	global old_x
	x = msg.linear.x
	z = msg.angular.z
	print(x,z)
	vit=0.5
	x=x/np.cos(z)

	if (x>=0 and x<2.4):
	    pwm=6.95-(0.1569*x + 0.0032)
#	elif (x<1.4): 

#	    pwm=6.95-(0.1569*1.4+0.0032)
	elif (x>2.4):
	    pwm=6.95-(0.4037*x-0.5757)
	elif (x<0 and old_x>=0):
	    pwm=7.5
	    q.ChangeDutyCycle(pwm)
	    print('back')
	    time.sleep(0.5)
	    q.ChangeDutyCycle(pwm)
	    time.sleep(0.5)
	    q.ChangeDutyCycle(pwm)
	    time.sleep(1.0)


	elif (x<0):
	    pwm=7.7-vit*x
	old_x=x
	print(pwm)
	p.ChangeDutyCycle(5.2-z/0.30)
	q.ChangeDutyCycle(pwm)

	

# 	rate.sleep()

def listener_and_pub():
	rospy.Subscriber("/cmd_vel", Twist, callback) #/cmd_vel /key_vel /ps3_vel /joy
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

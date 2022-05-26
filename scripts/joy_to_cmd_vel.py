#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May 23 09:11:38 2022

@author: student
"""
import rospy
import os
from ens_voiture_autonome.msg import DS4
from geometry_msgs.msg import Twist


rospy.init_node('joy_to_cmd_vel', anonymous=False)
pub = rospy.Publisher('/cmd_vel',Twist,queue_size=5)

vitesse_max=2
angle_max=0.30

PP_steering = 0
SC_steering = 0

driving_mode = 0

driving_mode_dict = {0:'MANUAL',1:'PURE PURSUIT',2:'STANLEY CONTROLLER'}

def callback(data):
    global vitesse_max
    global driving_mode
    global speed_mode
    
    vx = -data.AXIS_RIGHT_STICK_Y*vitesse_max
    msg = Twist()
    msg.linear.x = -data.AXIS_RIGHT_STICK_Y*vitesse_max
    
    if driving_mode == 0:
       msg.angular.z = -data.AXIS_LEFT_STICK_X*angle_max
       
    elif driving_mode == 1 :
        
        if msg.linear.x > 0:
            msg.angular.z = PP_steering
            
        else :
            msg.angular.z = -PP_steering
        
    elif driving_mode == 2 :
        msg.angular.z = SC_steering
        
        if speed_mode == 0:
            msg.linear.x = -data.AXIS_RIGHT_STICK_Y*speed_max
            
            if msg.linear.x < 0:
                msg.angular.z = -SC_steering
            pub.publish(msg)    
    
       
    pub.publish(msg)
    
    if (data.HAT_Y == 1 and data.RE_HAT_Y):
        vitesse_max = min(vitesse_max+1, 6)
        print(f'vitesse max :{vitesse_max} m/s')
        
    elif (data.HAT_Y == -1 and data.RE_HAT_Y):
        vitesse_max = max(vitesse_max-1,1)
        print(f'vitesse max :{vitesse_max} m/s')
        
    if (data.HAT_X == 1 and data.RE_HAT_X):
        driving_mode = min(driving_mode+1, 2)
        print(f'MODE {driving_mode_dict[driving_mode]}')
        
    elif (data.HAT_X == -1 and data.RE_HAT_X):
        driving_mode = max(driving_mode-1,0)
        print(f'MODE {driving_mode_dict[driving_mode]}')
        

    
    
def pure_pursuit_callback(data):
    global PP_steering
    
    PP_steering = data.angular.z
    
def stanley_control_callback(data):
    global SC_steering
    global SC_speed
    
    SC_speed = data.linear.x
    SC_steering = data.angular.z
    
    msg = Twist()
    
    if driving_mode == 2 :
        
        msg.angular.z = PP_steering
            
        if speed_mode == 1:
            msg.linear.x = speed_max
            
        elif speed_mode == 2:
            msg.linear.x = speed_max*np.exp(-SC_steering**2/sigma2)
            
        elif speed_mode == 3:
            if abs(PP_steering)>binary_steering_threshold:
                msg.linear.x = speed_max*corner_speed_coef
                
            else:
                msg.linear.x = speed_max
                
        elif speed_mode == 4:
            msg.linear.x = PP_speed
        
        if speed_mode != 0:
            pub.publish(msg)

def listener_and_pub():
    rospy.Subscriber("/DS4_input", DS4, callback)
    rospy.Subscriber("/pure_pursuit_cmd", Twist, pure_pursuit_callback)
    rospy.Subscriber("/stanley_control_cmd", Twist, stanley_control_callback)
    rospy.spin()
	
if __name__ == '__main__':
	try:
		listener_and_pub()
		


	except  rospy.ROSInterruptException:
		
		pass
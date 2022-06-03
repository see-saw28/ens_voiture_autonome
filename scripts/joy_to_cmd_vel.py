#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May 23 09:11:38 2022

@author: student
"""
import rospy
import os
import time
from ens_voiture_autonome.msg import DS4
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from dynamic_reconfigure.server import Server
from ens_voiture_autonome.cfg import ControllerConfig



rospy.init_node('joy_to_cmd_vel', anonymous=False)
pub = rospy.Publisher('/cmd_vel',Twist,queue_size=5)

speed_max=2
steer_max=0.30

corner_speed_coef = 0.5
sigma2 = -steer_max**2/np.log(corner_speed_coef)
binary_steering_threshold = 0.1

PP_spped = 0
PP_steering = 0

SC_speed = 0
SC_steering = 0

MB_STEERING = 0

speed_mode = 0
driving_mode = 0

aeb = False
breaking = False
aeb_count = 0
breaking_time = 0

velocity_mes = 0

driving_mode_dict = {0:'MANUAL',1:'PURE PURSUIT',2:'STANLEY CONTROLLER',3:'MOVE BASE',4:'DWA'}
speed_mode_dict = {0:'MANUAL',1:'CONSTANT',2:'GAUSSIAN', 3:'BINARY',4:'RECORDED'}

print(f'MODE {driving_mode_dict[driving_mode]} with {speed_mode_dict[speed_mode]} SPEED')

def callback(data):
    global speed_max
    global driving_mode
    global speed_mode
    global breaking
    
    
    if data.RE_SHARE :
        driving_mode = 0
        speed_mode =0
        print(f'MODE {driving_mode_dict[driving_mode]} with {speed_mode_dict[speed_mode]} SPEED')
    
    msg = Twist()
    
    
    
    if breaking and time.time()-breaking_time < 0.5:
        msg.linear.x = -3
        msg.angular.z = 0
        pub.publish(msg)
        
    elif breaking and time.time()-breaking_time > 0.5:
        breaking = False
        
    elif aeb :
        msg.linear.x = -3
        msg.angular.z = 0
        pub.publish(msg)
        
    elif driving_mode == 0:
        if data.BUTTON_SQUARE :
            msg.linear.x = speed_max
        else :
            msg.linear.x = -data.AXIS_RIGHT_STICK_Y*speed_max
        msg.angular.z = -data.AXIS_LEFT_STICK_X*steer_max
        pub.publish(msg)
       
    elif driving_mode == 1 and speed_mode == 0 :
        
        
        msg.angular.z = PP_steering
        if data.BUTTON_SQUARE :
            msg.linear.x = speed_max
        else :
            msg.linear.x = -data.AXIS_RIGHT_STICK_Y*speed_max
        pub.publish(msg)   
       
            
            
        
    elif driving_mode == 2 and speed_mode == 0 :
        msg.angular.z = SC_steering
        msg.linear.x = -data.AXIS_RIGHT_STICK_Y*speed_max
            
        # if msg.linear.x < 0:
        #     msg.angular.z = -SC_steering
        pub.publish(msg)    
		
    elif driving_mode == 3 and speed_mode == 0 :
        
        
        msg.angular.z = MB_STEERING
        msg.linear.x = -data.AXIS_RIGHT_STICK_Y*speed_max
        pub.publish(msg) 
        
    elif driving_mode == 4 and speed_mode == 0 :
        
        
        msg.angular.z = DWA_steering
        msg.linear.x = -data.AXIS_RIGHT_STICK_Y*speed_max
        pub.publish(msg) 
    
    
       
    
    
    if (data.RE_R1):
        speed_max = min(speed_max+0.5, 6)
        print(f'vitesse max :{speed_max} m/s')
        
    elif (data.RE_L1):
        speed_max = max(speed_max-0.5,1)
        print(f'vitesse max :{speed_max} m/s')
        
    if (data.HAT_X == 1 and data.RE_HAT_X):
        driving_mode = min(driving_mode+1, len(driving_mode_dict)-1)
        print(f'MODE {driving_mode_dict[driving_mode]} with {speed_mode_dict[speed_mode]} SPEED')
        
    elif (data.HAT_X == -1 and data.RE_HAT_X):
        driving_mode = max(driving_mode-1,0)
        print(f'MODE {driving_mode_dict[driving_mode]} with {speed_mode_dict[speed_mode]} SPEED')
        
    if (data.HAT_Y == 1 and data.RE_HAT_Y):
        speed_mode = min(speed_mode+1, 4)
        print(f'MODE {driving_mode_dict[driving_mode]} with {speed_mode_dict[speed_mode]} SPEED')
        
    elif (data.HAT_Y == -1 and data.RE_HAT_Y):
        speed_mode = max(speed_mode-1,0)
        print(f'MODE {driving_mode_dict[driving_mode]} with {speed_mode_dict[speed_mode]} SPEED')
          
        

    
    
def pure_pursuit_callback(data):
    global PP_steering
    global PP_speed
    global breaking
    
    PP_speed = data.linear.x
    PP_steering = data.angular.z
    
    msg = Twist()
    
    
    
    if driving_mode == 1 :
        
        msg.angular.z = PP_steering
            
        if breaking and time.time()-breaking_time < 1.0:
            msg.linear.x = -3
            # msg.angular.z = 0
            pub.publish(msg)
            
        elif breaking and time.time()-breaking_time > 1.0:
            breaking = False
        
        elif speed_mode == 1:
            msg.linear.x = speed_max
            
        elif speed_mode == 2:
            msg.linear.x = speed_max*np.exp(-PP_steering**2/sigma2)
            
        elif speed_mode == 3:
            if abs(PP_steering)>binary_steering_threshold:
                msg.linear.x = speed_max*corner_speed_coef
                
            else:
                msg.linear.x = speed_max
                
        elif speed_mode == 4:
            msg.linear.x = PP_speed
        
        if speed_mode != 0:
            pub.publish(msg)
    
def stanley_control_callback(data):
    global SC_steering
    global SC_speed
    
    SC_speed = data.linear.x
    SC_steering = data.angular.z
    
    msg = Twist()
    
    if driving_mode == 2 :
        
        msg.angular.z = SC_steering
            
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
            msg.linear.x = SC_speed
        
        if speed_mode != 0:
            pub.publish(msg)
            
def aeb_callback(data):
    global aeb
    global breaking
    global breaking_time
    global aeb_count
    
    aeb_count_threshold = 3
    
    if data.data and aeb_count <aeb_count_threshold:
        
        aeb_count+=1
        print(aeb_count)
        
    elif data.data and aeb_count >=aeb_count_threshold:
        breaking = True
        print('emergency break')
        breaking_time = time.time()
        
    else :
        aeb_count = 0
        
    
        
    aeb = data.data
	
def move_base_callback(data):
	global MB_STEERING
	
	MB_STEERING = data.angular.z
	
	if driving_mode == 3 :
		if speed_mode != 0:
			pub.publish(data)
            
   
def dwa_callback(data):
    global DWA_steering
    global DWA_speed
    global breaking
    
    DWA_speed = data.linear.x
    DWA_steering = data.angular.z
    
    msg = Twist()
    
    
    
    if driving_mode == 4 :
        
        msg.angular.z = DWA_steering
            
        if breaking and time.time()-breaking_time < 1.0:
            msg.linear.x = -3
            # msg.angular.z = 0
            pub.publish(msg)
            
        elif breaking and time.time()-breaking_time > 1.0:
            breaking = False
        
        elif speed_mode == 1:
            msg.linear.x = speed_max
            
        elif speed_mode == 2:
            msg.linear.x = speed_max*np.exp(-DWA_steering**2/sigma2)
            
        elif speed_mode == 3:
            if abs(DWA_steering)>binary_steering_threshold:
                msg.linear.x = speed_max*corner_speed_coef
                
            else:
                msg.linear.x = speed_max
                
        elif speed_mode == 4:
            msg.linear.x = DWA_speed
        
        if speed_mode != 0:
            pub.publish(msg)            

def odom_callback(data):
    
    global velocity_mes
    
    velocity_mes = -data.twist.twist.linear.x
    msg = Float32()
    msg.data = velocity_mes
    pub_vel.publish(msg)
    
def server_callback(config, level):
    global corner_speed_coef
    global binary_steering_threshold
    global sigma2 
            
    corner_speed_coef = config['corner_speed_coef']
    sigma2 = -steer_max**2/np.log(corner_speed_coef)
    binary_steering_threshold = config['binary_steering_threshold']
    
    
    return config

def listener_and_pub():
    global pub_vel
    
    rospy.Subscriber("/DS4_input", DS4, callback)
    rospy.Subscriber("/pure_pursuit_cmd", Twist, pure_pursuit_callback)
    rospy.Subscriber("/stanley_control_cmd", Twist, stanley_control_callback)
    rospy.Subscriber("/move_base_cmd", Twist, move_base_callback)
    rospy.Subscriber("/dwa_cmd", Twist, dwa_callback)
    rospy.Subscriber("/AEB", Bool, aeb_callback)
    rospy.Subscriber("/camera/odom/sample", Odometry, odom_callback)
    srv = Server(ControllerConfig, server_callback)
    pub_vel = rospy.Publisher('/vel',Float32,queue_size=5)
    rospy.spin()
	
if __name__ == '__main__':
	try:
		listener_and_pub()
		


	except  rospy.ROSInterruptException:
		
		pass
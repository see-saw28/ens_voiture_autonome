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
obstacle_speed_coef = 0.5
sigma2 = -steer_max**2/np.log(corner_speed_coef)
binary_steering_threshold = 0.1

PP_spped = 0
PP_steering = 0

SC_speed = 0
SC_steering = 0

MB_steering = 0

FTG_steering = 0

LSC_steering = 0

DWA_steering = 0

speed_mode = 0
driving_mode = 0

aeb = False
breaking = False
aeb_count = 0
breaking_time = 0

velocity_mes = 0

escape_distance = 0.8

collision = False

driving_mode_dict = {0:'MANUAL',1:'PURE PURSUIT',2:'STANLEY CONTROLLER',3:'MOVE BASE',4:'DWA',5:'FOLLOW THE GAP',6:'LOCAL STEERING CONTROLLER'}
speed_mode_dict = {0:'MANUAL',1:'CONSTANT',2:'GAUSSIAN', 3:'BINARY',4:'LINEAR',5:'RECORDED'}

print(f'MODE {driving_mode_dict[driving_mode]} with {speed_mode_dict[speed_mode]} SPEED')

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.stuck_x = 0
        self.stuck_y = 0
        self.yaw = yaw
        self.v = v
        self.driving_mode = 'DRIVE'
        self.time = time.time()

# initial state
state = State(x=0.0, y=0.0, yaw=0.0, v=0.0)

def get_velocity(steering):
    global breaking
    global state
        
    # print(state.driving_mode, np.sqrt((state.stuck_x - state.x)**2 + (state.stuck_y - state.y)**2))
    if breaking and time.time()-breaking_time < 0.5:
        velocity = -3
        
        
    elif breaking and time.time()-breaking_time > 0.5:
        breaking = False
        velocity = 0
        state.driving_mode = 'STUCK' 
        state.time = time.time()
        
    elif state.driving_mode == 'STUCK' and time.time()-state.time <1:
        
        velocity = -0.001
        
    elif state.driving_mode == 'STUCK' and time.time()-state.time >4:
        state.time = time.time()
        velocity = -0.001
        
        
        
    elif state.driving_mode == 'STUCK' and np.sqrt((state.stuck_x - state.x)**2 + (state.stuck_y - state.y)**2)<escape_distance:
        print('move back')
        velocity = -1
        
    
    elif state.driving_mode == 'STUCK' and np.sqrt((state.stuck_x - state.x)**2 + (state.stuck_y - state.y)**2)>escape_distance:
        state.driving_mode = 'DRIVE'
        print(state.driving_mode)
        velocity = 0
        
    elif aeb :
        velocity = -3
        
    elif collision :
        velocity = speed_max*np.exp(-steering**2/sigma2)
    
    elif speed_mode == 1:
        velocity = speed_max
        
    elif speed_mode == 2:
        velocity = speed_max*np.exp(-steering**2/sigma2)
        
    elif speed_mode == 3:
        if abs(steering)>binary_steering_threshold:
            velocity = speed_max*corner_speed_coef
            
        else:
            velocity = speed_max
            
    elif speed_mode == 4:
        velocity = speed_max - (1 - corner_speed_coef)* speed_max / steer_max * steering
    
    return velocity

def stuck_callback(data):
    
    global state
    
    if state.driving_mode != 'STUCK' and data.data:
        
        state.driving_mode = 'STUCK'
        state.time = time.time()
        state.stuck_x = state.x
        state.stuck_y = state.y
        print(state.driving_mode)
        print('wait')

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
        msg.angular.z = -data.AXIS_LEFT_STICK_X*steer_max
        pub.publish(msg)
        
    elif breaking and time.time()-breaking_time > 0.5:
        breaking = False
        
    elif aeb :
        msg.linear.x = -3
        msg.angular.z = -data.AXIS_LEFT_STICK_X*steer_max
        pub.publish(msg)
        
    elif driving_mode == 0:
        if data.BUTTON_SQUARE :
            msg.linear.x = -speed_max
        else :
            msg.linear.x = -data.AXIS_RIGHT_STICK_Y*speed_max
        msg.angular.z = -data.AXIS_LEFT_STICK_X*steer_max
        pub.publish(msg)
       
    elif driving_mode == 1 and speed_mode == 0 :
        
        
        msg.angular.z = PP_steering
        if data.BUTTON_SQUARE :
            msg.linear.x = -speed_max
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
        
        msg.angular.z = MB_steering
        msg.linear.x = -data.AXIS_RIGHT_STICK_Y*speed_max
        pub.publish(msg) 
        
    elif driving_mode == 4 and speed_mode == 0 :
        
        msg.angular.z = DWA_steering
        msg.linear.x = -data.AXIS_RIGHT_STICK_Y*speed_max
        pub.publish(msg) 
        
    elif driving_mode == 5 and speed_mode == 0 :
        
        msg.angular.z = FTG_steering
        msg.linear.x = -data.AXIS_RIGHT_STICK_Y*speed_max
        pub.publish(msg)
        
    elif driving_mode == 6 and speed_mode == 0 :
        
        msg.angular.z = LSC_steering
        msg.linear.x = -data.AXIS_RIGHT_STICK_Y*speed_max
        pub.publish(msg)
    
    
       
    
    
    if (data.RE_R1):
        speed_max = min(speed_max+0.25, 6)
        print(f'vitesse max :{speed_max} m/s')
        
    elif (data.RE_L1):
        speed_max = max(speed_max-0.25,0.5)
        print(f'vitesse max :{speed_max} m/s')
        
    if (data.HAT_X == 1 and data.RE_HAT_X):
        driving_mode = min(driving_mode+1, len(driving_mode_dict)-1)
        print(f'MODE {driving_mode_dict[driving_mode]} with {speed_mode_dict[speed_mode]} SPEED')
        
    elif (data.HAT_X == -1 and data.RE_HAT_X):
        driving_mode = max(driving_mode-1,0)
        print(f'MODE {driving_mode_dict[driving_mode]} with {speed_mode_dict[speed_mode]} SPEED')
        
    if (data.HAT_Y == 1 and data.RE_HAT_Y):
        speed_mode = min(speed_mode+1, len(speed_mode_dict)-1)
        print(f'MODE {driving_mode_dict[driving_mode]} with {speed_mode_dict[speed_mode]} SPEED')
        
    elif (data.HAT_Y == -1 and data.RE_HAT_Y):
        speed_mode = max(speed_mode-1,0)
        print(f'MODE {driving_mode_dict[driving_mode]} with {speed_mode_dict[speed_mode]} SPEED')
          
        

    
    
def pure_pursuit_callback(data):
    global PP_steering
    
    PP_speed = data.linear.x
    PP_steering = data.angular.z
    
    msg = Twist()

    if driving_mode == 1 :
        
        msg.angular.z = PP_steering
        
        if speed_mode<5 and speed_mode !=0:
            msg.linear.x = get_velocity(PP_steering)
        elif speed_mode == 5:
            msg.linear.x = PP_speed
        
        if speed_mode != 0:
            if state.driving_mode == 'STUCK':
                msg.angular.z = PP_steering
            pub.publish(msg)
    
def stanley_control_callback(data):
    global SC_steering
    
    SC_speed = data.linear.x
    SC_steering = data.angular.z
    
    msg = Twist()
    
    if driving_mode == 2 :
        
        msg.angular.z = SC_steering

        if speed_mode<5 and speed_mode !=0:
            msg.linear.x = get_velocity(SC_steering)
        elif speed_mode == 5:
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
	global MB_steering
	
	MB_steering = data.angular.z
	
	if driving_mode == 3 :
		if speed_mode != 0:
			pub.publish(data)
            
   
def dwa_callback(data):
    global DWA_steering

    DWA_speed = data.linear.x
    DWA_steering = data.angular.z
    
    msg = Twist()
    
    if driving_mode == 4 :
        
        msg.angular.z = DWA_steering

        if speed_mode<5 and speed_mode !=0:
            msg.linear.x = get_velocity(DWA_steering)
        elif speed_mode == 5:
            msg.linear.x = DWA_speed
        
        if speed_mode != 0:
            pub.publish(msg)   
            
def follow_the_gap_callback(data):
    global FTG_steering
    
    FTG_speed = data.linear.x
    FTG_steering = data.angular.z
    
    msg = Twist()
    
    if driving_mode == 5 :
        
        msg.angular.z = FTG_steering

        if speed_mode<5 and speed_mode !=0:
            msg.linear.x = get_velocity(FTG_steering)
        elif speed_mode == 5:
            msg.linear.x = FTG_speed
        
        if speed_mode != 0:
            pub.publish(msg)
            
            
def lsc_callback(data):
    global LSC_steering

    LSC_steering = data.angular.z
    
    msg = Twist()
    
    if driving_mode == 6 :
        
        msg.angular.z = LSC_steering

        if speed_mode<5 and speed_mode !=0:
            msg.linear.x = get_velocity(LSC_steering)
        elif speed_mode == 5:
            msg.linear.x = 0
            print('no speed return')
        
        if speed_mode != 0:
            pub.publish(msg) 

def odom_callback(data):
    
    global velocity_mes

    
    velocity_mes = -data.twist.twist.linear.x
    msg = Float32()
    msg.data = velocity_mes
    # pub_vel.publish(msg)
    
def server_callback(config, level):
    global corner_speed_coef
    global obstacle_speed_coef
    global binary_steering_threshold
    global sigma2 
            
    corner_speed_coef = config['corner_speed_coef']
    obstacle_speed_coef = config['obstacle_speed_coef']
    sigma2 = -steer_max**2/np.log(corner_speed_coef)
    binary_steering_threshold = config['binary_steering_threshold']
    
    
    return config

def odom_callback(data):
    
    global state
    
    state.x = data.pose.pose.position.x
    state.y = data.pose.pose.position.y
    
def collision_callback(data):
    
    global collision
    
    collision = data.data

def listener_and_pub():
    global pub_vel
    
    rospy.Subscriber("/DS4_input", DS4, callback)
    rospy.Subscriber("/pure_pursuit_cmd", Twist, pure_pursuit_callback)
    rospy.Subscriber("/stanley_control_cmd", Twist, stanley_control_callback)
    rospy.Subscriber("/move_base_cmd", Twist, move_base_callback)
    rospy.Subscriber("/dwa_cmd", Twist, dwa_callback)
    rospy.Subscriber("/follow_the_gap_cmd", Twist, follow_the_gap_callback)
    rospy.Subscriber("/local_steering_controller_cmd", Twist, lsc_callback)
    rospy.Subscriber("/AEB", Bool, aeb_callback)
    rospy.Subscriber("/camera/odom/sample", Odometry, odom_callback)
    rospy.Subscriber('stuck', Bool, stuck_callback, queue_size=10)
    rospy.Subscriber('camera/odom/sample', Odometry, odom_callback, queue_size=10)
    rospy.Subscriber('collision', Bool, collision_callback, queue_size=10)
    srv = Server(ControllerConfig, server_callback)
    pub_vel = rospy.Publisher('/vel',Float32,queue_size=5)
    rospy.spin()
	
if __name__ == '__main__':
	try:
		listener_and_pub()
		


	except  rospy.ROSInterruptException:
		
		pass
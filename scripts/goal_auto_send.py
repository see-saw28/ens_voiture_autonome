#!/usr/bin/env python3


import time
import sys
import rospy
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Header
import tf
from getkey import getkey, keys
import os
#os.environ['ROS_MASTER_URI']='http://172.20.10.9:11311'
#os.environ['ROS_IP']='172.20.10.8'



def callback(msg): 
    global i
    global position
    global temps
    global best_lap
    # x,y,z = pose.pose.pose.position
    x_r = msg.pose.pose.position.x
    y_r = msg.pose.pose.position.y
#    print(x_r,y_r)
    
    
    
    
    x_g,y_g,z_g=position[i%len(position)]
    
    rayon=1.0
#    print(abs(complex(x_r,y_r)-complex(x_g,y_g)))
    if (abs(complex(x_r,y_r)-complex(x_g,y_g))<rayon):
        
        
        if (i%len(position)==0 and i!=0):
            temps1=time.time()
            lap_time=temps1-temps
            temps=temps1
            if (lap_time<best_lap):
                best_lap=lap_time
                print('Meilleur tour :',lap_time)
            else:
                print('Temps du tour :', lap_time, 'PB :',best_lap)
        i+=1    
        print(i%len(position))
        psg = PoseStamped()
        psg.header.stamp = rospy.Time.now()
        psg.header.frame_id = "map"
        x_g,y_g,z_g=position[i%len(position)]
        psg.pose = Pose(Point(x_g, y_g, 0.),Quaternion(0,0,z_g,np.sqrt(1-z_g**2)))
        pub.publish(psg)
  


rospy.init_node('auto_checkpoints')
pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=2) 
rate = rospy.Rate(10)
i=0

#default map name
map_name="map1"

if (len(sys.argv)>1):
    map_name=sys.argv[1]
    
if (map_name=="map1"):
#map1
    position=[(-1.688,1.3676,0.700),(1.2898,2.7207,0),(1.667,0.9812,-1),(-0.7855,0.2721,-1)]
#position=[(-0.754,-0.858,0.241),(1.027,-1.808,-0.981),(-1.840,-2.90,0.992)]
elif (map_name=="big_lap"):
    position=[(-1.11,1.810,0.999),(-2.321,0.104,0.005),(2.36,0.805,0.803)]
#circuit
elif (map_name=="circuit"):
   position=[(-2.565,-0.539,0.013),(-1.126,1.128,-0.999),(-5.906,1.068,-0.627)]


temps = time.time()
best_lap = np.inf

if __name__ == '__main__':
    try:
        sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback) #We subscribe to the laser's topic
        print('1')
        psg = PoseStamped()
        psg.header.stamp = rospy.Time.now()
        psg.header.frame_id = "map"
        x_g,y_g,z_g=position[i%len(position)]
        psg.pose = Pose(Point(x_g, y_g, 0.),Quaternion(0,0,z_g,np.sqrt(1-z_g**2)))
        pub.publish(psg)
        rospy.spin()
        
    except rospy.ROSInterruptException:
        print('looser')
        pass

 

# msg.header = Header()
 
 
 

   

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jun 10 14:46:29 2022

@author: student
"""
from getkey import getkey, keys
key = getkey()

    
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat May 21 13:16:36 2022

@author: student
"""

import os
import time
import rospy
from ens_voiture_autonome.msg import DS4
from std_msgs.msg import String

os.environ["SDL_VIDEODRIVER"] = "dummy"


# Labels for DS4 controller axes
AXIS_LEFT_STICK_X = 0
AXIS_LEFT_STICK_Y = 1
AXIS_RIGHT_STICK_X = 3
AXIS_RIGHT_STICK_Y = 4
AXIS_R2 = 5
AXIS_L2 = 2

# Labels for DS4 controller buttons
# Note that there are 14 buttons (0 to 13 for pygame, 1 to 14 for Windows setup)
BUTTON_SQUARE = 3
BUTTON_CROSS = 0
BUTTON_CIRCLE = 1
BUTTON_TRIANGLE = 2

BUTTON_L1 = 4
BUTTON_R1 = 5
BUTTON_L2 = 6
BUTTON_R2 = 7

BUTTON_SHARE = 8
BUTTON_OPTIONS = 9

BUTTON_LEFT_STICK = 11
BUTTON_RIGHT_STICK = 12

BUTTON_PS = 10
BUTTON_PAD = 5

old_SQUARE = False
old_CROSS = False
old_CIRCLE = False
old_TRIANGLE = False

old_L1 = False
old_R1 = False
old_L2 = False
old_R2 = False

old_SHARE = False
old_OPTIONS = False

old_LEFT_STICK = False
old_RIGHT_STICK = False

old_PS = False
old_PAD = False

old_HAT_X = 0
old_HAT_Y = 0

# Labels for DS4 controller hats (Only one hat control)
HAT_1 = 0

quit = False


    
if __name__ == '__main__':
    try:
        
      
      

        # print("press 'q' to close")
        rospy.init_node('keyboard_teleop')
        pub = rospy.Publisher('keyboard_input', String, queue_size=10)
        rate = rospy.Rate(30) # 30hz
        
        
        
        #msg.pose = Pose(Point(x, y, 0.),Quaternion(*tf.transformations.quaternion_from_euler(roll, pitch, yaw)))
        
        while not (quit or rospy.is_shutdown()):
            key = getkey()
            print(key)
            # msg = DS4()
            # # Get events
            # for event in pygame.event.get():
        
            #     if event.type == pygame.JOYAXISMOTION:
            #         axis[event.axis] = round(event.value,3)
            #     elif event.type == pygame.JOYBUTTONDOWN:
            #         button[event.button] = True
            #     elif event.type == pygame.JOYBUTTONUP:
            #         button[event.button] = False
            #     elif event.type == pygame.JOYHATMOTION:
            #     	hat[event.hat] = event.value
                    
            # msg.AXIS_LEFT_STICK_X = axis[AXIS_LEFT_STICK_X]
            # msg.AXIS_LEFT_STICK_Y = axis[AXIS_LEFT_STICK_Y]
            # msg.AXIS_RIGHT_STICK_X = axis[AXIS_RIGHT_STICK_X]
            # msg.AXIS_RIGHT_STICK_Y = axis[AXIS_RIGHT_STICK_Y]
             
            # msg.AXIS_L2 = axis[AXIS_L2]
            # msg.AXIS_R2 = axis[AXIS_R2]
            # # Buttons
            # msg.BUTTON_SQUARE = button[BUTTON_SQUARE]
            # msg.BUTTON_CROSS = button[BUTTON_CROSS]
            # msg.BUTTON_CIRCLE = button[BUTTON_CIRCLE]
            # msg.BUTTON_TRIANGLE = button[BUTTON_TRIANGLE]
            # msg.BUTTON_L1 = button[BUTTON_L1]
            # msg.BUTTON_R1 = button[BUTTON_R1]
            # msg.BUTTON_L2 = button[BUTTON_L2]
            # msg.BUTTON_R2 = button[BUTTON_R2]
            # msg.BUTTON_SHARE = button[BUTTON_SHARE]
            # msg.BUTTON_OPTIONS = button[BUTTON_OPTIONS]
            # msg.BUTTON_LEFT_STICK = button[BUTTON_LEFT_STICK]
            # msg.BUTTON_RIGHT_STICK = button[BUTTON_RIGHT_STICK]
            # msg.BUTTON_PS = button[BUTTON_PS]
            # msg.BUTTON_PAD = button[BUTTON_PAD]
            # # Hats
            # msg.HAT_X = hat[HAT_1][0]
            # msg.HAT_Y =  hat[HAT_1][1]
            
            # msg.RE_SQUARE = (button[BUTTON_SQUARE] and not(old_SQUARE))
            # msg.RE_CROSS = (button[BUTTON_CROSS] and not(old_CROSS))
            # msg.RE_CIRCLE = (button[BUTTON_CIRCLE] and not(old_CIRCLE))
            # msg.RE_TRIANGLE = (button[BUTTON_TRIANGLE] and not(old_TRIANGLE))
            # msg.RE_L1 = (button[BUTTON_L1] and not(old_L1))
            # msg.RE_R1 = (button[BUTTON_R1] and not(old_R1))
            # msg.RE_L2 = (button[BUTTON_L2] and not(old_L2))
            # msg.RE_R2 = (button[BUTTON_R2] and not(old_R2))
            # msg.RE_SHARE = (button[BUTTON_SHARE] and not(old_SHARE))
            # msg.RE_OPTIONS = (button[BUTTON_OPTIONS] and not(old_OPTIONS))
            # msg.RE_LEFT_STICK = (button[BUTTON_LEFT_STICK] and not(old_LEFT_STICK))
            # msg.RE_RIGHT_STICK = (button[BUTTON_RIGHT_STICK] and not(old_RIGHT_STICK))
            # msg.RE_PS = (button[BUTTON_PS] and not(old_PS))
            # msg.RE_PAD = (button[BUTTON_PAD] and not(old_PAD))
            # msg.RE_HAT_X = (hat[HAT_1][0] != old_HAT_X)
            # msg.RE_HAT_Y = (hat[HAT_1][1] != old_HAT_Y)
            
            # old_SQUARE = button[BUTTON_SQUARE]
            # old_CROSS = button[BUTTON_CROSS]
            # old_CIRCLE = button[BUTTON_CIRCLE]
            # old_TRIANGLE = button[BUTTON_TRIANGLE]
            # old_L1 = button[BUTTON_L1]
            # old_R1 = button[BUTTON_R1]
            # old_L2 = button[BUTTON_L2]
            # old_R2 = button[BUTTON_R2]
            # old_SHARE = button[BUTTON_SHARE]
            # old_OPTIONS = button[BUTTON_OPTIONS]
            # old_LEFT_STICK = button[BUTTON_LEFT_STICK]
            # old_RIGHT_STICK = button[BUTTON_RIGHT_STICK]
            # old_PS = button[BUTTON_PS]
            # old_PAD = button[BUTTON_PAD]
            # old_HAT_X = hat[HAT_1][0]
            # old_HAT_Y = hat[HAT_1][1]
            
        
            # quit = button[BUTTON_PS]
        
            # Limited to 30 frames per second to make the display not so flashy
            # pub.publish(msg)
            rate.sleep()
    except rospy.ROSInterruptException or KeyboardInterrupt:
        
        pass

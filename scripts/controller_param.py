#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jun  1 17:12:42 2022

@author: student
"""
#!/usr/bin/env python3

import rospy

from dynamic_reconfigure.server import Server
from ens_voiture_autonome.cfg import ControllerConfig

def callback(config, level):
            
            rospy.loginfo("yo")
            
            return config

if __name__ == "__main__":
    rospy.init_node("controller_param", anonymous = False)

    srv = Server(ControllerConfig, callback)
    rospy.spin()

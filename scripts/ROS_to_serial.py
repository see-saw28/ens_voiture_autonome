#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri May 20 14:04:02 2022

@author: student
"""

#TODO: protocol ajouter KP et KI, terminal ASCII only

import threading
import time
import serial
import serial.tools.list_ports
import protocol
import pygame
import rospy
import os
from ens_voiture_autonome.msg import Payload
from geometry_msgs.msg import Twist

#os.environ['ROS_MASTER_URI']='http://192.168.1.174:11311'
#os.environ['ROS_IP']='192.168.1.174'

#boolean used to stop thread at the end
programEnded = False

#Serial port declaration
sp = serial.Serial()
sp.baudrate = 115200

sp.port='/dev/ttyACM0'
sp.open()
#Protocol object declaration
comm = protocol.CarProtocol()

#Default values
pwmProp = 1500
pwmDir = 1150
speed0 = 0
K0 = 1

#Limit values
propMax = 2000
propMin = 1000
dirMax = 1000
dirMin = 1300
speedMax = 8
speedMin = 0
Kmin = 0
Kmax = 1000

#Variable for Terminal
terminalBufferSize = 128


# pub = rospy.Publisher('path_marker', Marker, queue_size=5)

import struct

def calculateChecksum(speed_bytes):
    checksum = 255
    for i in range(4):
        
        checksum ^= speed_bytes[i]
    return checksum
        
def threadSpeedMeasure(serialPort, stringBuffer, pub):
    receiveState = 'START_FRAME'
    number_bytes = 1

    while not programEnded:
        comPortList = [p[0] for p in list(serial.tools.list_ports.comports())]
        if serialPort.is_open and serialPort.port in comPortList:
            # print(serialPort.in_waiting)
            c = serialPort.read_all()
        
            
            frame_received = False
            i=0
            while not frame_received:
                # print(i)
                if i+5<len(c):
                    start = c[len(c)-i-6]
                    speed_bytes = c[len(c)-i-5:len(c)-i-1]
                    speed = struct.unpack('f', speed_bytes)
                    cs = c[len(c)-i-1]
                   
                    
                    if start==255 and cs == calculateChecksum(speed_bytes) and speed_bytes[0]!= 255:
                        msg = Twist()
                    
                        msg.linear.x = speed[0]
                        pub.publish(msg)
                        frame_received = True
                        # print(start, speed, speed_bytes, cs, len(c))
                        
                    else :
                        i+=1
                        
                else :
                    print('dommage')
                    break
            
                
                
                
            
            '''stringBuffer += serialPort.read_all().decode("ASCII")
            # print(stringBuffer)
            string = stringBuffer.split('Vmes = ')
            if len(string)>1 :
                Vmes=int(string[1].split(' ')[0])/1000
                # print(Vmes)
                msg = Twist()
                msg.linear.x = Vmes*1.04
                pub.publish(msg)
            if len(stringBuffer) > terminalBufferSize:
                stringBuffer = stringBuffer[(len(stringBuffer) - terminalBufferSize):]'''
             
        time.sleep(0.01)
    

def threadSerialPort(serialPort, portCB):
    """
    Enables to connect and disconnec serial ports while
    the program is running.

    Parameters
    ----------
    serialPort : serialwin32.Serial
        Serial port object used to communicate.
    tkWindow : Tk
        Current window.

    Returns
    -------
    None.

    """
    #The program crashes when it tries to acces destroyed widgets,
    #save previous ports to limit their acces.
    # previousPorts = []
    
    #Infinite loop
    while not programEnded:
        #Check every ports connected, save them if they changed
        comPortList = [p[0] for p in list(serial.tools.list_ports.comports())]
        # if not comPortList == previousPorts:
        #     #Change detected, modify combobox
        #     if len(comPortList) == 0:
        #         portCB['values'] = ["No port"]
        #         portCB.set("No port")
        #     else:
        #         portCB['values'] = comPortList
        # previousPorts = comPortList.copy()
        
        #If nothing is connected, try to connect to ports until it's connected
        if (not serialPort.is_open):    
            for port in comPortList:
                try:
                    serialPort.port = port
                    serialPort.open()
                    print("Port " + port + ' opened')
                    portCB=port
                except serial.SerialException:
                    print("Unable to open port " + port)
        #Else, if the selected port is not the connected one, change it
        elif portCB not in [serialPort.port, "No port"]:
            port = portCB
            try:
                serialPort.close()
                serialPort.port = port
                serialPort.open()
                
            except serial.SerialException:
                print("Unable to open port " + port)
        #Else, if the connection has ended, close the port
        elif (serialPort.port not in comPortList):
            serialPort.close()
            
        #Wait to avoid spam
        time.sleep(0.1)
        


def callback(msg):
    
    protocol = msg.Protocol
    if protocol == "PWM":
        payload = (msg.pwmProp, msg.pwmDir)
        
    elif protocol == "ASSERVISSEMENT":
        payload = (msg.Vcons, msg.pwmDir)
        
    elif protocol == "PARAMETRES":
        payload = (msg.Kp, msg.Ki)
        
        
    comm.setProtocol(protocol)
    sp.write(comm.encodeMessage(payload))

    # if sp.is_open:
    #     pwmProp = propulsion.get()
    #     pwmDir = direction.get()
    #     payload = (pwmProp, pwmDir)
    #     sp.write(comm.encodeMessage(payload))
    

    # if sp.is_open:
    #     pwmDir = direction.get()
    #     pwmProp = propulsion.get()
    #     payload = (pwmProp, pwmDir)
    #     sp.write(comm.encodeMessage(payload))
        
        
    #     comm.setProtocol(protocol)
    


    
rospy.init_node('ROS_to_serial', anonymous=False)
pub = rospy.Publisher('/vel',Twist,queue_size=5)
rospy.Subscriber("/payload", Payload, callback)
# rospy.Subscriber("/cmd_vel", Twist, vel_callback)

#Timer init.
terminalBuffer = ""
terminalUpdateThread = threading.Thread(target=threadSpeedMeasure, args=(sp, terminalBuffer, pub, ))
terminalUpdateThread.start()


comportCB = "No port"
# #Thread Init.
# threadSPManager = threading.Thread(target=threadSerialPort, args=(sp, comportCB, ))
# threadSPManager.start()

# #Thread Init.
# threadDS4Manager = threading.Thread(target=threadDS4, args=(sp, comportCB, ))
# threadDS4Manager.start()


while not rospy.is_shutdown():
    
    rospy.Rate(10).sleep()
    
# try:
#     rospy.Subscriber("/payload", Payload, callback)
#     rospy.spin()
    
# except rospy.ROSInterruptException:
   
#     pass

#Closing program
if sp.isOpen():
    sp.close()          #Closing serial port
programEnded = True     #Closing thread
# threadSPManager.join()
terminalUpdateThread.join()

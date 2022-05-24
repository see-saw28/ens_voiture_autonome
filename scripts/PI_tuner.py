#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri May 20 14:02:34 2022

@author: student
"""
# -*- coding: utf-8 -*-
"""
Created on Sat Apr 16 10:38:26 2022

@author: Pierre-Alexandre
"""
#TODO: protocol ajouter KP et KI, terminal ASCII only
from tkinter import *
from tkinter import ttk
import threading
import time
import serial
import serial.tools.list_ports
import protocol
import pygame
import os
import rospy
from ens_voiture_autonome.msg import Payload, DS4
from geometry_msgs.msg import Twist

# os.environ['ROS_MASTER_URI']='http://192.168.1.174:11311'
# os.environ['ROS_IP']='192.168.1.174'


os.environ["SDL_VIDEODRIVER"] = "dummy"
pygame.init()
pygame.joystick.init()
try :
    controller = pygame.joystick.Joystick(0)
    controller.init()

    # Three types of controls: axis, button, and hat
    axis = {}
    button = {}
    hat = {}
    
    # Assign initial data values
    # Axes are initialized to 0.0
    for i in range(controller.get_numaxes()):
    	axis[i] = 0.0
    # Buttons are initialized to False
    for i in range(controller.get_numbuttons()):
    	button[i] = False
    # Hats are initialized to 0
    for i in range(controller.get_numhats()):
    	hat[i] = (0, 0)

except :
    print('no controller')
    
    
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

# Labels for DS4 controller hats (Only one hat control)
HAT_1 = 0

#boolean used to stop thread at the end
programEnded = False

#Serial port declaration
sp = serial.Serial()
sp.baudrate = 115200

#Protocol object declaration
comm = protocol.CarProtocol(protocol='PARAMETRES')
comm1 = protocol.CarProtocol(protocol='ASSERVISSEMENT')

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

asservissement = True
backward = False

leftHat_old = False
vit_old = 0


vit_max = 2
vitesse = 0



        
        
def callback_vel(msg):
    stringBuffer = f'Vmes = {msg.linear.x}, Vcons = {vitesse}'
    if not programEnded:
        terminal.delete(1.0, END)
        terminal.insert(END, stringBuffer)

#Functions
def threadDS4(comm1):
    global programEnded
    global asservissement
    global backward
    global leftHat_old
    global vit_old
   
    while not programEnded:
        # Get events
        for event in pygame.event.get():

            if event.type == pygame.JOYAXISMOTION:
                axis[event.axis] = round(event.value,3)
            elif event.type == pygame.JOYBUTTONDOWN:
                button[event.button] = True
            elif event.type == pygame.JOYBUTTONUP:
                button[event.button] = False
            elif event.type == pygame.JOYHATMOTION:
            	hat[event.hat] = event.value

        programEnded = (hat[HAT_1][1]==-1)
        
        leftHat = hat[HAT_1][0]==-1
        pwmDir_percent = 5.2+1.5*axis[AXIS_LEFT_STICK_X]
        pwmDir = int(pwmDir_percent*200)
        
        if (leftHat and not(leftHat_old)):
            asservissement = not(asservissement)
            if asservissement:
                comm1.setProtocol("ASSERVISSEMENT")
                print('Mode asservi')
            else:
                comm1.setProtocol("PWM")
                print('Mode direct')
                
        leftHat_old = leftHat
        
        
        R_JS = axis[AXIS_RIGHT_STICK_Y]
        vitesse = -vit_max*R_JS
        
        if asservissement:
            

            if (vitesse<0 and vit_old >=0):
                comm1.setProtocol("PWM")
                backward = True
                print('Mode direct, marche arrière')
                pwmProp = 1500
                
                for i in range(100):
                    send_payload((pwmProp, pwmDir))
                    
                time.sleep(0.2)
                send_payload((pwmProp, pwmDir))
                time.sleep(0.2)
                send_payload((pwmProp, pwmDir))
                time.sleep(0.2)
                
            elif (vit_old<0 and vitesse>0):
                    comm1.setProtocol("ASSERVISSEMENT")
                    backward = False
                    print('Mode asservissement, marche avant')
                
            elif backward:
                pwmProp_percent = 7.5-vitesse
                pwmProp = int(pwmProp_percent*200)
                
                comm1.setProtocol("PWM")
                send_payload((pwmProp,pwmDir))
                
            else :
                comm1.setProtocol("ASSERVISSEMENT")
                send_payload((vitesse, pwmDir))
                
            
        else :
            pwmProp_percent = 7.5-vitesse
            pwmProp = int(pwmProp_percent*200)
            
            comm1.setProtocol("PWM")
            send_payload((pwmProp,pwmDir))
            
        vit_old = vitesse
        
        # print(pwmProp, pwmDir)
        # Limited to 30 frames per second to make the display not so flashy
        clock = pygame.time.Clock()
        clock.tick(30)
        
def callback(msg):
    global programEnded
    global asservissement
    global backward
    global vit_old
    global vitesse
    
        
    leftHat = hat[HAT_1][0]==-1
    pwmDir_percent = 5.2+1.5*msg.AXIS_LEFT_STICK_X
    pwmDir = int(pwmDir_percent*200)
    
    if msg.RE_SQUARE:
        asservissement = not(asservissement)
        if asservissement:
            comm1.setProtocol("ASSERVISSEMENT")
            print('Mode asservi')
        else:
            comm1.setProtocol("PWM")
            print('Mode direct')
            
  
    
    
   
    vitesse = -vit_max*msg.AXIS_RIGHT_STICK_Y
    # print(msg.AXIS_RIGHT_STICK_Y)
    
    if asservissement:
        

        if (vitesse<0 and vit_old >=0):
            comm1.setProtocol("PWM")
            backward = True
            print('Mode direct, marche arrière')
            pwmProp = 1500
            
            send_payload((pwmProp, pwmDir))
            time.sleep(0.2)
            send_payload((pwmProp, pwmDir))
            time.sleep(0.2)
            send_payload((pwmProp, pwmDir))
            time.sleep(0.2)
            
        elif (vit_old<0 and vitesse>0):
                comm1.setProtocol("ASSERVISSEMENT")
                backward = False
                print('Mode asservissement, marche avant')
            
        elif backward:
            pwmProp_percent = 7.5-vitesse
            pwmProp = int(pwmProp_percent*200)
            
            comm1.setProtocol("PWM")
            send_payload((pwmProp,pwmDir))
            
        else :
            comm1.setProtocol("ASSERVISSEMENT")
            send_payload((vitesse, pwmDir))
            
        
    else :
        pwmProp_percent = 7.5-vitesse
        pwmProp = int(pwmProp_percent*200)
        
        comm1.setProtocol("PWM")
        send_payload((pwmProp,pwmDir))
        
    vit_old = vitesse


        
def send_payload(payload):
    msg = Payload()
    msg.Protocol = comm1.protocol
    if comm1.protocol == "PWM":
        
        msg.pwmProp = payload[0]
        msg.pwmDir = payload[1]
        
    elif comm1.protocol == "ASSERVISSEMENT":
        msg.Vcons = payload[0]
        msg.pwmDir = payload[1]
        
    elif comm1.protocol == "PARAMETRES":
        msg.Kp = payload[0]
        msg.Ki = payload[1]
        
    pub.publish(msg)

def slideProp(var_prop):
    """
    Function used when propulsion slider was moved.
    If a serial port is opened, send an order to change PWM.

    Parameters
    ----------
    var : TYPE
        DESCRIPTION.

    Returns
    -------
    None.

    """
  
    pwmProp = propulsion.get()
    pwmDir = direction.get()
    payload = (pwmProp, pwmDir)
    msg = Payload()
    msg.Protocol = comm.protocol
    # print(comm.protocol, payload)
    if comm.protocol == "PWM":
        
        msg.pwmProp = payload[0]
        msg.pwmDir = payload[1]
        
    elif comm.protocol == "ASSERVISSEMENT":
        msg.Vcons = payload[0]
        msg.pwmDir = payload[1]
        
    elif comm.protocol == "PARAMETRES":
        msg.Kp = payload[0]
        msg.Ki = payload[1]
        
    pub.publish(msg)
    
def slideDir(var_dir):
    """
    Function used when direction slider was moved.
    If a serial port is opened, send an order to change PWM.

    Parameters
    ----------
    var : TYPE
        DESCRIPTION.

    Returns
    -------
    None.

    """
  
    pwmProp = propulsion.get()
    pwmDir = direction.get()
    payload = (pwmProp, pwmDir)
    msg = Payload()
    msg.Protocol = comm.protocol
    # print(comm.protocol, payload)
    if comm.protocol == "PWM":
        
        msg.pwmProp = payload[0]
        msg.pwmDir = payload[1]
        
    elif comm.protocol == "ASSERVISSEMENT":
        msg.Vcons = payload[0]
        msg.pwmDir = payload[1]
        
    elif comm.protocol == "PARAMETRES":
        msg.Kp = payload[0]
        msg.Ki = payload[1]
        
    pub.publish(msg)    
        

def reset():
    """
    Function used when reset button was clicked.
    If a serial port is opened, send reset order.

    Returns
    -------
    None.

    """
    if comm.protocol == "PWM":
        pwmProp = 1500    
        pwmDir = 1150
    elif comm.protocol == "ASSERVISSEMENT":
        pwmProp = 0
        pwmDir = 1150
    elif comm.protocol == "PARAMETRES":
        pwmProp = 1
        pwmDir = 1
    
   
    payload = (pwmProp, pwmDir)
    send_payload(payload)
    propulsion.set(pwmProp)
    direction.set(pwmDir)
    
def onProtocolChange(evt):
    """
    

    Parameters
    ----------
    evt : Tkinter.event
        Tells many things.

    Returns
    -------
    None.

    """
    protocol = protocolCB.get()
    comm.setProtocol(protocol)
    if protocol == "PWM":
        propulsion.configure(to=propMax, from_=propMin, resolution=1, label='Propulsion')
        propulsion.set(pwmProp)
        direction.configure(to=dirMax, from_=dirMin, resolution=1, label='Direction')
        direction.set(pwmDir)
    elif protocol == "ASSERVISSEMENT":
        propulsion.configure(to=speedMax, from_=speedMin, resolution=0.001, label='Consigne de vitesse')
        propulsion.set(speed0)
        direction.configure(to=dirMax, from_=dirMin, resolution=1, label='Direction')
        direction.set(pwmDir)
    elif protocol == "PARAMETRES":
        propulsion.configure(to=Kmax, from_=Kmin, resolution=0.1, label="Kp")
        propulsion.set(K0)
        direction.configure(to=Kmax, from_=Kmin, resolution=0.1, label="Ki")
        direction.set(K0)
    propulsion.pack()

    

rospy.init_node('PWM_tester', anonymous=False)
pub = rospy.Publisher('/payload',Payload,queue_size=5)
rospy.Subscriber('DS4_input',DS4, callback)


#Window size
width = 750
height = 350

#Window Init.
root = Tk()
root.title("PWM tester - connexion pending")
root.geometry(str(width) + "x" + str(height))
root.resizable(width=False, height=False)

#Frames
lfCommand = LabelFrame(root, text="Commandes", padx=10, pady=10)
lfCommand.pack(side=BOTTOM, fill="both", expand="yes")

lfSettings = LabelFrame(root, text="Paramètres", padx=10, pady=10)
lfSettings.pack(anchor='nw', side=LEFT, fill="both", expand="yes")

lfTerminal = LabelFrame(root, text="Terminal", padx=10, pady=10)
lfTerminal.pack(anchor='ne', side=RIGHT, fill="both", expand="yes")

fSliders = Frame(lfCommand, relief=GROOVE, borderwidth=2)
fSliders.pack(padx = 10, pady=10)

fPort = Frame(lfSettings, relief=GROOVE, borderwidth=0)
fPort.pack(side=LEFT, anchor="nw", padx = 10, pady=0)

fProtocol = Frame(lfSettings, relief=GROOVE, borderwidth=0)
fProtocol.pack(side=RIGHT, anchor="ne", padx = 10, pady=0)



propulsion = Scale(fSliders, from_=Kmin, to=Kmax, orient=HORIZONTAL, resolution=0.1, length=width*0.95, label='Kp', command=slideProp)
propulsion.pack()
propulsion.set(K0)

#Direction slider
direction = Scale(fSliders, from_=Kmin, to=Kmax, orient=HORIZONTAL, resolution=0.1, length=width*0.95, label='Ki', command=slideDir)
direction.pack()
direction.set(K0)

#Reset button
resetButton = Button(lfCommand, command=reset, text='Reset')
resetButton.pack()

#COM PORT label
comportL = Label(fPort, text="Port", padx = 10, pady=10)
comportL.pack(side=TOP, anchor='nw')

#COM PORT combobox
comportCB = ttk.Combobox(fPort, state="readonly", values=["No port"])
comportCB.set("No port")
comportCB.pack(side=LEFT, anchor="nw")


#Protocol label
protocolL = Label(fProtocol, text="Protocole", padx = 10, pady=10)
protocolL.pack(anchor='nw')

#Protocol combobox
protocolCB = ttk.Combobox(fProtocol, state="readonly", values=["PWM", "ASSERVISSEMENT", "PARAMETRES"])
protocolCB.set("PARAMETRES")
protocolCB.bind("<<ComboboxSelected>>", onProtocolChange)
protocolCB.pack(side=LEFT, anchor="nw")

#Terminal label
terminal = Text(lfTerminal, padx=10, pady=10, state='normal')
terminal.pack()

rospy.Subscriber('cmd_vel',Twist, callback_vel)

# #Timer init.
# terminalBuffer = ""
# terminalUpdateThread = threading.Thread(target=threadUpdateTerminal, args=(sp, terminal, terminalBuffer, ))
# terminalUpdateThread.start()

# # #Thread Init.
# threadSPManager = threading.Thread(target=threadSerialPort, args=(sp, root, comportCB, ))
# threadSPManager.start()

# #Thread Init.
# threadDS4Manager = threading.Thread(target=threadDS4, args=(comm1, ))
# threadDS4Manager.start()



    


#infinite loop
root.mainloop()

#Closing program
if sp.isOpen():
    sp.close()          #Closing serial port
programEnded = True     #Closing thread
# threadSPManager.join()
# terminalUpdateThread.join()
# threadDS4Manager.join()


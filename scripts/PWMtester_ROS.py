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
from ens_voiture_autonome.msg import Payload

#os.environ['ROS_MASTER_URI']='http://192.168.1.174:11311'
#os.environ['ROS_IP']='192.168.1.174'


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
AXIS_RIGHT_STICK_X = 2
AXIS_RIGHT_STICK_Y = 5
AXIS_R2 = 3
AXIS_L2 = 4

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

#Functions
def threadUpdateTerminal(serialPort, tkTerminal, stringBuffer):
    while not programEnded:
        comPortList = [p[0] for p in list(serial.tools.list_ports.comports())]
        if serialPort.is_open and serialPort.port in comPortList:
            stringBuffer += serialPort.read_all().decode("ASCII")
            string = stringBuffer.split('Vmes = ')
            if len(string)>1 :
                Vmes=int(string[1].split(' ')[0])/1000
                print(Vmes)
            if len(stringBuffer) > terminalBufferSize:
                stringBuffer = stringBuffer[(len(stringBuffer) - terminalBufferSize):]
            if not programEnded:
                tkTerminal.delete(1.0, END)
                tkTerminal.insert(END, stringBuffer)
        time.sleep(0.1)
    

#Functions
def threadDS4():
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

        quit = (hat[HAT_1][1]==-1)

        # Print out results
        # os.system('cls')
        # # Axes
        # print("Left stick X:", axis[AXIS_LEFT_STICK_X])
        # print("Left stick Y:", axis[AXIS_LEFT_STICK_Y])
        # print("Right stick X:", axis[AXIS_RIGHT_STICK_X])
        # print("Right stick Y:", axis[AXIS_RIGHT_STICK_Y])
        # print("L2 strength:", axis[AXIS_L2])
        # print("R2 strength:", axis[AXIS_R2],"\n")
        # # Buttons
        # print("Square:", button[BUTTON_SQUARE])
        # print("Cross:", button[BUTTON_CROSS])
        # print("Circle:", button[BUTTON_CIRCLE])
        # print("Triangle:", button[BUTTON_TRIANGLE])
        # print("L1:", button[BUTTON_L1])
        # print("R1:", button[BUTTON_R1])
        # print("L2:", button[BUTTON_L2])
        # print("R2:", button[BUTTON_R2])
        # print("Share:", button[BUTTON_SHARE])
        # print("Options:", button[BUTTON_OPTIONS])
        # print("Left stick press:", button[BUTTON_LEFT_STICK])
        # print("Right stick press:", button[BUTTON_RIGHT_STICK])
        # print("PS:", button[BUTTON_PS])
        # print("Touch Pad:", button[BUTTON_PAD],"\n")
        # Hats
        # print("Hat X:", hat[HAT_1][0])
        # print("Hat Y:", hat[HAT_1][1],"\n")

        # print("Press PS button to quit:", quit)
        print(axis[AXIS_LEFT_STICK_X])
        
        vit=axis[AXIS_RIGHT_STICK_Y]
    #    if (vit<-0.5):
    #        pwm=-0.5
     #   else:
      #      pwm=vit
        pwm=7.5+1.5*vit
       
        print(pwm)
        # Limited to 30 frames per second to make the display not so flashy
        clock = pygame.time.Clock()
        clock.tick(30)

def threadSerialPort(serialPort, tkWindow, portCB):
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
    previousPorts = []
    
    #Infinite loop
    while not programEnded:
        #Check every ports connected, save them if they changed
        comPortList = [p[0] for p in list(serial.tools.list_ports.comports())]
        if not comPortList == previousPorts:
            #Change detected, modify combobox
            if len(comPortList) == 0:
                portCB['values'] = ["No port"]
                portCB.set("No port")
            else:
                portCB['values'] = comPortList
        previousPorts = comPortList.copy()
        
        #If nothing is connected, try to connect to ports until it's connected
        if (not serialPort.is_open):    
            for port in comPortList:
                try:
                    serialPort.port = port
                    serialPort.open()
                    if not programEnded:
                        tkWindow.title("PWM tester - connected [{}]".format(port))
                    portCB.set(port)
                except serial.SerialException:
                    print("Unable to open port " + port)
        #Else, if the selected port is not the connected one, change it
        elif portCB.get() not in [serialPort.port, "No port"]:
            port = portCB.get()
            try:
                serialPort.close()
                serialPort.port = port
                serialPort.open()
                if not programEnded:
                    tkWindow.title("PWM tester - connected [{}]".format(port))
            except serial.SerialException:
                print("Unable to open port " + port)
        #Else, if the connection has ended, close the port
        elif (serialPort.port not in comPortList):
            serialPort.close()
            if not programEnded:
                tkWindow.title("PWM tester - connexion pending")
        #Wait to avoid spam
        time.sleep(0.1)
        
def send_payload(payload):
    msg = Payload()
    msg.Protocol = comm.protocol
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
        direction.configure(to=Kmax, from_=Kmin, resolution=0.1, label="Kp")
        direction.set(K0)
    propulsion.pack()

    

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

#Propulsion slider 
propulsion = Scale(fSliders, from_=propMin, to=propMax, orient=HORIZONTAL, resolution=1, length=width*0.95, label='Propulsion', command=slideProp)
propulsion.pack()
propulsion.set(pwmProp)

#Direction slider
direction = Scale(fSliders, from_=dirMin, to=dirMax, orient=HORIZONTAL, resolution=1, length=width*0.95, label='Direction', command=slideDir)
direction.pack()
direction.set(pwmDir)

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
protocolCB.set("PWM")
protocolCB.bind("<<ComboboxSelected>>", onProtocolChange)
protocolCB.pack(side=LEFT, anchor="nw")

#Terminal label
terminal = Text(lfTerminal, padx=10, pady=10, state='normal')
terminal.pack()

# #Timer init.
# terminalBuffer = ""
# terminalUpdateThread = threading.Thread(target=threadUpdateTerminal, args=(sp, terminal, terminalBuffer, ))
# terminalUpdateThread.start()

# #Thread Init.
# threadSPManager = threading.Thread(target=threadSerialPort, args=(sp, root, comportCB, ))
# threadSPManager.start()

# #Thread Init.
# threadDS4Manager = threading.Thread(target=threadDS4)
# threadDS4Manager.start()

try:
    rospy.init_node('PWM_tester', anonymous=False)
    pub = rospy.Publisher('/payload',Payload,queue_size=5)
    
except rospy.ROSInterruptException:
    pass

#infinite loop
root.mainloop()

#Closing program
if sp.isOpen():
    sp.close()          #Closing serial port
programEnded = True     #Closing thread
# threadSPManager.join()
# terminalUpdateThread.join()

# Use pygame to get readouts from a ds4 controller
# This is an efficient way to get inputs as long as you don't need six-axis data

import pygame
import os
from getkey import getkey, keys
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
os.environ["SDL_VIDEODRIVER"] = "dummy"
pygame.init()
pygame.joystick.init()

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


# Main loop, one can press the PS button to break
quit = False
while quit == False:

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

    quit = hat[HAT_1][1]==-1

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
    p.ChangeDutyCycle(5.2+1.5*axis[AXIS_LEFT_STICK_X])
    vit=axis[AXIS_RIGHT_STICK_Y]
#    if (vit<-0.5):
#        pwm=-0.5
 #   else:
  #      pwm=vit
    pwm=7.5+1.5*vit
    q.ChangeDutyCycle(pwm)
    print(pwm)
    # Limited to 30 frames per second to make the display not so flashy
    clock = pygame.time.Clock()
    clock.tick(30) 
    
q.stop()
p.stop()
GPIO.cleanup()


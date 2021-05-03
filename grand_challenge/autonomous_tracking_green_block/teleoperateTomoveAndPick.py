import cv2
import os
import numpy as np
import imutils
import RPi.GPIO as GPIO
import time

#globals
trig = 16
echo = 18


def init():
    # setup the GPIO Pins
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(31, GPIO.OUT)  #IN1
    GPIO.setup(33, GPIO.OUT)  #IN2
    GPIO.setup(35, GPIO.OUT)  #IN3
    GPIO.setup(37, GPIO.OUT)  #IN4
    GPIO.setup(36,GPIO.OUT)
    GPIO.setup(trig, GPIO.OUT)
    GPIO.setup(echo, GPIO.IN)
    

def gameover():
    # Set all pins low
    GPIO.output(31, False)
    GPIO.output(33, False)
    GPIO.output(35, False)
    GPIO.output(37, False)


def forward(tf):
    init()
    # Left wheels
    GPIO.output(31, True) 
    GPIO.output(33, False) 
    # Right wheels
    GPIO.output(35, False) 
    GPIO.output(37, True) 
    # Wait
    time.sleep(tf)
    # Set all pins low and cleanup
    gameover()
    GPIO.cleanup()


def reverse(tf):
    init()
    # Left wheels
    GPIO.output(31, False) 
    GPIO.output(33, True) 
    # Right wheels
    GPIO.output(35, True) 
    GPIO.output(37, False) 
    # Wait
    time.sleep(tf)
    # Set all pins low and cleanup
    gameover()
    GPIO.cleanup()


def pivotright(tf):
    init()
    # Left wheels
    GPIO.output(31, True)
    GPIO.output(33, False)
    # Right wheels
    GPIO.output(35, True)
    GPIO.output(37, False)
    # Wait
    time.sleep(tf)
    # Set all pins low and cleanup
    gameover()
    GPIO.cleanup()


def pivotleft(tf):
    init()
    # Left wheels
    GPIO.output(31, False)
    GPIO.output(33, True)
    # Right wheels
    GPIO.output(35, False)
    GPIO.output(37, True)
    # Wait
    time.sleep(tf)
    # Set all pins low and cleanup
    gameover()
    GPIO.cleanup()
    
# def closeGripper():
#     GPIO.setmode(GPIO.BOARD)
#     GPIO.setup(36, GPIO.OUT)  #Gripper
    
#     pwm = GPIO.PWM(36, 50)
#     pwm.start(5)
    
#     rate = 0.15
#     duty = float(3.5)
#     while True:
#         duty += rate
#         pwm.ChangeDutyCycle(duty)
#         time.sleep(0.2)
#         if duty >= 6.5:#6.25:
#             break
#     pwm.stop()
#     #clear the output pins
#     #GPIO.cleanup() 

# def openGripper():
#     GPIO.setmode(GPIO.BOARD)
#     GPIO.setup(36, GPIO.OUT)  #Gripper
    
#     pwm = GPIO.PWM(36, 50)
#     pwm.start(5)
    
#     rate = 0.15
#     duty = float(6)
#     while True:
#         pwm.ChangeDutyCycle(duty)
#         duty -= rate        
#         time.sleep(0.2)
#         if duty <= 3.5:
#             break
#     pwm.stop()
#     #clear the output pins
#     #GPIO.cleanup() 


init()
pwm_servo = GPIO.PWM(36,50)
pwm_servo.start(7.7)
time.sleep(0.5)



def open_servo():
    pwm_servo.ChangeDutyCycle(7.7) 
    time.sleep(0.3)

def close_servo():
    pwm_servo.ChangeDutyCycle(2.5) 
    time.sleep(0.3)



            
def distance():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(trig, GPIO.OUT)
    GPIO.setup(echo, GPIO.IN)
    
    #Ensure outout has no value
    GPIO.output(trig, False)
    time.sleep(0.01)

    #Generate Trigger pulse
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    #Generate Echo time signal
    while GPIO.input(echo) == 0:
        pulse_start = time.time()

    while GPIO.input(echo) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start

    #Convert time to distance
    distance = pulse_duration*17150
    distance = round(distance, 2)
        
    #clear the output pins
    GPIO.cleanup()        
    return distance

def key_input(event):
    init()
    print("Key: ", event)
    key_press = event
    tf = 1
    
    if key_press.lower() == 'w':
        forward(tf)
    elif key_press.lower() == 's':
        reverse(tf)
    elif key_press.lower() == 'a':
        pivotleft(tf)
    elif key_press.lower() == 'd':
        pivotright(tf)
    elif key_press.lower() == 'c':
        # closeGripper()
        close_servo()
    elif key_press.lower() == 'o':
        # openGripper()
        open_servo()
    else:
        print("Invalid key pressed!")
    
    #distance_sum = 0;
    #for i in range(0,10):
    #    distance_sum = distance_sum + distance()
    
    #avg_distance = distance_sum / 10
    #print("Distance for closest object in the front: "+str(avg_distance)+"cm")

if __name__ == '__main__':

    while True:
        key_press = input("Select driving mode: ")
        if key_press == 'q':
            
            break
        key_input(key_press)

    GPIO.cleanup()

    
    


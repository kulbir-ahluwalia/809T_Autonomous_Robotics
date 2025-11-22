import cv2
import imutils
import numpy as np
import matplotlib.pyplot as plt
import time
import RPi.GPIO as gpio
import serial
import datetime
from picamera.array import PiRGBArray
from picamera import PiCamera

import math

import os
import smtplib
from smtplib import SMTP
from smtplib import SMTPException
import email
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage

def init():
    gpio.cleanup()
    gpio.setmode(gpio.BOARD)
    gpio.setup(31,gpio.OUT) #IN1
    gpio.setup(33,gpio.OUT) #IN2
    gpio.setup(35,gpio.OUT) #IN3
    gpio.setup(37,gpio.OUT) #IN4
    gpio.setup(36,gpio.OUT)
    #right back wheel encoder
    gpio.setup(12,gpio.IN,pull_up_down = gpio.PUD_UP)
    #left front wheel encoder
    gpio.setup(7,gpio.IN,pull_up_down = gpio.PUD_UP)

def gameover():
    gpio.setmode(gpio.BOARD)
    gpio.setup(31,gpio.OUT) #IN1
    gpio.setup(33,gpio.OUT) #IN2
    gpio.setup(35,gpio.OUT) #IN3
    gpio.setup(37,gpio.OUT) #IN4
    gpio.setup(36,gpio.OUT)
    gpio.output(31,False)
    gpio.output(33,False)
    gpio.output(35,False)
    gpio.output(37,False)
    gpio.cleanup()

def motors_off():
    gpio.setmode(gpio.BOARD)
    gpio.setup(31,gpio.OUT) #IN1
    gpio.setup(33,gpio.OUT) #IN2
    gpio.setup(35,gpio.OUT) #IN3
    gpio.setup(37,gpio.OUT) #IN4
    gpio.setup(36,gpio.OUT)
    gpio.output(31,False)
    gpio.output(33,False)
    gpio.output(35,False)
    gpio.output(37,False)
    # gpio.cleanup()
#MAIN CODE

#Indentify serial communication
ser = serial.Serial('/dev/ttyUSB0', 9600)

global path_list 
path_list = []

init()
pwm_servo = gpio.PWM(36,50)
pwm_servo.start(7.7)
# time.sleep(0.5)

def open_servo():
    pwm_servo.ChangeDutyCycle(7.7) 
    time.sleep(0.3)

def close_servo():
    pwm_servo.ChangeDutyCycle(2.5) 
    time.sleep(0.3)


def sendEmail(picName):
    # Email information
    smtpUser = 'enpm809t.kulbir@gmail.com'
    smtpPass = '809Tenpm.123'

    # Destination email information
    
    # toAdd = ['kulbir97ahluwalia@gmail.com','kulbir.ahluwalia97@gmail.com','ENPM809TS19@gmail.com', 'skotasai@umd.edu']
    toAdd = ['kulbir97ahluwalia@gmail.com','kulbir.ahluwalia97@gmail.com']

    
    fromAdd = smtpUser
    subject = 'ENPM809T_picture_green_blocked_retrieved_Kulbir '
    msg = MIMEMultipart()
    msg['Subject'] = subject
    msg['From'] = fromAdd
    #msg['To'] = toAdd
    msg['To'] = ",".join(toAdd)
    msg.preamble = "Image of green block"
    
    #Email Text
    body = MIMEText("Name: Kulbir Singh Ahluwalia, Image: green block retrieved!! ")
    msg.attach(body)

    #Attach image
    fp = open(picName+'.jpg','rb')
    img = MIMEImage(fp.read())
    fp.close
    msg.attach(img)
    
    #send email
    s = smtplib.SMTP('smtp.gmail.com',587)

    s.ehlo()
    s.starttls()
    s.ehlo()
    s.login(smtpUser, smtpPass)
    s.sendmail(fromAdd, toAdd, msg.as_string())
    s.quit()
    
    print('Email Delivered!')



def reverse(distance):
        
    init()

    # print("How much distance do you want to go backward?")
    # dist = float(input())
    dist = distance
    # number_of_ticks = int(97.94*dist)
    number_of_ticks = int(98*dist)
    # print("number of ticks required: ",number_of_ticks)


    # dist = float(0)
    # while True:
    #     motors_off()
    #     print("How much distance do you want to go forward?")
    #     dist = float(input())
    #     if dist != 0:
    #         break


    Back_Right_counter = np.uint64(0)
    Front_Left_counter = np.uint64(0) 
    buttonBR = int(0)
    buttonFL = int(0)

    #initialize pwm signal to control motor

    pwm_BR = gpio.PWM(35,50)
    pwm_FL = gpio.PWM(33,50)

    val = 40

    pwm_BR.start(val)
    pwm_FL.start(val)

    time.sleep(0.1)

    # list_of_gpioBR = []
    # list_of_gpioFL = []






    # for i in range(0,2000000000000):
    while True:
        # time.sleep(0.05)
        # print("Back_Right_counter = ",Back_Right_counter,"GPIO state at Pin 12: ",gpio.input(12))
        # print("Front_Left_counter = ",Front_Left_counter,"GPIO state at Pin 7: ",gpio.input(7))
        # list_of_gpioBR.append(gpio.input(12))
        # list_of_gpioFL.append(gpio.input(7))
        if int (gpio.input(12)) != int(buttonBR):
            buttonBR = int(gpio.input(12))
            # buttonFL = int(gpio.input(7))
            Back_Right_counter+= 1
            # Front_Left_counter+=1

        if int (gpio.input(7)) != int(buttonFL):
            buttonFL = int(gpio.input(7))
            Front_Left_counter+=1

        if Back_Right_counter>=number_of_ticks:
            pwm_BR.stop()

        if Front_Left_counter>=number_of_ticks:
            pwm_FL.stop()

        if Back_Right_counter>=number_of_ticks and Front_Left_counter>=number_of_ticks:
            #  pwm_BR.stop()
            gameover()
            # print("THANKS")
            #  gpio.cleanup()
            break




def forward(distance):
    init()
    # print("How much distance do you want to go forward?")
    # dist = float(input())
    dist = distance
    # number_of_ticks = int(97.94*dist)
    number_of_ticks = int(98*dist)
    # print("number of ticks required: ",number_of_ticks)


    # dist = float(0)
    # while True:
    #     motors_off()
    #     print("How much distance do you want to go forward?")
    #     dist = float(input())
    #     if dist != 0:
    #         break


    Back_Right_counter = np.uint64(0)
    Front_Left_counter = np.uint64(0) 
    buttonBR = int(0)
    buttonFL = int(0)

    #initialize pwm signal to control motor

    pwm_BR = gpio.PWM(37,50)
    pwm_FL = gpio.PWM(31,50)

    val = 30

    pwm_BR.start(val)
    pwm_FL.start(val)

    time.sleep(0.1)

    # list_of_gpioBR = []
    # list_of_gpioFL = []






    # for i in range(0,2000000000000):
    while True:
        # time.sleep(0.05)
        # print("Back_Right_counter = ",Back_Right_counter,"GPIO state at Pin 12: ",gpio.input(12))
        # print("Front_Left_counter = ",Front_Left_counter,"GPIO state at Pin 7: ",gpio.input(7))
        # list_of_gpioBR.append(gpio.input(12))
        # list_of_gpioFL.append(gpio.input(7))
        if int (gpio.input(12)) != int(buttonBR):
            buttonBR = int(gpio.input(12))
            # buttonFL = int(gpio.input(7))
            Back_Right_counter+= 1
            # Front_Left_counter+=1

        if int (gpio.input(7)) != int(buttonFL):
            buttonFL = int(gpio.input(7))
            Front_Left_counter+=1

        if Back_Right_counter>=number_of_ticks:
            pwm_BR.stop()

        if Front_Left_counter>=number_of_ticks:
            pwm_FL.stop()

        if Back_Right_counter>=number_of_ticks and Front_Left_counter>=number_of_ticks:
            #  pwm_BR.stop()
            # gameover()
            motors_off()
            # print("THANKS")
            #  gpio.cleanup()
            break

    # file = open('BR_gpio_states.txt','w')
    # for i in list_of_gpioBR:
    #     file.write(str(i))
    #     file.write('\n')
    # file.close()


    # file = open('FL_gpio_states.txt','w')
    # for i in list_of_gpioFL:
    #     file.write(str(i))
    #     file.write('\n')
    # file.close()



def left(left_angle):
    init()

    # print("How much angle in degrees do you want to go left?")
    # dist = float(input())
    # angle = float(input()) 
    angle = left_angle 
    current_angle_threshold_range = 1 #degrees
    
    # angular_distance = 0.0012654*angle*1.42
    # number_of_ticks = int(98*angular_distance)

    Back_Right_counter = np.uint64(0)
    Front_Left_counter = np.uint64(0) 
    buttonBR = int(0)
    buttonFL = int(0)

    # time.sleep(0.1)

    global count
    count =0

    global target_angle
    target_angle = 0

    while True:
        if(ser.in_waiting > 0):
            count+=1
            
            #read serial stream
            line = ser.readline()
            # print(line)
            
            #avoid first n lines of serial info
            if(count>15):
                
                #strip serial stream of extra characters
                line = line.rstrip().lstrip()
                # print(line)

                line = str(line)
                line = line.strip("'")
                line = line.strip("b'")
                # print(line)


                #return float
                start_angle = float(line)
                print("start_angle is: ", start_angle)
                # current_angle = line
                # target_angle = (float(line) - angle)%360
                target_angle = (float(line) - angle)

                # if target_angle>270:
                #     target_angle = target_angle-360

                print("target_angle is: ", target_angle)
                break

                # print(line,"\n")

    pwm_BR = gpio.PWM(37,50)

    pwm_FL = gpio.PWM(33,50)


    while True:
        if(ser.in_waiting > 0):
            count+=1
            
            #read serial stream
            line = ser.readline()
            # print(line)
            
            #avoid first n lines of serial info
            if(count>15):
                
                #strip serial stream of extra characters
                line = line.rstrip().lstrip()
                # print(line)


                

                line = str(line)
                line = line.strip("'")
                line = line.strip("b'")
                # print(line)


                #return float
                line = float(line)
                current_angle = float(line)
                print("current_angle is: ", current_angle)

                val = 50

                pwm_BR.start(val)
                pwm_FL.start(val)

                angles_info_list = [current_angle,target_angle]
                path_list.append(angles_info_list)
        
                if int (gpio.input(12)) != int(buttonBR):
                    buttonBR = int(gpio.input(12))
                    # buttonFL = int(gpio.input(7))
                    Back_Right_counter+= 1
                    # Front_Left_counter+=1

                if int (gpio.input(7)) != int(buttonFL):
                    buttonFL = int(gpio.input(7))
                    Front_Left_counter+=1

                if (target_angle>=current_angle-current_angle_threshold_range) and (target_angle<=current_angle+current_angle_threshold_range):
                    pwm_BR.stop()
                    pwm_FL.stop()
                    gameover()
                    break

    



def right(right_angle):
    init()

    # print("How much angle in degrees do you want to go left?")
    # dist = float(input())
    # angle = float(input()) 
    angle = right_angle 
    current_angle_threshold_range = 2 #degrees
    
    # angular_distance = 0.0012654*angle*1.42
    # number_of_ticks = int(98*angular_distance)

    Back_Right_counter = np.uint64(0)
    Front_Left_counter = np.uint64(0) 
    buttonBR = int(0)
    buttonFL = int(0)

    # time.sleep(0.1)

    global count
    count =0

    global target_angle
    target_angle = 0

    while True:
        if(ser.in_waiting > 0):
            count+=1
            
            #read serial stream
            line = ser.readline()
            # print(line)
            
            #avoid first n lines of serial info
            if(count>15):
                
                #strip serial stream of extra characters
                line = line.rstrip().lstrip()
                # print(line)

                line = str(line)
                line = line.strip("'")
                line = line.strip("b'")
                # print(line)


                #return float
                start_angle = float(line)
                print("start_angle is: ", start_angle)
                # current_angle = line
                # target_angle = (float(line) + angle)%360
                target_angle = (float(line) + angle)%360


                print("target_angle is: ", target_angle)

                # if target_angle>270:
                #     target_angle = target_angle-360

                break

                # print(line,"\n")

    pwm_BR = gpio.PWM(35,50)
    pwm_FL = gpio.PWM(31,50)


    while True:
        if(ser.in_waiting > 0):
            count+=1
            
            #read serial stream
            line = ser.readline()
            # print(line)
            
            #avoid first n lines of serial info
            if(count>15):
                
                #strip serial stream of extra characters
                line = line.rstrip().lstrip()
                # print(line)

                line = str(line)
                line = line.strip("'")
                line = line.strip("b'")
                # print(line)


                #return float
                line = float(line)
                current_angle = float(line)
                print("current_angle is: ", current_angle)

                val = 50

                pwm_BR.start(val)
                pwm_FL.start(val)

                angles_info_list = [current_angle,target_angle]
                path_list.append(angles_info_list)
        
                if int (gpio.input(12)) != int(buttonBR):
                    buttonBR = int(gpio.input(12))
                    # buttonFL = int(gpio.input(7))
                    Back_Right_counter+= 1
                    # Front_Left_counter+=1

                if int (gpio.input(7)) != int(buttonFL):
                    buttonFL = int(gpio.input(7))
                    Front_Left_counter+=1

                if (target_angle>=current_angle-current_angle_threshold_range) and (target_angle<=current_angle+current_angle_threshold_range):
                    pwm_BR.stop()
                    pwm_FL.stop()
                    gameover()
                    break

    





# image = cv2.imread("all_blocks_arena.jpg")

# image = imutils.resize(image, width = 640)
# # cv2.imshow("base image", image)

# hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
# # cv2.imshow("hsv image", hsv)

# # green_light_lower_range = np.array([75,100,100])
# # green_light_upper_range = np.array([87,255,255])

# # #room parameters
# # green_light_lower_range = np.array([47,103,29])
# # green_light_upper_range = np.array([99,255,174])

# #arena parameters
# green_light_lower_range = np.array([41,105,20])
# green_light_upper_range = np.array([83,255,224])

# mask = cv2.inRange(hsv, green_light_lower_range, green_light_upper_range)

# # mask = cv2.inRange(hsv, (155,95,85), (165,105,255))
# # cv2.imshow("mask", mask)


# image_edges = cv2.Canny(mask,30,200)
# # cv2.imshow("edges using canny edge detector", image_edges)

# # contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
# contours = cv2.findContours(image_edges.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
# contours = imutils.grab_contours(contours)

# # print("contours = " + str(len(contours)))

# # see the contours array
# print("contours = ", contours)

# cv2.drawContours(mask, contours, -1, (255,0,0), 3)
# # cv2.imshow("with contours on mask", mask)

# contour_points = contours[0]
# (x,y),radius = cv2.minEnclosingCircle(contour_points)

# circle_center = (int(x),int(y))

# radius = int(radius)
# cv2.circle(image,circle_center,radius,(238,130,238),3)
# cv2.circle(image,circle_center,2,(0,0,0),3)

# cv2.drawContours(image, [contour_points], 0, (238,0,0), 3)
# # cv2.imshow("final_image", image)




# # press the 'q' key to close all images
# key = cv2.waitKey(0) & 0xFF
# print(key)

# if key == ord("q"):
#     cv2.destroyAllWindows()

def nothing(x):
    pass

# image_center = (320,240)

#using picamera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 25
rawCapture = PiRGBArray(camera, size=(640,480))
time.sleep(0.1)


# create the object using video writer using xvid codec
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('trackblockandretrive.avi', fourcc, 10, (640, 480))


#using video capture, ran into hanging issues
# cam= cv2.VideoCapture(0)
# fps = cam.get(cv2.CAP_PROP_FPS)
# timestamps = [cam.get(cv2.CAP_PROP_POS_MSEC)]
# calc_timestamps = [0.0]

#text parameters
font = cv2.FONT_HERSHEY_SIMPLEX 
fontScale = 0.5
font_color = (0, 0, 255)
font_thickness = 1
direction_scale = 1
direction_thickness = 2

#videowrite
# fourcc = cv2.VideoWriter_fourcc(*'XVID')
# out = cv2.VideoWriter('green_light_detection_video.avi', fourcc, 5.0, (640,480))
# timeElapsedlist = []

open_servo()


for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=False):
    image = frame.array
    image = cv2.rotate(image, cv2.ROTATE_180)


    startTime = time.time()
    
   
    hsv_img = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

    blurred = cv2.GaussianBlur(hsv_img, (5, 5), 0)
  
    lower_range = np.array([41,105,20])
    upper_range = np.array([83,255,224])

    # use threshold
    mask = cv2.inRange(blurred, lower_range, upper_range)
    # image_edges = cv2.Canny(mask,85,255)

    #using contours
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    if len(cnts) == 0:
        print("No contours found. No object detected!")
      

    #find centre of the contour
    biggest_contour = max(cnts, key=cv2.contourArea)
   
    (object_X,object_Y),radius = cv2.minEnclosingCircle(biggest_contour)
    center = (int(object_X),int(object_Y))
    radius = int(radius)
    print("center is: ",center,"radius is: ",radius)

    
    # cv2.circle(image, (object_X, object_Y), 15, (0,0,255), 8)  
    cv2.circle(image, (center[0], center[1]), radius, (0,0,255), 3) 
    cv2.circle(image,(center[0], center[1]),2,(0,0,0),3)
    
    global block_pixel_location 
    block_pixel_location = center
    
    image_center_X = 320
    image_center_Y = 240

    #draw crosshair centered at 320,240
    cv2.line(image,(320,120),(320,360),(0,0,0),2)
    cv2.line(image,(200,240),(440,240),(0,0,0),2)
    location_text = "Block_locn: " + str(block_pixel_location) 
    cv2.putText(image,location_text,(20,35),cv2.FONT_HERSHEY_DUPLEX,direction_scale,(0,0,255), direction_thickness)

    radius_text = "Contour_Radius: " + str(radius) 
    cv2.putText(image,radius_text,(20,75),cv2.FONT_HERSHEY_DUPLEX,direction_scale,(0,0,255), direction_thickness)
    cv2.imshow("cam",image)

    # time.sleep(2)



    global pixel_to_rotate 
    pixel_to_rotate = image_center_X - object_X
    print("pixel_to_rotate",pixel_to_rotate)


    global degree_to_rotate
    degree_to_rotate = int(0.061*pixel_to_rotate)
    print("degree_to_rotate",degree_to_rotate)


    global distance_of_block_in_pixels
    distance_of_block_in_pixels = image_center_Y - object_Y
    print("distance_of_block_in_pixels: ",distance_of_block_in_pixels,"\n")

    # set range of pixels for the block
    global object_center_pixel_threshold
    object_center_pixel_threshold = 50

    global angle_to_rotate 
    angle_to_rotate = degree_to_rotate









    if (radius<190):
        # if(pixel_to_rotate<-object_center_pixel_threshold):
        #     right(angle_to_rotate)
        #     time.sleep(0.1)
            
        # elif(pixel_to_rotate>object_center_pixel_threshold):
        #     left(angle_to_rotate)
        #     time.sleep(0.1)
            
        # elif(-object_center_pixel_threshold<=pixel_to_rotate<=object_center_pixel_threshold):
        #     forward(0.05)
        #     open_servo()
        #     time.sleep(0.1)

        if(object_X<(image_center_X - object_center_pixel_threshold)):
            left(abs(angle_to_rotate))
            time.sleep(0.1)
            
        elif(object_X>(image_center_X + object_center_pixel_threshold)):
            right(abs(angle_to_rotate))
            time.sleep(0.1)
            
        elif (((image_center_X - object_center_pixel_threshold)<=object_X) and (object_X<=(image_center_X + object_center_pixel_threshold))):
            forward(0.05)
            open_servo()

            time.sleep(0.1)



    elif (radius>=190):
        open_servo()
        forward(0.1)
        close_servo()
        time.sleep(0.5)
        # reverse(0.4)
        # left(90)
        # forward(0.2)
        # command = 'raspistill -w 1280 -h 720 -vf -hf -o green_block_kulbir' + '.jpg'
        # command = "raspistill -o green_block_kulbir.jpg"object_Y
        # sendEmail('green_block_kulbir')
        break

    
    # cv2.imshow("cam",image)

    out.write(image)


    # cv2.imshow("cam",image)
    # out.write(image)
    # timeElapsed = time.time() - startTime 
    # timeElapsedlist.append(timeElapsed)

    # open .txt file to save data
    # f = open('time_elapsed_data.txt','a')
    # print time to run through loop to the screen & save to file
    # f.write(str(timeElapsed))



    # f.write('\n')
    # print('time elapsed for frame', timeElapsed)

    k = cv2.waitKey(50) & 0xFF
    if k == 27: 
        break


# f.close()
cam.release()
# out.release()
cv2.destroyAllWindows() 

# left(45)
# right(45)

file = open('path_list.txt','w')

for i in path_list:
    file.write(str(i))
    file.write('\n')
file.close()

gpio.cleanup()













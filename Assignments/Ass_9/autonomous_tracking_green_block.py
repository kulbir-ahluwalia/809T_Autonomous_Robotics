import cv2
import imutils
import numpy as np
import matplotlib.pyplot as plt
import time
import RPi.GPIO as gpio



def init():
    gpio.setmode(gpio.BOARD)
    gpio.setup(31,gpio.OUT) #IN1
    gpio.setup(33,gpio.OUT) #IN2
    gpio.setup(35,gpio.OUT) #IN3
    gpio.setup(37,gpio.OUT) #IN4
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
    gpio.output(31,False)
    gpio.output(33,False)
    gpio.output(35,False)
    gpio.output(37,False)
    # gpio.cleanup()
#MAIN CODE
    
# init()

def forward(distance):
    init()
    # print("How much distance do you want to go forward?")
    # dist = float(input())
    dist = distance
    # number_of_ticks = int(97.94*dist)
    number_of_ticks = int(98*dist)
    print("number of ticks required: ",number_of_ticks)


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

    list_of_gpioBR = []
    list_of_gpioFL = []






    # for i in range(0,2000000000000):
    while True:
        # time.sleep(0.05)
        print("Back_Right_counter = ",Back_Right_counter,"GPIO state at Pin 12: ",gpio.input(12))
        print("Front_Left_counter = ",Front_Left_counter,"GPIO state at Pin 7: ",gpio.input(7))
        list_of_gpioBR.append(gpio.input(12))
        list_of_gpioFL.append(gpio.input(7))
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
            print("THANKS")
            #  gpio.cleanup()
            break

    file = open('BR_gpio_states.txt','w')
    for i in list_of_gpioBR:
        file.write(str(i))
        file.write('\n')
    file.close()


    file = open('FL_gpio_states.txt','w')
    for i in list_of_gpioFL:
        file.write(str(i))
        file.write('\n')
    file.close()


def left(left_angle):
    init()

    print("How much angle in degrees do you want to go left?")
    # dist = float(input())
    # angle = float(input())
    angle = left_angle
    # number_of_ticks = int(97.94*dist)

    angular_distance = 0.0012654*angle*1.42
    number_of_ticks = int(98*angular_distance)


    # number_of_ticks = int()

    # number_of_ticks = int(98*dist)
    print("number of ticks required: ",number_of_ticks)


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
    pwm_FL = gpio.PWM(33,50)

    val = 55

    pwm_BR.start(val)
    pwm_FL.start(val)

    time.sleep(0.1)

    list_of_gpioBR = []
    list_of_gpioFL = []






    # for i in range(0,2000000000000):
    while True:
        # time.sleep(0.05)
        print("Back_Right_counter = ",Back_Right_counter,"GPIO state at Pin 12: ",gpio.input(12))
        print("Front_Left_counter = ",Front_Left_counter,"GPIO state at Pin 7: ",gpio.input(7))
        list_of_gpioBR.append(gpio.input(12))
        list_of_gpioFL.append(gpio.input(7))
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
            print("THANKS")
            #  gpio.cleanup()
            break

    file = open('BR_gpio_states.txt','w')
    for i in list_of_gpioBR:
        file.write(str(i))
        file.write('\n')
    file.close()


    file = open('FL_gpio_states.txt','w')
    for i in list_of_gpioFL:
        file.write(str(i))
        file.write('\n')
    file.close()




def right(right_angle):
    init()

    # print("How much distance do you want to go backward?")

    # print("How much angle in degrees do you want to go right?")
    # dist = float(input())
    angle = right_angle
    # number_of_ticks = int(97.94*dist)

    angular_distance = 0.0012654*angle*1.4385
    number_of_ticks = int(98*angular_distance)


    # number_of_ticks = int()

    # number_of_ticks = int(98*dist)
    print("number of ticks required: ",number_of_ticks)


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
    pwm_FL = gpio.PWM(31,50)

    val = 48

    pwm_BR.start(val)
    pwm_FL.start(val)

    time.sleep(0.1)

    list_of_gpioBR = []
    list_of_gpioFL = []






    # for i in range(0,2000000000000):
    while True:
        # time.sleep(0.05)
        print("Back_Right_counter = ",Back_Right_counter,"GPIO state at Pin 12: ",gpio.input(12))
        print("Front_Left_counter = ",Front_Left_counter,"GPIO state at Pin 7: ",gpio.input(7))
        list_of_gpioBR.append(gpio.input(12))
        list_of_gpioFL.append(gpio.input(7))
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
            print("THANKS")
            #  gpio.cleanup()
            break

    file = open('BR_gpio_states.txt','w')
    for i in list_of_gpioBR:
        file.write(str(i))
        file.write('\n')
    file.close()


    file = open('FL_gpio_states.txt','w')
    for i in list_of_gpioFL:
        file.write(str(i))
        file.write('\n')
    file.close()






image = cv2.imread("all_blocks.jpg")

image = imutils.resize(image, width = 640)
# cv2.imshow("base image", image)

hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
# cv2.imshow("hsv image", hsv)

# green_light_lower_range = np.array([75,100,100])
# green_light_upper_range = np.array([87,255,255])

green_light_lower_range = np.array([47,103,29])
green_light_upper_range = np.array([99,255,174])

mask = cv2.inRange(hsv, green_light_lower_range, green_light_upper_range)

# mask = cv2.inRange(hsv, (155,95,85), (165,105,255))
# cv2.imshow("mask", mask)


image_edges = cv2.Canny(mask,30,200)
# cv2.imshow("edges using canny edge detector", image_edges)

# contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
contours = cv2.findContours(image_edges.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
contours = imutils.grab_contours(contours)

# print("contours = " + str(len(contours)))

# see the contours array
print("contours = ", contours)

cv2.drawContours(mask, contours, -1, (255,0,0), 3)
# cv2.imshow("with contours on mask", mask)

contour_points = contours[0]
(x,y),radius = cv2.minEnclosingCircle(contour_points)

circle_center = (int(x),int(y))

radius = int(radius)
cv2.circle(image,circle_center,radius,(238,130,238),3)
cv2.circle(image,circle_center,2,(0,0,0),3)

cv2.drawContours(image, [contour_points], 0, (238,0,0), 3)
cv2.imshow("final_image", image)




# # press the 'q' key to close all images
# key = cv2.waitKey(0) & 0xFF
# print(key)

# if key == ord("q"):
#     cv2.destroyAllWindows()

def nothing(x):
    pass

# image_center = (320,240)



cam= cv2.VideoCapture(0)
fps = cam.get(cv2.CAP_PROP_FPS)
timestamps = [cam.get(cv2.CAP_PROP_POS_MSEC)]
calc_timestamps = [0.0]

#text parameters
font = cv2.FONT_HERSHEY_SIMPLEX 
fontScale = 0.5
font_color = (0, 0, 255)
font_thickness = 1
direction_scale = 1
direction_thickness = 2

#videowrite
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('green_light_detection_video.avi', fourcc, 20.0, (640,480))
timeElapsedlist = []
while True:
    ret, image=cam.read()
    image = imutils.resize(image, width = 640)
    print(image.shape)
    image = cv2.flip(image,-1)
    startTime = time.time()
    # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    # thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
    # frame = cv2.flip(thresh,1)
   
    hsv_img = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
#     # create the Mask
#     lower_range = np.array([l_h,l_s,l_v])
#     upper_range = np.array([u_h,u_s,u_v])
    lower_range = np.array([47,103,29])
    upper_range = np.array([99,255,174])
    mask = cv2.inRange(hsv_img, lower_range, upper_range)
    # image_edges = cv2.Canny(mask,85,255)

    #using contours
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)


    for c in cnts:
        M = cv2.moments(c)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = 0, 0

        # centre_x= int((min(cX)+max(cX))/2)
        # centre_y = int((min(cY)+max(cY))/2)
        centre_x= cX
        centre_y= cY
        # dist_2 = calc_dist((centre_x,centre_y))


        contour_points = cnts[0]
        (x,y),radius = cv2.minEnclosingCircle(contour_points)
        center = (int(x),int(y))
        radius = int(radius)
        # cv2.circle(image, (cX, cY), 15, (0,0,255), 8)  
        cv2.circle(image, (cX, cY), radius, (0,0,255), 3) 
        cv2.circle(image,(cX, cY),2,(0,0,0),3)
        block_pixel_location = [cX,cY]
        
        #draw crosshair
        cv2.line(image,(320,120),(320,360),(0,0,0),2)
        cv2.line(image,(200,240),(440,240),(0,0,0),2)
        cv2.putText(image,str(block_pixel_location),(20,35),cv2.FONT_HERSHEY_DUPLEX,direction_scale,(0,0,255), direction_thickness)

        pixel_to_rotate = 320-block_pixel_location[0]
        print("pixel_to_rotate",pixel_to_rotate)
        degree_to_rotate = 0.061*pixel_to_rotate
        print("degree_to_rotate",degree_to_rotate)

        pixel_center_range = 120
        angle_to_rotate = 5


    if(pixel_to_rotate<-pixel_center_range):
        right(angle_to_rotate)
        
    elif(pixel_to_rotate>pixel_center_range):
        left(angle_to_rotate)
        
    elif(-pixel_center_range<=pixel_to_rotate<=pixel_center_range):
        motors_off()










    cv2.imshow("cam",image)
    out.write(image)
    timeElapsed = time.time() - startTime 
    timeElapsedlist.append(timeElapsed)
    # open .txt file to save data
    f = open('time_elapsed_data.txt','a')
    # print time to run through loop to the screen & save to file
    f.write(str(timeElapsed))
    f.write('\n')
    print('time elapsed for frame', timeElapsed)
    k = cv2.waitKey(50) & 0xFF
    if k == 27: 
        break
f.close()
cam.release()
out.release()
cv2.destroyAllWindows() 










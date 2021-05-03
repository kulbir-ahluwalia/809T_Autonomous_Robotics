#automatically rotate the wheel for one rotation and test the encoder
import RPi.GPIO as gpio
import time
import numpy as np
import serial


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
    gpio.output(31,False)
    gpio.output(33,False)
    gpio.output(35,False)
    gpio.output(37,False)
    gpio.cleanup()

    

    
init()

#MAIN CODE



#initialize pwm signal to control motor
angle = 90 #USER DEFINED ANGLE
time_left_turn  = ((angle*1.3)/90)


ser = serial.Serial('/dev/ttyUSB0',9600)

count = 0
new_x_angle = 0

list_of_gpio = []
list_of_gpio_2 = []
list_of_x = []


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
            gameover()
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
            gameover()
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





# def forward(time_to_run,ser):
#     pin = 31
#     pin2 = 37
#     val = 30
#     pwm1 = gpio.PWM(pin,50)
#     pwm1.start(val)
#     pwm4 = gpio.PWM(pin2,50)
#     pwm4.start(val)
#     t = time.time()
#     counter = np.uint64(0)
#     counter2 = np.uint64(0) 
#     button = int(0)
#     button2 = int(0)
#     while time.time()-t<time_to_run:
#         #time.sleep(0.1)
#         line = ser.readline()
#         print('LINE', line)
#         line = line.rstrip().lstrip()
#             #print(line)
            
#         line = str(line)
#         line = line.strip("'")
#         line = line.strip("b'")
#         print("imu=",line)
#         list_of_x.append(line[3:])
#         if int (gpio.input(12)) != int(button):
#             button = int(gpio.input(12))
#             counter+= 1
#         if int (gpio.input(7)) != int(button2):
#             button2 = int(gpio.input(7))
#             counter2+=1
            
#         #PROPORTIONAL CONTROLLER
#         err = counter - counter2
#         print(err)
#         kp = 1.0
#         if err<0:
#             pwm1.ChangeDutyCycle(val + (err*kp))
#             #time.sleep(0.1)
#         elif err>=0:
#             pwm1.ChangeDutyCycle(val - (err*kp))
#             #time.sleep(0.1)
            
#     list_of_gpio.append(counter)
#     list_of_gpio_2.append(counter2)
    
# def left(val):
#     pin = 33
#     pin2 = 37
#     pwm1 = gpio.PWM(pin,50)
#     pwm1.start(val)
#     pwm4 = gpio.PWM(pin2,50)
#     pwm4.start(val)
#     t = time.time()
#     counter = np.uint64(0)
#     counter2 = np.uint64(0) 
#     button = int(0)
#     button2 = int(0)
#     time.sleep(0.05)
#     if int (gpio.input(12)) != int(button):
#         button = int(gpio.input(12))
#         counter+= 1
#     if int (gpio.input(7)) != int(button2):
#         button2 = int(gpio.input(7))
#         counter2+=1   


# def get_imu_angle():

#     print(ser.readline()[5:9])

#     return float(ser.readline()[5:9])

#    if(ser.in_waiting > 0):
#         # count+=1


#         #read serial stream
#         line = ser.readline()
#         print(line)

#         #avoid first n lines of serial info
#         # if(count>10):

#         #strip serial stream of extra characters
#         line = line.rstrip().lstrip()
#         print(line)

#         line = str(line)
#         line = line.strip("'")
#         line = line.strip("b'")
#         print(line)


#         #return float
#         line = float(line)

#         print(line,"\n") 
#         return line





# def turn_left_imu(val):

#     new_x_angle = 0
#     data = ser.readline()[5:9]
#     print(data)
#     curr_x = float(ser.readline()[5:9])

#     print('curr_x is : ',curr_x)

#     while (abs(curr_x-new_x_angle))<=90:
#         curr_x = float(ser.readline()[5:9])
#         print(curr_x)
#         left(val)
#         line = ser.readline()
#         new_x_angle = float(line[5:9])
#         print('angle diff between: ',curr_x ,'and',new_x_angle, '=',abs(curr_x-new_x_angle))
        




while True:
    if(ser.in_waiting > 0):
        count+=1
        
        line = ser.readline()
        #print(line)
        
        
        if(count>10):
            


            time_front = 10
            time_front_2 = 5
            delay_between = 0.9
            val =  50

            forward(1)
            time.sleep(delay_between)
            curr_x = float(ser.readline()[5:9])
            print('curr_x is : ',curr_x)
            while (abs(curr_x-new_x_angle))<=90:
                line = ser.readline()
                new_x_angle = float(line[5:9])
                print('angle diff between: ',curr_x ,'and',new_x_angle, '=',abs(curr_x-new_x_angle))
                left(10)
            time.sleep(delay_between)
            print('FIRST SIDE DONE')
            new_x_angle = 0
            forward(time_front_2,ser)
            time.sleep(delay_between)
            curr_x = float(ser.readline()[5:9])
            print('curr_x is : ',curr_x)
            while (abs(curr_x-new_x_angle))<=90:
                line = ser.readline()
                new_x_angle = float(line[5:9])
                print('angle diff between: ',curr_x ,'and',new_x_angle, '=',abs(curr_x-new_x_angle))
                left(val)
            time.sleep(delay_between)
            print('SECOND SIDE DONE')
            new_x_angle = 0
            forward(time_front,ser)
            time.sleep(delay_between)
            curr_x = float(ser.readline()[5:9])
            print('curr_x is : ',curr_x)
            while (abs(curr_x-new_x_angle))<=90:
                line = ser.readline()
                new_x_angle = float(line[5:9])
                print('angle diff between: ',curr_x ,'and',new_x_angle, '=',abs(curr_x-new_x_angle))
                left(val)
            time.sleep(delay_between)
            print('THIRD SIDE DONE')
            new_x_angle = 0
            forward(time_front_2,ser)
            time.sleep(delay_between)
            curr_x = float(ser.readline()[5:9])
            print('curr_x is : ',curr_x)
            while (abs(curr_x-new_x_angle))<=90:
                line = ser.readline()
                new_x_angle = float(line[5:9])
                print('angle diff between: ',curr_x ,'and',new_x_angle, '=',abs(curr_x-new_x_angle))
                left(val)
            time.sleep(delay_between)
            break
            print('FOURTH SIDE DONE')
print('PROCESS DONE!')
print(list_of_gpio)
print(list_of_gpio_2)

file = open('gpio_values_05_ALL_fOUR_1.txt','w')
for i in list_of_gpio:
    file.write(str(i))
    file.write('\n')
file.close()
    
file = open('gpio_values_05_ALL_fOUR_2.txt','w')
for i in list_of_gpio_2:
    file.write(str(i))
    file.write('\n')
file.close()
    
file = open('imu_x.txt','w')
for i in list_of_x:
    file.write(str(i))
    file.write('\n')
file.close()

# forward(4,ser)
# left(50)

# get_imu_angle()

# turn_left_imu(50)

gameover()
gpio.cleanup()





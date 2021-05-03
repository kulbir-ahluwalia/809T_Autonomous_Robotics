#automatically rotate the wheel for one rotation and test the encoder
import RPi.GPIO as gpio
import time
import numpy as np

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

def motors_off():
    gpio.output(31,False)
    gpio.output(33,False)
    gpio.output(35,False)
    gpio.output(37,False)
    # gpio.cleanup()
#MAIN CODE
    
init()

print("How much distance do you want to go backward?")
dist = float(input())
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

pwm_BR = gpio.PWM(35,50)
pwm_FL = gpio.PWM(33,50)

val = 40

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

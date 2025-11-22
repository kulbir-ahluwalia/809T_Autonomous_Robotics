import RPi.GPIO as GPIO
import time
import numpy as np
import cv2
import datetime
import os
import glob

GPIO.setmode(GPIO.BOARD)
GPIO.setup(36,GPIO.OUT)

pwm = GPIO.PWM(36,50)

#opens completely
pwm.start(7.7)
time.sleep(0.5)

# # fully closes at 3.4% duty cycle
# pwm.ChangeDutyCycle(3.4) 
# time.sleep(0.5)
os.system("sudo mkdir servo_pics")


# define the codec and create VideoWriter object
fps_out = 1
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter("servo_pwm" + ".avi", fourcc, fps_out, (1280, 720))

direction_scale = 1
direction_thickness = 2

# pwm.ChangeDutyCycle(7) 
def close():
        
    i = 0
    duty_cycles1 = [x for x in np.arange(7.7,3.4,-2)]
    print(duty_cycles1)

    for i in duty_cycles1:

        pwm.ChangeDutyCycle(i) 
        time.sleep(0.2)

        image = "sudo raspistill -w 1280 -h 720 -hf -vf -o /home/pi/Desktop/809T_Autonomous_Robotics/Assignments/Ass_6/servo_pics/" + str(i) + ".jpg"
        os.system(image)

        image = cv2.imread(str(i) + ".jpg")

        text = "Duty: " + str(i) + "%"

        cv2.putText(image,text,(50,50),cv2.FONT_HERSHEY_DUPLEX,direction_scale,(0,0,255), direction_thickness)
        # cv2.imshow("image",image)
        cv2.imwrite(str(i) + ".jpg",image)        
        print("Saved image: ", i)
        time.sleep(0.1)


def open():
        
    i = 0
    duty_cycles2 = [x for x in np.arange(3.4,7.7,2)]
    print(duty_cycles2)

    for i in duty_cycles2:

        pwm.ChangeDutyCycle(i) 
        time.sleep(0.2)

        image = "sudo raspistill -w 1280 -h 720 -hf -vf -o /home/pi/Desktop/809T_Autonomous_Robotics/Assignments/Ass_6/servo_pics/" + str(i) + ".jpg"
             


        os.system(image)

        print("Saved image: ", i)
        time.sleep(0.1)

# open()

close()

open()

# close()

pwm.stop()
GPIO.cleanup()


# # find all images recorded during the run
# files = glob.glob("servo_pics/*.jpg")
# print(files)
# files = sorted(glob.glob("servo_pics/*.jpg"))
# print(files)



# # loop through and print frames to video file
# for x in files:
# 	print(x)
# 	image_out = cv2.imread(x)
# 	out.write(image_out)

print("Video_servo" + ".avi is now ready for viewing!")

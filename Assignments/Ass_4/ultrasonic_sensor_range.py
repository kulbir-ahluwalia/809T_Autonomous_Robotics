import RPi.GPIO as gpio
import time
import imutils
## Capturing image
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2

# initialize camera
camera = PiCamera()
camera.rotation = 180

rawCapture = PiRGBArray(camera)

# warmup for camera
time.sleep(0.1)


camera.capture(rawCapture, format="bgr")
obstacle_image = rawCapture.array
obstacle_image = imutils.resize(obstacle_image, width = 720)
average_dist_image = imutils.resize(obstacle_image, width = 720)



font = cv2.FONT_HERSHEY_SIMPLEX 
org = (50, 50) 
fontScale = 1
color = (0, 255, 0)
thickness = 2

image = cv2.putText(obstacle_image, 'Obstacle_Image', org, font,  
                   fontScale, color, thickness, cv2.LINE_AA) 

cv2.imshow("Obstacle_Image", obstacle_image)


trig =16
echo = 18

def distance():
	gpio.setmode(gpio.BOARD)
	gpio.setup(trig, gpio.OUT)
	gpio.setup(echo,gpio.IN)

	#ensure output is low = no value
	gpio.output(trig, False)
	time.sleep(0.01)

	#generate trigger pulse
	gpio.output(trig, True)
	time.sleep(0.00001)
	gpio.output(trig, False)

	#generate echo time signal
	while gpio.input(echo) == 0:
		pulse_start = time.time()

	while gpio.input(echo) == 1:
		pulse_end = time.time()
	
	pulse_duration =  pulse_end - pulse_start
	
	# convert time to distance
	distance = pulse_duration*17150
	distance = round(distance,2)

	# cleanup gpio pins  and return distance estimate
	gpio.cleanup()

	return distance

distance_list = []

for i in range(0,10,1):

    distance_temp = distance()
    print("Reading number: ", i+1)
    print("distance from obstacle in cm is: ", distance_temp)
    distance_list.append(distance_temp)

    time.sleep(1.0)
    
print("List of distance readings: ", distance_list)

Average_reading = sum(distance_list) / len(distance_list)

print("Average reading: ", Average_reading)


avg_dist_image = cv2.putText(average_dist_image, 'Average obstacle distance: ' + str(Average_reading), org, font,  
                   fontScale, color, thickness, cv2.LINE_AA) 

cv2.imshow("Average distance image", average_dist_image)





# press the 'q' key to close all images
key = cv2.waitKey(0) & 0xFF
print(key)

if key == ord("q"):
    cv2.destroyAllWindows()














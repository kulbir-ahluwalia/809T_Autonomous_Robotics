import cv2
import imutils
import numpy as np
import matplotlib.pyplot as plt
import time

image = cv2.imread("traffic_light_kulbir_final.jpg")

image = imutils.resize(image, width = 720)
cv2.imshow("base image", image)

hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
cv2.imshow("hsv image", hsv)

green_light_lower_range = np.array([75,100,100])
green_light_upper_range = np.array([87,255,255])

mask = cv2.inRange(hsv, green_light_lower_range, green_light_upper_range)

# mask = cv2.inRange(hsv, (155,95,85), (165,105,255))
cv2.imshow("mask", mask)


image_edges = cv2.Canny(mask,30,200)
cv2.imshow("edges using canny edge detector", image_edges)

contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
# print("contours = " + str(len(contours)))

# see the contours array
print("contours = ", contours)

cv2.drawContours(mask, contours, -1, (255,0,0), 3)
cv2.imshow("with contours on mask", mask)

contour_points = contours[0]
(x,y),radius = cv2.minEnclosingCircle(contour_points)

circle_center = (int(x),int(y))

radius = int(radius)
# cv2.circle(image,circle_center,radius,(238,130,238),3)
cv2.circle(image,circle_center,2,(0,0,0),3)

cv2.drawContours(image, [contour_points], 0, (238,0,0), 3)
cv2.imshow("final_image", image)




# press the 'q' key to close all images
key = cv2.waitKey(0) & 0xFF
print(key)

if key == ord("q"):
    cv2.destroyAllWindows()



def nothing(x):
    pass

cam= cv2.VideoCapture(0)
fps = cam.get(cv2.CAP_PROP_FPS)
timestamps = [cam.get(cv2.CAP_PROP_POS_MSEC)]
calc_timestamps = [0.0]

#videowrite
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('green_light_detection_video.avi', fourcc, 20.0, (640,480))
timeElapsedlist = []
while True:
    ret, image=cam.read()
    startTime = time.time()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
    frame = cv2.flip(thresh,1)
        # write the flipped frame
#     l_h = cv2.getTrackbarPos("L-H","Trackbars")
#     l_s = cv2.getTrackbarPos("L-S","Trackbars")
#     l_v = cv2.getTrackbarPos("L-V","Trackbars")
#     u_h = cv2.getTrackbarPos("U-H","Trackbars")
#     u_s = cv2.getTrackbarPos("U-S","Trackbars")
#     u_v = cv2.getTrackbarPos("U-V","Trackbars")
    hsv_img = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
#     # create the Mask
#     lower_range = np.array([l_h,l_s,l_v])
#     upper_range = np.array([u_h,u_s,u_v])
    lower_range = np.array([75,100,100])
    upper_range = np.array([87,255,255])
    mask = cv2.inRange(hsv_img, lower_range, upper_range)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    for c in cnts:
        M = cv2.moments(c)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = 0, 0
        (x,y),radius = cv2.minEnclosingCircle(contour_points)
        center = (int(x),int(y))
        radius = int(radius)
        cv2.circle(image, (cX, cY), 15, (0,0,255), 8)  
        cv2.circle(image, (cX, cY), radius, (0,0,255), 3) 
        cv2.circle(image,(cX, cY),2,(0,0,0),3)


    cv2.imshow("cam",image)
    out.write(image)
    timeElapsed = time.time() - startTime 
    timeElapsedlist.append(timeElapsed)
    # open .txt file to save data
    f = open('time_elapsed_data.txt','a')
    # print time to run through loop to the screen & save to file
    f.write(str(timeElapsed))
    f.write('\n')
    print('time elapsed in this frame is > > > > ', timeElapsed)
    k = cv2.waitKey(100) & 0xFF
    if k == 27: 
        break
f.close()
cam.release()
out.release()
cv2.destroyAllWindows() 










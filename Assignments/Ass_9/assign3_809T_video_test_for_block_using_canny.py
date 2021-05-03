import cv2
import imutils
import numpy as np
import matplotlib.pyplot as plt
import time

# image = cv2.imread("all_blocks.jpg")
image = cv2.imread("green_block_up_close.jpg")

image = imutils.resize(image, width = 640)
cv2.imshow("base image", image)

hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
cv2.imshow("hsv image", hsv)

# green_light_lower_range = np.array([75,100,100])
# green_light_upper_range = np.array([87,255,255])

#far range
# green_light_lower_range = np.array([47,103,29])
# green_light_upper_range = np.array([99,255,174])

#close range
green_light_lower_range = np.array([59,98,8])
green_light_upper_range = np.array([75,255,173])


mask = cv2.inRange(hsv, green_light_lower_range, green_light_upper_range)
cv2.imwrite('masked_green_block.png',mask)

# mask = cv2.inRange(hsv, (155,95,85), (165,105,255))
# cv2.imshow("mask", mask)


image_edges = cv2.Canny(mask,77,132)
cv2.imshow("edges using canny edge detector", image_edges)

contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
# print("contours = " + str(len(contours)))

# # see the contours array
# print("contours = ", contours)

cv2.drawContours(mask, contours, -1, (255,0,0), 3)
cv2.imshow("with contours on mask", mask)

contour_points = contours[0]
(x,y),radius = cv2.minEnclosingCircle(contour_points)

circle_center = (int(x),int(y))

radius = int(radius)
print("radius is",radius)
cv2.circle(image,circle_center,radius,(238,130,238),3)
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

def calc_dist(pt1):
    pt2 = (320,240)
    dist = 320-pt1[0]
    return dist

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

    # #far range
    # lower_range = np.array([47,103,29])
    # upper_range = np.array([99,255,174])
    
    #close range
    lower_range = np.array([59,98,8])
    upper_range = np.array([75,255,173])



    mask = cv2.inRange(hsv_img, lower_range, upper_range)
    edged = cv2.Canny(mask, 30, 200)
    cv2.imshow("with contours on mask", edged)


    # _,contours,_= cv2.findContours(edged,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # contours_poly = [None]*len(contours)
    # centers = [None]*len(contours)
    # radius = [None]*len(contours)
    # cx= []
    # cy = []
    # for i, c in enumerate(contours):
    #     contours_poly[i] = cv2.approxPolyDP(c, 3, True)
    #     centers[i], radius[i] = cv2.minEnclosingCircle(contours_poly[i])
    #     cx.append(centers[i][0])
    #     cy.append(centers[i][1])

    # centre_x= int((min(cx)+max(cx))/2)
    # print("centre_x_is:",centre_x)

    # centre_y = int((min(cy)+max(cy))/2)
    # print("centre_y_is:",centre_y)

    # dist_2 = calc_dist((centre_x,centre_y))
    
    
    # # if(dist_2<-100):
    # #     right(val)
        
    # # elif(dist_2>100):
    # #     left(val)
        
    # # elif(-100<=dist_2<=100):
    # #     stop()
        
        
    # print("dist_2",dist_2)
    
    # # draw(orig_image)
    # # cv2.putText(orig_image,str(dist_2),(500,240),cv2.FONT_HERSHEY_COMPLEX,1,(255,255,0),2,cv2.LINE_AA)

    # draw(edged)
    # cv2.putText(edged,str(dist_2),(500,240),cv2.FONT_HERSHEY_COMPLEX,1,(255,255,0),2,cv2.LINE_AA)
    # cv2.imshow("cam",edged)

    
    
    # #using canny




    
    
    
    
    
    # using contours
    cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    print(cnts)

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










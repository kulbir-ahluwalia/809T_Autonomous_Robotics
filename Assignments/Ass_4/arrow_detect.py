import cv2
import imutils
import numpy as np
import matplotlib.pyplot as plt
import time

image = cv2.imread("green_arrow1.jpg")

image = cv2.rotate(image, cv2.ROTATE_180)

image = imutils.resize(image, width = 720)
cv2.imshow("green arrow image", image)

h = image.shape[0]
w = image.shape[1]
no_arrow_detected_background_image = np.zeros((h,w,3),np.uint8) 

hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
cv2.imshow("hsv arrow image", hsv)

green_light_lower_range = np.array([45,105,95])
green_light_upper_range = np.array([95,255,255])

mask = cv2.inRange(hsv, green_light_lower_range, green_light_upper_range)

# mask = cv2.inRange(hsv, (155,95,85), (165,105,255))
cv2.imshow("masked image", mask)

blurred_masked_image = cv2.GaussianBlur(mask,(9,9),0)
cv2.imshow("blurred mask image", blurred_masked_image)

corners = cv2.goodFeaturesToTrack(blurred_masked_image,7,0.01,12)
print(corners)

corners = np.int0(corners)
blurred_image_with_corners = image.copy()

font = cv2.FONT_HERSHEY_SIMPLEX 
fontScale = 0.5
font_color = (0, 0, 255)
font_thickness = 1

# image = cv2.putText(obstacle_image, 'Obstacle_Image', org, font,  
#                    fontScale, color, thickness, cv2.LINE_AA) 

x_coords_corners = []
y_coords_corners = []



for i in corners:
    x,y = i.ravel()
    cv2.circle(blurred_image_with_corners,(x,y),5,(255,0,0),-1)
    point_coord_string = "("+str(x)+","+str(y)+")"
    x_coords_corners.append(x)
    y_coords_corners.append(y)


    cv2.putText(blurred_image_with_corners,point_coord_string, (x,y),font,fontScale,font_color,font_thickness)

print("X Coords of corners are: ", x_coords_corners)
print("Y Coords of corners are: ", y_coords_corners)

cv2.imshow("blurred mask image with corners", blurred_image_with_corners)

x_range = max(x_coords_corners) - min(x_coords_corners)
y_range = max(y_coords_corners) - min(y_coords_corners)
x_mid = min(x_coords_corners) + x_range/2
y_mid = min(y_coords_corners) + y_range/2

x_counter_right =0
x_counter_left = 0

y_counter_up = 0
y_counter_down = 0

direction_scale = 1
direction_thickness = 2

if y_range<x_range:
    #left or right

    for x in x_coords_corners:

        if(x >= x_mid):
            x_counter_right += 1

        elif(x<x_mid):
            x_counter_left+=1

    print("x_counter_right is: ", x_counter_right)
    print("x_counter_left is: ", x_counter_left)

    if (x_counter_left < x_counter_right):
        cv2.putText(blurred_image_with_corners,'Right',(50,50),cv2.FONT_HERSHEY_DUPLEX,direction_scale,(0,0,255), direction_thickness)
    else:
        cv2.putText(blurred_image_with_corners,'Left',(50,50),cv2.FONT_HERSHEY_DUPLEX,direction_scale,(0,0,255), direction_thickness)

else: 
    #up or down
    for y in y_coords_corners:

        if(y >= y_mid):
            y_counter_up += 1

        elif(y<y_mid):
            y_counter_down+=1

    print("y_counter_up is: ", y_counter_up)
    print("y_counter_down is: ", y_counter_down)

    if (y_counter_up < y_counter_down):
        cv2.putText(blurred_image_with_corners,'Down',(50,50),cv2.FONT_HERSHEY_DUPLEX,direction_scale,(0,0,255),direction_thickness)
    else:
        cv2.putText(blurred_image_with_corners,'Up',(50,50),cv2.FONT_HERSHEY_DUPLEX,direction_scale,(0,0,255),direction_thickness)

cv2.imshow("blurred mask image with corners and direction", blurred_image_with_corners)




# press the 'q' key to close all images
key = cv2.waitKey(0) & 0xFF
print(key)

if key == ord("q"):
    cv2.destroyAllWindows()



cam= cv2.VideoCapture(0)
fps = cam.get(cv2.CAP_PROP_FPS)
timestamps = [cam.get(cv2.CAP_PROP_POS_MSEC)]
calc_timestamps = [0.0]

#videowrite
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('arrow_detection_green.avi', fourcc, 20.0, (640,480))
timeElapsedlist = []
while True:
    ret, image=cam.read()
    image = cv2.rotate(image, cv2.ROTATE_180)
    startTime = time.time()
    # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    # thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
    # frame = cv2.flip(thresh,1)
   
    hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
#     # create the Mask
#     lower_range = np.array([l_h,l_s,l_v])
#     upper_range = np.array([u_h,u_s,u_v])
    green_light_lower_range = np.array([45,105,95])
    green_light_upper_range = np.array([95,255,255])


    mask = cv2.inRange(hsv, green_light_lower_range, green_light_upper_range)
    blurred_masked_image = cv2.GaussianBlur(mask,(9,9),0)

    corners = cv2.goodFeaturesToTrack(blurred_masked_image,7,0.01,12)
    print(corners)

    h = image.shape[0]
    w = image.shape[1]
    no_arrow_detected_background_image = np.zeros((h,w,3),np.uint8) 

    

    if (np.any(corners)):
        corners = np.int0(corners)
        blurred_image_with_corners = image.copy()
        

        font = cv2.FONT_HERSHEY_SIMPLEX 
        fontScale = 0.5
        font_color = (0, 0, 255)
        font_thickness = 1

        
        x_coords_corners = []
        y_coords_corners = []



        for i in corners:
            x,y = i.ravel()
            cv2.circle(blurred_image_with_corners,(x,y),5,(255,0,0),-1)
            point_coord_string = "("+str(x)+","+str(y)+")"
            x_coords_corners.append(x)
            y_coords_corners.append(y)


            cv2.putText(blurred_image_with_corners,point_coord_string, (x,y),font,fontScale,font_color,font_thickness)

        print("X Coords of corners are: ", x_coords_corners)
        print("Y Coords of corners are: ", y_coords_corners)

        # cv2.imshow("blurred mask image with corners", blurred_image_with_corners)

        x_range = max(x_coords_corners) - min(x_coords_corners)
        y_range = max(y_coords_corners) - min(y_coords_corners)
        x_mid = min(x_coords_corners) + x_range/2
        y_mid = min(y_coords_corners) + y_range/2

        x_counter_right =0
        x_counter_left = 0

        y_counter_up = 0
        y_counter_down = 0

        direction_scale = 1
        direction_thickness = 2

        if y_range<x_range:
            #left or right

            for x in x_coords_corners:

                if(x >= x_mid):
                    x_counter_right += 1

                elif(x<x_mid):
                    x_counter_left+=1

            print("x_counter_right is: ", x_counter_right)
            print("x_counter_left is: ", x_counter_left)

            if (x_counter_left < x_counter_right):
                cv2.putText(blurred_image_with_corners,'Right',(50,50),cv2.FONT_HERSHEY_DUPLEX,direction_scale,(0,0,255), direction_thickness)
            else:
                cv2.putText(blurred_image_with_corners,'Left',(50,50),cv2.FONT_HERSHEY_DUPLEX,direction_scale,(0,0,255), direction_thickness)

        else: 
            #up or down
            for y in y_coords_corners:

                if(y >= y_mid):
                    y_counter_up += 1

                elif(y<y_mid):
                    y_counter_down+=1

            print("y_counter_up is: ", y_counter_up)
            print("y_counter_down is: ", y_counter_down)

            if (y_counter_up < y_counter_down):
                cv2.putText(blurred_image_with_corners,'Up',(50,50),cv2.FONT_HERSHEY_DUPLEX,direction_scale,(0,0,255),direction_thickness)
            else:
                cv2.putText(blurred_image_with_corners,'Down',(50,50),cv2.FONT_HERSHEY_DUPLEX,direction_scale,(0,0,255),direction_thickness)

        cv2.imshow("corners and direction",blurred_image_with_corners)
        out.write(blurred_image_with_corners)

    else:
        cv2.putText(no_arrow_detected_background_image,'No arrow detected',(50,50),cv2.FONT_HERSHEY_DUPLEX,direction_scale,(0,0,255),direction_thickness)
        cv2.imshow("corners and direction",no_arrow_detected_background_image)
        out.write(no_arrow_detected_background_image)
        print("No arrow detected")
        



    
    timeElapsed = time.time() - startTime 
    timeElapsedlist.append(timeElapsed)
    # open .txt file to save data
    f = open('time_elapsed_data_Ass4.txt','a')
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










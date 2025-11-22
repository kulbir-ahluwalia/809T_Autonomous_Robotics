
from move_actions import *

import os
import smtplib
from smtplib import SMTP
from smtplib import SMTPException
import email
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage


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






image = cv2.imread("all_blocks_arena.jpg")

image = imutils.resize(image, width = 640)
# cv2.imshow("base image", image)

hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
# cv2.imshow("hsv image", hsv)

# green_light_lower_range = np.array([75,100,100])
# green_light_upper_range = np.array([87,255,255])

# #room parameters
# green_light_lower_range = np.array([47,103,29])
# green_light_upper_range = np.array([99,255,174])

#arena parameters
green_light_lower_range = np.array([41,105,20])
green_light_upper_range = np.array([83,255,224])

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
# cv2.imshow("final_image", image)




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

open_servo()


while True:
    ret, image=cam.read()
    # image = imutils.resize(image, width = 640)
    # print(image.shape)
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
    lower_range = np.array([41,105,20])
    upper_range = np.array([83,255,224])
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


        distance_of_block_in_pixels = 240 - block_pixel_location[1]
        print("distance_of_block_in_pixels: ",distance_of_block_in_pixels)


        pixel_center_range = 50
        angle_to_rotate = 5

    if (distance_of_block_in_pixels>21):
            right(angle_to_rotate)
            time.sleep(0.3)
            
        elif(pixel_to_rotate>pixel_center_range):
            left(angle_to_rotate)
            time.sleep(0.3)
            
        elif(-pixel_center_range<=pixel_to_rotate<=pixel_center_range):
            forward(0.03)
            open_servo()
            time.sleep(0.2)

    elif (distance_of_block_in_pixels<=20):
        open_servo()
        forward(0.1)
        close_servo()
        time.sleep(0.5)
        reverse(0.4)
        left(90)
        forward(0.2)
        # command = 'raspistill -w 1280 -h 720 -vf -hf -o green_block_kulbir' + '.jpg'
        # command = "raspistill -o green_block_kulbir.jpg"
        # os.system(command)
        time.sleep(4)

        # sendEmail('green_block_kulbir')
        break














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










import cv2
import time
import os

command = 'sudo modprobe bcm2835-v4l2'
os.system(command)

# set up camera object
cap = cv2.VideoCapture(0)

def make_1080p():
    cap.set(3, 1920)
    cap.set(4, 1080)

def make_720p():
    cap.set(3, 1280)
    cap.set(4, 720)

def make_480p():
    cap.set(3, 640)
    cap.set(4, 480)

def change_res(width, height):
    cap.set(3, width)
    cap.set(4, height)


make_480p()
# make_720p()

# change_res(720, 1280)+
# make_1080p()


# QR code detection object
detector = cv2.QRCodeDetector()

frame_rate = 10
prev = 0 

while True:

    time_elapsed = time.time() - prev
    # get the image
    res, img = cap.read()
    print(img.shape)

    if time_elapsed > 1/frame_rate:
        prev = time.time()

        # get bounding box coords and data
        data, bbox, _ = detector.detectAndDecode(img)
        
        # if there is a bounding box, draw one, along with the data
        if(bbox is not None):
            for i in range(len(bbox)):
                cv2.line(img, tuple(bbox[i][0]), tuple(bbox[(i+1) % len(bbox)][0]), color=(255,
                        0, 255), thickness=2)
            cv2.putText(img, data, (int(bbox[0][0][0]), int(bbox[0][0][1]) - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 2)
            if data:
                print("data found: ", data)

    # display the image preview
    cv2.imshow("code detector", img)
    if(cv2.waitKey(1) == ord("q")):
        break
# free camera object and exit
cap.release()
cv2.destroyAllWindows()
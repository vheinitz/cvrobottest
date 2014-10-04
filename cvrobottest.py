''' Detect human skin tone and draw a boundary around it.
Useful for gesture recognition and motion tracking.

Inspired by: http://stackoverflow.com/a/14756351/1463143

Date: 08 June 2013
'''

# Required moduls
import cv2
import numpy
import serial

s = serial.Serial(port='/dev/ttyUSB0', baudrate=9600)



# Constants for finding range of skin color in YCrCb
min_YCrCb = numpy.array([0,133,77],numpy.uint8)
max_YCrCb = numpy.array([255,173,127],numpy.uint8)

# Create a window to display the camera feed
cv2.namedWindow('UI')

# Get pointer to video frames from primary device
videoFrame = cv2.VideoCapture(0)

# Process the video frames
keyPressed = -1 # -1 indicates no key pressed

bw = 150
bh = 75

lbx= 50
lby= 50


rbx= 430
rby= 50

while(keyPressed < 0): # any key pressed has a value >= 0

    # Grab video frame, decode it and return next video frame
    _, img = videoFrame.read()

    cv2.flip(img,1,img)

    rbdetected=False
    lbdetected=False

    roil= img[lby:lby+bh,lbx:lbx+bw];
    imageYCrCb = cv2.cvtColor(roil,cv2.COLOR_BGR2YCR_CB)
    skinRegion = cv2.inRange(imageYCrCb,min_YCrCb,max_YCrCb)
    contours, hierarchy = cv2.findContours(skinRegion, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for i, c in enumerate(contours):
        area = cv2.contourArea(c)
        if area > 500 and area < 1000:
            #cv2.drawContours(img, contours, i, (0, 255, 0), 3)
            lbdetected=True

    roir= img[rby:rby+bh, rbx:rbx+bw];
    imageYCrCb = cv2.cvtColor(roir,cv2.COLOR_BGR2YCR_CB)
    skinRegion = cv2.inRange(imageYCrCb,min_YCrCb,max_YCrCb)
    contours, hierarchy = cv2.findContours(skinRegion, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for i, c in enumerate(contours):
        area = cv2.contourArea(c)
        if area > 500 and area < 1000:
            #cv2.drawContours(img, contours, i, (0, 255, 0), 3)
            rbdetected=True


    if lbdetected:
        cv2.rectangle(img,(lbx,lby),(lbx+bw, lby+bh),[255,200,200],3)
        s.write(str(1))

    else:
        cv2.rectangle(img,(lbx,lby),(lbx+bw, lby+bh),[200,200,200],1)

    if rbdetected:
        cv2.rectangle(img,(rbx,rby),(rbx+bw, rby+bh),[255,200,200],3)
        s.write(str(3))
    else:
        cv2.rectangle(img,(rbx,rby),(rbx+bw, rby+bh),[200,200,200],1)

    if rbdetected or lbdetected:
        s.write(str(0))
    #dst = img;
    #cv2.flip(img,1,dst)
    cv2.imshow('UI',img)

    # Check for user input to close program
    keyPressed = cv2.waitKey(1) # wait 1 milisecond in each iteration of while loop

# Close window and camera after exiting the while loop
cv2.destroyWindow('UI')
videoFrame.release()

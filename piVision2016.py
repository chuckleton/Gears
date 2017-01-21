from __future__ import print_function
import pivideostream
from pivideostream import PiVideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import imutils
import time
import cv2
import serial
import numpy
 
lowerGreen = numpy.array([0, 190, 190])
upperGreen = numpy.array([100, 255, 255])


# created a *threaded *video stream, allow the camera sensor to warmup,

ser = serial.Serial(
    port = '/dev/ttyUSB0',
    baudrate = 9600,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    timeout = 1
    )

#print("[INFO] sampling THREADED frames from `picamera` module...")
vs = PiVideoStream().start()
time.sleep(2.0)
#fps = FPS().start()
 
# loop over some frames...this time using the threaded stream
while 1:
    # grab the frame from the threaded video stream and resize it
    # to have a maximum width of 400 pixels
    frame = vs.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lowerGreen, upperGreen)
    frame = cv2.bitwise_and(frame,frame, mask=mask)
    #cv2.imshow("Frame", frame)
    maskgrey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #ret,thresh = cv2.threshold(maskgrey,127,255,0)
    #kernel = numpy.ones((5,5),numpy.uint8)

    im2,contours, hierarchy = cv2.findContours(maskgrey,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    centres = []
    rects = []
    areas = []
    for i in contours:
        hull = cv2.convexHull(i)
        #epsilon = 0.1*cv2.arcLength(hull,True)
        #rects.append(cv2.approxPolyDP(hull,epsilon,True))
        areas.append(cv2.contourArea(hull))
        #print ('rect:',rects[-1])
        #print ('area:',areas[-1])
        if(areas[-1] > 100):
            moments = cv2.moments(hull)
            centres.append((int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])))
            #cv2.circle(frame, centres[-1], 3, (0, 0, 0), -1)
            #cv2.drawContours(frame,[rects[-1]],-1,(255,0,0),2)
            #print ('center:',centres[-1])
        else:
            areas.pop(-1)
            #rects.pop(-1)

    serialString = ''
    if(len(centres) != 0):
        for i in centres:
            serialString += str(i[0])
            serialString += ';'
	serialString+='.'
        print(serialString)
        ser.write(serialString.encode())


        
    # check to see if the frame should be displayed to our screen
    

    #if cv2.waitKey(1) & 0xFF == ord('q'):
    #        break
    # update the FPS counter
    #fps.update()
 
# stop the timer and display FPS information
#print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
#print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
 
# do a bit of cleanup
#fps.stop()
cv2.destroyAllWindows()
vs.stop()

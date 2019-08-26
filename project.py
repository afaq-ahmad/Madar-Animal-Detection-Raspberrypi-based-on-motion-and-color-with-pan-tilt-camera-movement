import numpy as np
import imutils
import cv2
import RPi.GPIO as GPIO
import time
from time import sleep
from picamera.array import PiRGBArray
from picamera import PiCamera

#--------------------------Servo1 Initialization----------------
panServo = 18
panAngle= 90
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(panServo, GPIO.OUT)
#--------------------------------------------------------------


#--------------------------Servo2 Initialization----------------
tiltServo = 16
tiltAngle = 90
GPIO.setup(tiltServo, GPIO.OUT)
#--------------------------------------------------------------


def setServoAngle(servo, angle):
    assert angle >=30 and angle <= 150
    pwm = GPIO.PWM(servo, 50)
    pwm.start(8)
    dutyCycle = angle / 18. + 3.
    pwm.ChangeDutyCycle(dutyCycle)
    sleep(0.3)
    pwm.stop()


# position servos to present object at center of the frame
def mapServoPosition (x_axis, y_axis):
    if (x_axis < 180):
        panAngle += 10
        if panAngle > 140:
            panAngle = 140
        setServoAngle(panServo, panAngle)
    if (x_axis > 220):
        panAngle -= 10
        if panAngle < 40:
            panAngle = 40
        setServoAngle(panServo, panAngle)
    if (y_axis < 92):
        tiltAngle += 10
        if tiltAngle > 140:
            tiltAngle = 140
        setServoAngle(tiltServo, tiltAngle)
    if (y_axis > 132):
        tiltAngle -= 10
        if tiltAngle < 40:
            tiltAngle = 40
        setServoAngle(tiltServo, tiltAngle)


#--------------------------Relay Initialization----------------
RelayPIN1 = 15
GPIO.setup(RelayPIN1, GPIO.OUT)
GPIO.output(RelayPIN1, GPIO.LOW)
relayOn = False
#--------------------------------------------------------------
# positioning Pan/Tilt servos at initial position
setServoAngle(panServo, panAngle)
setServoAngle(tiltServo, tiltAngle)


showvideo="yes"
#--------------------------Camera Setting---------------------
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = tuple([640, 480])
camera.framerate = 16
rawCapture = PiRGBArray(camera, size=tuple([640, 480]))
print("[INFO] warming up...")
time.sleep(2.5)
firstFrame = None
motionCounter = 0
minmotion = 4
#-------------------------------------------------------------

p=0
#--------------------------Animal color Range RGB--------------
lower = [65, 49, 28]# for color range of animal start to end
upper = [187, 159, 174]
lower = np.array(lower, dtype = "uint8")
upper = np.array(upper, dtype = "uint8")
#--------------------------------------------------------------

# capture frames from the camera
for f in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image and initialize
    # the timestamp and occupied/unoccupied text
    frame = f.array
    text = "No Motion"

    framecopy=frame.copy()
    # resize the frame, convert it to grayscale, and blur it
    frame = imutils.resize(frame, width=400)
    gray1 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray1, (17, 17), 0)
    
    # if the first frame is None, initialize it
    if firstFrame is None or p==200:
        p=0
        firstFrame = gray.copy()
        rawCapture.truncate(0)
        continue
    p=p+1
    
    # accumulate the weighted average between the current frame and
    # previous frames, then compute the difference between the current
    # frame and running average
    frameDelta = cv2.absdiff(gray, cv2.convertScaleAbs(firstFrame))

    # threshold the delta image, dilate the thresholded image to fill
    # in holes, then find contours on thresholded image
    thresh = cv2.threshold(frameDelta, 5, 255, cv2.THRESH_BINARY)[1]
    
    thresh = cv2.dilate(thresh, None, iterations=2)
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    # loop over the contours
    
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]
    
    for c in cnts:
        # if the contour is too small, ignore it
        if cv2.contourArea(c) < 600:
            continue
        # compute the bounding box for the contour, draw it on the frame,
        #print(cv2.contourArea(c))# and update the text
        (x, y, w, h) = cv2.boundingRect(c)
        ccc=framecopy[y: y+h, x: x+w].copy()

        # find the colors within the specified boundaries and apply
        # the mask
        mask = cv2.inRange(ccc, lower, upper)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 1))
        detectarea = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours = cv2.findContours(detectarea.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        contours = contours[0] if imutils.is_cv2() else contours[1]

        #mask = np.zeros(ccc.shape, dtype=np.uint8)
        for d in contours:
            if cv2.contourArea(d) < 3000:
                continue
            #print(cv2.contourArea(d))
            text1= "Detected"
            cv2.putText(frame, "Animal: {}".format(text1),(10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            x_axis=int(x+(w/2))
            y_axis=int(y+(h/2))
            cv2.circle(frame, (int(x+(w/2)),int(y+(h/2))), 5, (0, 0, 255), -1)
            
            motionCounter += 1
            
            if motionCounter > minmotion:
                minmotion = 0
                mapServoPosition (x_axis, y_axis)
                if not relayOn:
                    GPIO.output(RelayPIN1, GPIO.HIGH)
                    relayOn = True
                
                print("Servo Working")
        
        text = "Motion Detected"
    # draw the text and timestamp on the frame
    cv2.putText(frame, "Status: {}".format(text), (10, 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    # show the frame and record if the user presses a key
    # check to see if the frames should be displayed to screen
    if showvideo=="yes":
        # display the security feed
        cv2.imshow("Security Feed", frame)
        key = cv2.waitKey(1) & 0xFF

        # if the `q` key is pressed, break from the lop
        if key == ord("q"):
            break

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
    
p.stop()
GPIO.cleanup()
GPIO.output(RelayPIN1, GPIO.LOW)
import numpy as np
import argparse
import cv2
import time
from time import sleep
#import RPi.GPIO as GPIO
import wiringpi
from picamera.array import PiRGBArray
from picamera import PiCamera

# initialization
frame = None
roi_Pts = []
inputMode = False
#initialize Pin output on PI
io=wiringpi.GPIO(wiringpi.GPIO.WPI_MODE_SYS)
io.pinMode(24, io.OUTPUT)

camera = PiCamera()
#Pi is not able to calculate very fast, so set these parameters smaller
camera.resolution = (640, 480)
camera.framerate = 50
rawCapture = PiRGBArray(camera)
time.sleep(2)

def selectROI(event, x, y, flags, param):
	global frame, roi_Pts, inputMode
    #select the tracking target by clicking the left button on mouse four times
	if inputMode and event == cv2.EVENT_LBUTTONDOWN and len(roi_Pts) < 4:
		roi_Pts.append((x, y))
		cv2.circle(frame, (x, y), 4, (0, 255, 0), 2)
		cv2.imshow("frame", frame)

def main():
	ap = argparse.ArgumentParser()
	ap.add_argument("-v", "--video")
	args = vars(ap.parse_args())
    
	global frame, roi_Pts, inputMode

	cv2.namedWindow("frame")
	cv2.setMouseCallback("frame", selectROI)
	termination = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)
	roi_Box = None

    #LED Output test
    '''while True:
            io.digitalWrite(24, io.HIGH)
            sleep(2)
            io.digitalWrite(24, io.LOW)
            sleep(2)'''
                
	# reading frames
	while True:
		# current frame
		camera.capture(rawCapture,format='bgr', use_video_port=True)
        frame = rawCapture.array

		if roi_Box is not None:
            #convert color into HSV and do meanshift
			hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
			backProj = cv2.calcBackProject([hsv], [0], roi_Hist, [0, 180], 1)

            #camshift
			(r, roi_Box) = cv2.CamShift(backProj, roi_Box, termination)
			pts = np.int0(cv2.boxPoints(r))
            
			#the moving direction we need
			#if(cv2.minEnclosingCircle(pts)[0][0]>350):
				#print "right"
			#if(cv2.minEnclosingCircle(pts)[0][0]<290):
				#print "left"
			if(cv2.minEnclosingCircle(pts)[1]<230):
				#print "go"
                io.digitalWrite(24, io.HIGH)
			if(cv2.minEnclosingCircle(pts)[1]>=230):
				#print "stop"
                io.digitalWrite(24, io.LOW)

			cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

		# show the frame and record
		cv2.imshow("frame", frame)
		key = cv2.waitKey(1) & 0xFF
                rawCapture.truncate(0)

        #selection mode when clicking 'i'
		if key == ord("i") and len(roi_Pts) < 4:

			inputMode = True
			orig = frame.copy()

			while len(roi_Pts) < 4:
				cv2.imshow("frame", frame)
				cv2.waitKey(0)

			roi_Pts = np.array(roi_Pts)
			s = roi_Pts.sum(axis = 1)
			tl = roi_Pts[np.argmin(s)]
			br = roi_Pts[np.argmax(s)]
            
            #convert color
			roi_ = orig[tl[1]:br[1], tl[0]:br[0]]
			roi_ = cv2.cvtColor(roi_, cv2.COLOR_BGR2HSV)
            #compute HSV
			roi_Hist = cv2.calcHist([roi_], [0], None, [16], [0, 180])
			roi_Hist = cv2.normalize(roi_Hist, roi_Hist, 0, 255, cv2.NORM_MINMAX)
			roi_Box = (tl[0], tl[1], br[0], br[1])

		# stop running
		elif key == ord("q"):
			break

	camera.release()
	cv2.destroyAllWindows()

if __name__ == "__main__":
	main()
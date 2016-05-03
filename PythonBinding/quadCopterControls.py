from picamera import PiCamera
from fractions import Fraction
import picamera.array
import sys
import io
import numpy as np
import serial
import cv2
import time
import RPi.GPIO as GPIO
import DroneUtils


# Use Raspberry Pi Board pin numbers
GPIO.setmode(GPIO.BOARD)

GPIO.setup(7, GPIO.OUT)

#Deinfe the Arduino pin... to find pin, type the follwing with Arduino unplugged
# from the Pi. Then plug the Arduino into the Pi and type the same command,
# the newly listed device is the Ardunio
#       $ ls /dev/tty*
ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 0);

#GPIO.setup(26, GPIO.OUT);


def sendCmd(str):
#---------------------------INIT CONTORLS---------------------------------------
	#Arm the drone
	if cmd == 'r':
          #      print("This line is commented out so the software devs don't accidently kill anyone, soooo you should fix that")
		ser.write('ia0000');
	#Disarm the drone
	if cmd == 't':
		ser.write('ik0000');
	#Set alt
	if cmd == 'h':
		ser.write('is0000');
	#Run test
	if cmd == 'g':
        #         print("This line is commented out so the software devs don't accidently kill anyone, soooo you should fix that")
		ser.write('it0000');
#---------------------------DRONE MOVEMENT CONTROLS------------------------------
	#Move the drone up
	if cmd == 'u':
		ser.write('mu2012'); #THRMAX
	#Move the drone down
	if cmd == 'o':
		ser.write('md1029'); #THRMIN
	#Move the drone forward
	if cmd == 'i':
		ser.write('mf2300'); #PIT60
	#Move the drone right
	if cmd == 'm':
		ser.write('mr1600'); #ROLL60		
	#Move the drone backwards
	if cmd == 'k':
		ser.write('mb1400'); #PIT40
	#Move the drone left
	if cmd == 'n':
		ser.write('ml1400'); #ROLL40	
	#Turn the drone left
	if cmd == 'j':
		ser.write('tl1428'); #YAW40
	#Turn the drone right (YAW60)
	if cmd == 'l':
		ser.write('tr1628'); #YAW60
#--------------------------SERVO CONTROLS----------------------------------------
        #Move the horizontal servo to the left.. hard coded for 5 degrees currently
	if cmd == 'a':
                ser.write('sl0005');
        #Move the vertical servo down.. hard coded for 5 degrees currently
	if cmd == 's':
                ser.write('sd0005');
        #Move the horizontal servo to the right.. hard coded for 5 degrees currently
	if cmd == 'd':
                ser.write('sr0005');
        #Move the vertical servo up.. harde coded for 5 degrees currently
	if cmd == 'w':
                ser.write('su0005');
        #Reset the servos positions
	if cmd == 'x':
                ser.write('sx0000');
#-------------------------LASER CONTROLS-------------------------------------------
        #Toggle the laser on/off
	if cmd == 'f':                
		GPIO.output(7, 1);
#		ser.write('l00000');
	if cmd == 'p':
		GPIO.output(7, 0);
#		ser.write('100000');
#------------------------QUIT CONTROLS---------------------------------------------       
	#Get the distance to the laser (alt)
	if cmd == 'z':
		#Set Servos to 90:90
		ser.write('sx0000');
		#Move vertical servo down 35Deg
		ser.write('sd0090');
		#Set MID values for motors.
		ser.write('it0000');
		#wait for the drone to be above 5 feet.
		d = getCurDist();
		safety = 0;
		#while (d < 4) or (safety > 4):
		#	d = getCurDist();
		#	safety += 1;
		#Disarm the Drone
		ser.write('ik0000');
                print d
	#Get the distance to the laser (far)
	if cmd == 'v':
		ser.write('sx0000');
		d = getCurDist();
		print d
        if cmd == 'q':
                getLaserPoint()
	#Calibrate lzr
	if cmd == 'c':
		print "Enter dist to wall:"
                wallDist = raw_input("")		
		print "Enter y-coord of lzr:"
                lzrY = raw_input("")
	        pt = [float(wallDist), float(lzrY)]
		list = []	
		list.append(pt)
		DroneUtils.getLaserConstants(list)


	#Quit the program
	if cmd == 'quit':
		ser.close();
                sys.exit(0);
        else :
                return;

# This method captures and returns two images.. One with the laser on, one with it off.
#Returns an array of data for two images, which can be written as an image file.
def toggleLaserPictures():
	#Setup camera and stuffs
	camera = PiCamera();
	camera.brightness = 30
	camera.resolution = (2592, 1944)
	camera.awb_mode = 'incandescent'
	stream = io.BytesIO()
	GPIO.output(7,1)
	time.sleep(1)
	camera.capture(stream, format='png')
	img1Data = np.fromstring(stream.getvalue(), dtype=np.uint8)
	stream.close()
	GPIO.output(7,0)
	time.sleep(1)
	stream = io.BytesIO()
	camera.capture(stream, format='png')
	img2Data = np.fromstring(stream.getvalue(), dtype=np.uint8)
	stream.close()
	images = [img1Data, img2Data]
	camera.close()
	return images



def getLaserPoint():
        imgs = toggleLaserPictures()
        img1 = cv2.imdecode(imgs[0],1)
        cv2.imwrite("DroneLASAR.png",img1)
        lst = DroneUtils.getLaserDistOneImgLoc(img1.tostring('c'), img1.shape[1], img1.shape[0])
        print(lst[0],lst[1])
        
# This method uses the laser pointer to determine the distance to the laser. Dist
# is returned in feet.
def getCurDist():
	imgs = toggleLaserPictures();
	img1 = cv2.imdecode(imgs[0], 1);
	cv2.imwrite("RangePic.png", img1);
	dist = DroneUtils.getLaserDistOneImg(img1.tostring('c'), img1.shape[1], img1.shape[0],241.978,1287);
	return dist;


print "Now controlling the QuadCopter"
print "--------------------------------------------------------------------"
print "To ARM the drone: r "
print "To DISARM the drone: t"
print "To SET ALTITUDE: h"
print "To run TEST: g"
print " "
print "To move the Servos:    a, s, d, w"
print "Reset the servos to 90,90: x "
print " "
print "To turn the Laser on: f"
print "To turn the laser off: p"
print "To get alt. dist: z"
print " "
print "To move the drone:"
print " i = forward"
print " j = turn left"
print " l = turn right"
print " k = backward"
print " n = strafe left"
print " m = strafe right"
print " u = UP"
print " o = DOWN"
print "--------------------------------------------------------------------"    
print "Enter 'quit' to quit the program."

#Get user input and listen for response from drone
while 1 :
        res = ser.readline();
	cmd = raw_input("");
        if cmd:
                sendCmd(cmd);
	if res:
		print res;
	

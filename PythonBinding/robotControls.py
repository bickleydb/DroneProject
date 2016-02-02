from picamera import PiCamera
import serial
import time
import numpy as np
import cv2
import DroneUtils

ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 0);

def sendCmd(str):
	#Move Forward
	if str == 'w':
		ser.write('mf0100');
	#Move Backward
	if str == 's':
		ser.write('mb0100');
	#Turn Left
	if str == 'a':
		ser.write('ml0090');
	#Turn Right
	if str == 'd':
	 	ser.write('mr0090');
	#Laser Toggle
	if str == 'f':
		ser.write('l00000');
	#Move Servo Up 5 degrees
	if str == 'i':
		ser.write('su0005');
	#Move Servo Right 5 degrees
	if str == 'l':
		ser.write('sr0005');
	#Move Servo Down 5 degrees
	if str == 'k':
		ser.write('sd0005');
	#Move Servo Left 5 degrees
	if str == 'j':
		ser.write('sl0005');
	#Get lasers distance
	if str == 'r':
		d = getCurDist();
		print d;
	#Quit Program
	if str == 'quit':
		ser.close();
		sys.exit();
	else:
		return;

def getCurDist():
	camera = PiCamera();
	camera.resolution = (2592,1944);
	time.sleep(3);
	camera.capture('img1.png');
	time.sleep(5);
	ser.write('l00000');
	camera.capture('img2.png');
	time.sleep(5);
	ser.write('l00000');

	image2 = cv2.imread("img2.png");
	image = cv2.imread("img1.png");

	dist = DroneUtils.getLaserLocation(image.tostring('c'), image2.tostring('c'), image.shape[1], image.shape[0]);
	cv2.destroyAllWindows();
	return dist;

print "Now controlling the Ground Robot"
print "------------------------------------------------------------"
print "To move teh drone:"
print "  w = forward"
print "  s = backward"
print "  a = left"
print "  d = right"
print " "
print "To toggle the laser: f"
print " "
print "To move the servos:"
print "  j = turn left"
print "  k = look down"
print "  l = turn right"
print "  i = look up"
print " "
print "To get a distance read:"
print " r = Get distance to laser"
print " "
print "------------------------------------------------------------"
print "Type 'quit' to exit the program"


while 1:
	cmd = raw_input("");
	if cmd:
		sendCmd(cmd);

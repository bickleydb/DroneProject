from picamera import PiCamera
import picamera.array
import sys
import serial
import io
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
		ser.write('su0001');
	#Move Servo Right 5 degrees
	if str == 'l':
		ser.write('sr0001');
	#Move Servo Down 5 degrees
	if str == 'k':
		ser.write('sd0001');
	#Move Servo Left 5 degrees
	if str == 'j':
		ser.write('sl0001');
	#Reset servos
	if str == 'x':
		ser.write('sx0000');
        if str == 'y':
                takePictures()
        #Get lasers distance
	if str == 'r':
		d = getCurDist();
		print d;
	#Get paper distance
	if str == 'b':
		d = getPaperDist();
		print d;
	#Get lasers positon
	if str == 't':
		p = getLaserPos();
		print p;
	#Find boxes
        if str == 'p':
                s = findBox();
		print s;
        if str == 'z':
                getBoxCorners()
	#Point laser to right most box
	if str == 'e':
		pointToRightBox();
		print "I found the right box!"
	#Point laser to left most box
	if str == 'q':
		pointToLeftBox();
		print "I found the left box!"
	#Testrun
	if str == 'c':
		automateCalibration();
		print "Done"
	#Test motor dist
	if str == 'v':
		testMotor();
	#Quit Program
	if str == 'quit':
		ser.close();
		sys.exit(0);	
	else:
                return;

def pointToLeftBox():
	boxes = findBox();
	laser = getLaserPos();
	centerLaser(boxes[0], laser, 0);

def pointToRightBox():
	boxes = findBox();
	laser = getLaserPos();
	centerLaser(boxes[1], laser, 1);

def centerLaser(boxes, laser, direction):
	boxMinX = boxes[0];
	len = boxes[2];
	boxMaxX = (boxMinX + len);
	boxMinY = boxes[1];
	hgt = boxes[3];
	boxMaxY = (boxMinY + hgt);
	laserX = laser[0];
	laserY = laser[1];

	if laserX > boxMaxX :
		ser.write('sl0010');
		if direction == 0 :
			pointToLeftBox();
		if direction == 1 :
			pointToRightBox();
	if laserX < boxMinX :
		ser.write('sr0010');
		if direction == 0 :
			pointToLeftBox();
		if direction == 1 :
			pointToRightBox();
	if laserY > boxMaxY :
		ser.write('su0010');
		if direction == 0 :
			pointToLeftBox();
		if direction == 1 :
			pointToRightBox();
	if laserY < boxMinY :
		ser.write('sd0010');
		if direction == 0 :
			pointToLeftBox();
		if direction == 1 :
			pointToRightBox();
	else:
		return;

# This method captures and returns two images, taken simultaneously from the same
# position. One image is taken with the laser, the other without.
# Returns an array of data for two images, ready to be written to an image file.
# @return  images[]
#	images[0] = image taken with laser on
#	images[1] = image taken with laser off
def toggleLaserPictures():
	#Setup camera an stuffs
	camera = PiCamera();
	camera.resolution = (2592,1944);
	time.sleep(2);
	stream = io.BytesIO();	
	
	#Turn on laser and get first image, captured into a stream
	ser.write('l00000');
	time.sleep(2);
	camera.capture(stream, format='jpeg');
	img1Data = np.fromstring(stream.getvalue(), dtype=np.uint8);	
		
	stream.close();
	#Turn off laser and get second image, then read image into a new stream
	ser.write('l00000');
	time.sleep(2);
	stream = io.BytesIO();
	camera.capture(stream, format='jpeg');
	img2Data = np.fromstring(stream.getvalue(), dtype=np.uint8);

	stream.close();	
	images = [img1Data, img2Data];	
	camera.close();
	return 	images;

def takePictures():
        camera = PiCamera()
        camera.resolution = (2592,1944)
        stream = io.BytesIO()
        camera.capture(stream,format='jpeg')
        data = np.fromstring(stream.getvalue(),dtype=np.uint8)
        image = cv2.imdecode(data,1)
        cv2.imwrite("BoxesAtHighRez.png",image);
        camera.close()
        
def findBox():
        camera = PiCamera()
        camera.resolution = (2592,1944)
        stream= io.BytesIO()
        camera.capture(stream,format='jpeg')
        data = np.fromstring(stream.getvalue(),dtype=np.uint8)
        image = cv2.imdecode(data,1)
        stuff = DroneUtils.getAllBoxes(image.tostring('c'), image.shape[1],image.shape[0])

        for box in stuff:
                cv2.rectangle(image,(box[0],box[1]),(box[0]+box[2],box[1]+box[3]),(255,255,0),5)
        cv2.imwrite('FindBox.png',image)
        camera.close()
        cv2.destroyAllWindows()
        return stuff;


def getBoxCorners():
        camera = PiCamera();
        camera.resolution = (2592,1944)
        stream = io.BytesIO()
        camera.capture(stream,format='png')
        data = np.fromstring(stream.getvalue(),dtype=np.uint8)
        image = cv2.imdecode(data,1)
        cv2.imshow("Image",image)
        cv2.waitKey(10)
        lst = DroneUtils.getBoxCorner(image.tostring('c'),image.shape[1],image.shape[0])
        for i in lst:
                for x in range(len(i)/2):
                        print(str(i[2*x]) + " " + str(i[2*x+1]))
        cv2.imwrite("output.png",image)
        camera.close()
        cv2.destroyAllWindows()
        print(lst)


def getCurDist():
	images = toggleLaserPictures();
	img1 = cv2.imdecode(images[0], 1);
	img2 = cv2.imdecode(images[1], 1);
	cv2.imwrite("ta.jpeg", img1);
	cv2.imwrite("tb.jpeg", img2);
	dist = DroneUtils.getLaserDist(img2.tostring('c'), img1.tostring('c'), img1.shape[1], img1.shape[0]);
	return dist;

def getPaperDist():
	images = toggleLaserPictures();
	img = cv2.imdecode(images[1], 1);
	cv2.imwrite("t1a.jpeg", img);
	dist = DroneUtils.getPaperDistByCorner(img.tostring('c'), img.shape[1], img.shape[0]);
	return dist;

def getLaserPos():
	images = toggleLaserPictures();
	img1 = cv2.imdecode(images[0], 1);
	img2 = cv2.imdecode(images[1], 1);
	cv2.imwrite("t1.jpeg", img1);
	cv2.imwrite("t2.jpeg", img2);
	pos = DroneUtils.getLaserLocation(img1.tostring('c'), img2.tostring('c'), img1.shape[1], img1.shape[0]);
	return pos;	

def automateCalibration():
	distToPaper = getPaperDist();
	print (distToPaper);
	while (distToPaper > 5):
		images = toggleLaserPictures();
		img1 = cv2.imdecode(images[0], 1);
		img2 = cv2.imdecode(images[1], 1);
		cv2.imwrite('%02d_withLaser.jpeg' % distToPaper, img1);
		cv2.imwrite('%02d_withOut.jpeg' % distToPaper, img2);
		ser.write('mf0100');
		distToPaper = getPaperDist();
		print(distToPaper);
	return;

def testMotor():
	distToPaper = getPaperDist();
	print ("Initial Dist: ", distToPaper);
	ser.write('mf0100');
	time.sleep(2);
	distToPaper = getPaperDist();
	print ("Dist. after: ", distToPaper);
	return;

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
print "  x = reset servo positions"
print " "
print "To get a distance read:"
print " r = Get distance to laser"
print " b = Get distance alt. "
print " t = Get position of laser"
print " p = Find boxes"
print " q = Point laser to left most box"
print " e = Point laser to right most box"
print " "
print "------------------------------------------------------------"
print "Type 'quit' to exit the program"


while 1:
	cmd = raw_input("");
	if cmd:
		sendCmd(cmd);

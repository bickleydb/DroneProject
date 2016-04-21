from picamera import PiCamera
from fractions import Fraction
import picamera.array
import sys
import serial
import io
import time
import numpy as np
import math
import cv2
import DroneUtils

ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 0)

# CLASS DEF FOR C++ SWITCH FUNCTION
class switch(object):
    def __init__(self, value):
        self.value = value
        self.fall = False

    def __iter__(self):
        """Return the match method once, then stop"""
        yield self.match
        raise StopIteration
    
    def match(self, *args):
        """Indicate whether or not to enter a case suite"""
        if self.fall or not args:
            return True
        elif self.value in args: # changed for v1.5, see below
            self.fall = True
            return True
        else:
            return False

		


def waitForDebug():
    ser.write('l00000')
    print("Waiting for stuff")
    raw_input()
    ser.write('l00000')
#**********************************************************************************************
# This method creates a command string to be sent to the Arduino. This method ensures that any
# commands sent to the Arduino are always 6 characters long by adding the needed digits. 
#
# Note* this method cannot be used for the laser toggle command on the robot.. those cmds should
# always be hard coded. 
# Parameters:
#		first - the first character of the desired command
#		second - the second character of the desired command
#		mod - the value of the modifer for the command
# @ return  ret - string representation of the command
# Example: createCommand("m", "f", "5")  -> returns "mf0005" (Moves forward 5 ft)
# Example: createCommand("s", "r", "45") -> returns "sr0045" (Turns servo right 45 degrees)
def createCommand(first, second, mod):
	modValue = int(round(mod))
	includes = [first, second, str(modValue)]
	cmd = ["*","*","0","0","0","0"]
	# for every character included in the call..
	for s in includes:
		# check to see where it belongs int the command
		for case in switch(s):
			if case("m", "s"):
				cmd[0] = s[0]
				break
			if case("f", "b", "r", "l", "u", "d", "x"):
				cmd[1] = s[0]
				break
			if case():
				number = int(s)
				numDigits = numLen(number)
				nums = [c for c in s]
				if (numDigits == 1):
					cmd[5] = nums[0]
				if (numDigits == 2):
					cmd[5] = nums[1]
					cmd[4] = nums[0]
				if (numDigits == 3):
					cmd[5] = nums[2]
					cmd[4] = nums[1]
					cmd[3] = nums[0]
				if (numDigits == 4):
					cmd[5] = nums[3]
					cmd[4] = nums[2]
					cmd[3] = nums[1]
					cmd[2] = nums[0]
				break
	# Return the command
	ret = ''.join(cmd)
	return ret

#Helper method for number of digits
def numLen(n):
	return len(str(abs(n)))

# Takes two images, with and without the laser
def toggleLaserPictures():
		camera = PiCamera()
		camera.brightness = 30
		camera.resolution = (2592, 1944)
		camera.awb_mode = 'incandescent'
		stream = io.BytesIO()
		ser.write('l00000')
		time.sleep(2)
		camera.capture(stream, format='png')
		img1Data = np.fromstring(stream.getvalue(), dtype=np.uint8)
		stream.close()
		ser.write('l00000')
		camera.close()
                images = [img1Data]
		return images
	
# Returns the distance from the robot to the laser pointer in feet
def getCurDist():
	images = toggleLaserPictures()
	img1 = cv2.imdecode(images[0], 1)
	dist = DroneUtils.getLaserDistOneImg(img1.tostring('c'), img1.shape[1], img1.shape[0], 255.405, 922.538)
	return dist
	
def findBox():
	camera = PiCamera()
	camera.resolution = (2592,1944)
        camera.brightness = 75
	stream = io.BytesIO()
	camera.capture(stream, format='png')
	data = np.fromstring(stream.getvalue(), dtype=np.uint8)
	image = cv2.imdecode(data,1)
	stuff = DroneUtils.getAllBoxes(image.tostring('c'), image.shape[1], image.shape[0])
	for box in stuff:
		cv2.rectangle(image, (box[0], box[1]), (box[0]+box[2], box[1]+box[3]), (255,255,0),5)
	cv2.imwrite('FindaBox.png', image)
	camera.close()
	cv2.destroyAllWindows()
	return stuff
	
# Gets the lasers (x,y) position
def getLaserPos():
	images = toggleLaserPictures();
	img1 = cv2.imdecode(images[0], 1)
	cv2.imwrite('testLaser.png', img1)
	pos = DroneUtils.getLaserDistOneImgLoc(img1.tostring('c'), img1.shape[1], img1.shape[0])
	return pos
	
#******************************************************************************************
# This method attempts to navigate the robot around an obstacle. The robot will get the far
# distances to the left and right walls, which ever distance is greater is the direction
# the robot will try to go around the obstacle by travelling 1 ft at a time. After going 1
# foot, the robot will then get the distance in the direction of the box, if it is greater
# than 1 foot, the robot has gone past the obstacle and proceed past it, otherwise it keeps
# going. If it runs out of distance without going around the obstacle, the robot will try
# the other direction.
def goAroundBox():
	print "Attempting to navigate around the box.."
	# Get the left dist
	ser.write('sl0080')
	leftDist = getCurDist()
	ser.write('sx0000')
	time.sleep(1)
	# Get the right dist
	ser.write('sr0080')
	rightDist = getCurDist()
	ser.write('sx0000')
	
	print ("Distance to left-side wall: %d" % leftDist)
	print ("Distance to right-side wall: %d" % rightDist)
	time.sleep(1)
	passedObj = False
	farDist = -1
	# Determine which direction to go around the obstacle
	if (leftDist > rightDist):
		print "Trying to pass on the left side.."
		# Turn robot left
		ser.write('ml0090')
		time.sleep(2)
		# Make sure robot has room to move on the left
		if (leftDist > 2):
			hasRoom = True
			roomRemaining = leftDist
		else :
			hasRoom = False
			print "No room to move left!"
		# While the robot has not passed the obstacle and still has room to move..
		while (not(passedObj) and hasRoom):
			# Make sure the robot still has room to move
			if (roomRemaining <= 2):
				hasRoom = False
			# Drive 1 foot
			ser.write('mf0001')
			time.sleep(3)
			# Get the far distance, if > 1, the robot has passed the obstacle
			ser.write('sr0080')
			farDist = getCurDist()
			ser.write('sx0000')
			time.sleep(1)
			if (farDist > 2):
				print "Cleared obstacle, driving past it now.."
				# Turn the robot and drive past the obstacle
				ser.write('mr0090')
				time.sleep(2)
				ser.write('mf0003')
				time.sleep(5)
				passedObj = True
			print "Haven't cleared obstacle yet, advancing.."
			# Otherwise decrement the room remaining, an try again
			roomRemaining -= 1
		if (passedObj):
			print "Passed obstacle."
			return
		else :
			# Turn robot back, and try again (going the other way)
			ser.write('mr0090')
			time.sleep(2)
			print "Unable to pass obstacle on the left, trying again."
			goAroundBox()
	else :
		print "Trying to pass on the right side.."
		# Turn robot right
		ser.write('mr0090')
		time.sleep(2)
		# Make sure the robot has room to move on the right
		if (rightDist > 2):
			hasRoom = True
			roomRemaining = rightDist
		else :
			hasRoom = False
			print "No room to move right!"
		while (not(passedObj) and hasRoom):
			if (roomRemaining <= 2):
				hasRoom = False
			ser.write('mf0001')
			time.sleep(3)
			ser.write('sr0080')
			farDist = getCurDist()
			ser.write('sx0000')
			time.sleep(1)
			if (farDist > 2):
				print "Cleared obstacle, driving past it now.."
				ser.write('mr0090')
				time.sleep(2)
				ser.write('mf0003')
				time.sleep(5)
				passedObj = True
			print "Haven't cleared obstacle yet, advancing.."
			roomRemaining -= 1
		if (passedObj):
			print "Passed obstacle."
			return
		else :
			ser.write('ml0090')
			time.sleep(2)
			print "Unable to pass obstacle on the right, trying again."
			goAroundBox()
		
#******************************************************************************************
# This method attempts to center the robot with a box discovered in the room with it. The
# parameters are:
#	dist = the distance from the robot to the box
#	offset = the degree (-90 - 90) in which the box is located from the robot
# The robot will turn 90 degrees left or right towards the box, and travel a determined dist
# x (x = dist * cos(offset)) in that direction. The robot will then turn 90 degrees back so
# that it is now facing the box. The robot will then drive towards the box, and stop 1 ft
# from it.
def centerWithBox(dist, offset):
	print ("Centering robot with box..")
	# Reset the servos
	ser.write('sx0000')
	time.sleep(1)
	# Determine if robot needs to turn left or right
	if (offset < 0):
		ser.write('ml0090')
		time.sleep(3)
	else:
		ser.write('mr0090')
		time.sleep(3)
        waitForDebug()
	# Determine the distance 'x' the robot is to drive, and do so
	x = dist * math.sin(offset)
	x = abs(x)

	print ("Moving robot %d ft" % x)
        print(dist)
        waitForDebug()
	ser.write(createCommand("m", "f", x))
	time.sleep(5)
	# Turn the robot back to face the box
	if (offset < 0):
		ser.write('mr0090')
	else:
		ser.write('ml0090')
	time.sleep(2)
	# Get the distance to the box
	newDist = getCurDist()
	if ((newDist-2) > 0):
		travelDist = newDist - 2
		print ("Moving robot %d ft towards box.." % travelDist)
	
		# Drive to the box
		ser.write(createCommand("m", "f", travelDist))
		time.sleep(5)
		return True
	else:
		print("Unable to advance towards the box.. not enough room.")
		return False



def passBox(dist, offset):
	ser.write('sx0000')
	time.sleep(1)
	if (offset<0):
		ser.write('ml0020')
	else:
		ser.write('mr0020')
	time.sleep(3)
	ser.write(createCommand("m", "f", (dist/2)))
	time.sleep(5)
	time.sleep(1)
	if (offset<0):
		ser.write('mr0020')
	else:
		ser.write('ml0020')
	time.sleep(3)
	newDist = getCurDist()
	if (newDist >(dist/2)):
		distToTravel = (dist/2)+4
		print ("Moving robot %d " % distToTravel)
		ser.write(createCommand("m", "f", distToTravel))
		time.sleep(5)
	else:
		passBox(newDist, offset)
	return True

#*******************************************************************************************
# This method is the 'brains' of the robot. It is responsible for handling the robots A.I.
# so that it is able to navigate a room with obstacles and find the target object.
def itsAlive():
	print "STARTING AUTONOMOUS NAVIGATION"
	#Robot searches for boxes in an image
	print "Looking for boxes.."
	possibleBoxes = findBox()
	# If no boxes were found.
	if not possibleBoxes:
		#Do something
		print "Uh-oh, no boxes were found.. "
		
	# Otherwise at least one box was found
	else:
		numBoxes = len(possibleBoxes)
		print ("Found %d boxes!" % numBoxes)
		
		# boolean for terminating box scanning for loop
		keepLooking = True
		# Robots distance to box
		dist = -1
		while (keepLooking):
			# For each box found, find its distance by pointing the laser near the center of the box
			# Really only uses first box found
			for i in range(numBoxes):
				print ("Centering laser on box %d .." % (i+1))
				curBox = possibleBoxes[i]
				# Get the current box's bounds
				boxMinX = curBox[0]
				boxMinY = curBox[1]
				boxMaxX = boxMinX + curBox[2]
				boxMaxY = boxMinY + curBox[3]
				boxMeanX = (boxMinX + boxMaxX)/2
				boxMeanY = (boxMinY + boxMaxY)/2
				print ("Center of box x-cord: %d" % boxMeanX)
				print ("Center of box y-cord: %d" % boxMeanY)
			
				# Get the lasers current position
				laserPos = getLaserPos()
				laserX = laserPos[0]
				laserY = laserPos[1]
				print ("Laser x: %d" % laserX)
				print ("Laser y: %d" % laserY)
				
				# Determine how far off the laser is from the box, and center it.
			
				# Determine if the laser is right or left of the box
				if (laserX < boxMeanX):
					print "Laser is left of the box"
					xDir = "r"
				if (laserX > boxMeanX):
					print "Laser is right of the box"
					xDir = "l"
				# Determine if the laser is up or down of the box
				if (laserY < boxMeanY):
					print "Laser is also above the box"
					yDir = "d"
				if (laserY > boxMeanY):
					print "Laser is also below the box"
					yDir = "u"

                                
				x = abs(boxMeanX - laserX)
				y = abs(boxMeanY - laserY)
				horOffset = int(round(((x*1.2)/73)))
				verOffset =  int(round(((y*1.8)/73)))

                                print(horOffset)
                                print(verOffset)
                             
				# Center laser
				horCmd = createCommand("s", xDir, horOffset)
				verCmd = createCommand("s", yDir, verOffset)
				print ("Moving horizontal servos %d degrees via " % horOffset)
				print horCmd
				print ("Moving vertical servos %d degrees via " % verOffset)
				print verCmd
				ser.write(horCmd)
				time.sleep(1)
				ser.write(verCmd)
				dist = getCurDist()
				print ("Box is %d ft away" % dist)
				if (xDir == "l"):
					horOffset = horOffset*-1			
				boxApproached = passBox(dist, horOffset)
		

				#if (xDir == "l"):
				#	horOffset = horOffset*-1			# Pass in the distance to the box, and the degree the box is offset by
                                #print(horOffset)
				#boxApproached = centerWithBox(dist, horOffset)
			
				if (boxApproached):
					keepLooking = False
				else:
					keepLooking = True
				break
				
		# Navigate around the obstacle
		#goAroundBox()
		# Look for the target object
		print "Searching for target.."
		print "Done!"
		#targetImg = searchForTarget()
		#if not targetImg:
		#	print "Target not found, starting over.."
		#	itsAlive()
		#else :
		#	print "Target found! Done!"
		#	cv2.imwrite("Target.png", targetImg)
			
			
		
		
		
		

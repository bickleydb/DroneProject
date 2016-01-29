import serial
import time

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
		ser.write('ia0000');
	#Disarm the drone
	if cmd == 't':
		ser.write('ik0000');
	#Set alt
	if cmd == 'h':
		ser.write('is0000');
	#Run test
	if cmd == 'g':
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
		#GPIO.output(26, True);
		ser.write('l00000');
#------------------------QUIT CONTROLS---------------------------------------------       
	#Quit the program
	if cmd == 'quit':
		ser.close();
                sys.exit(0);
        else :
                return;


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
print "To toggle the Laser: f"
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
	

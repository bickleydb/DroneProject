import serial
import time

ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 0);

def sendCmd(str):
	#Move Forward
	if str == 'w':
		ser.write('mf0000');
	#Move Backward
	if str == 's':
		ser.write('mb0000');
	#Turn Left
	if str == 'a':
		ser.write('ml0000');
	#Turn Right
	if str == 'd':
	 	ser.write('mr0000');
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
	#Quit Program
	if str == 'quit':
		ser.close();
		sys.exit();
	else:
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
print " "
print "------------------------------------------------------------"
print "Type 'quit' to exit the program"


while 1:
	cmd = raw_input("");
	if cmd:
		sendCmd(cmd);

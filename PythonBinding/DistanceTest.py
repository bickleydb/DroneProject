#!usr/bin/python
import cv2
import numpy as np
import serial
import time
import io
import DroneUtils

ser = serial.Serial('/dev/ttyACM0')

img2 = cv2.imread("laser10b.png")
img = cv2.imread("laser10bwithout.png")
dist = DroneUtils.getLaserDist(img.tostring('c'), img2.tostring('c'), 1080, 1920)
print "%d" % dist

#!usr/bin/python
import numpy as np
import time
import cv2
import DroneUtils


image = cv2.imread("img2.png");
image2 = cv2.imread("img1.png");
print("For the images 'laser10b.png' and 'laser10bwithout.png', which were taken 10 feet out: ");
#Corner methods
distByCorner = DroneUtils.getPaperDistByCorner(image.tostring('c'),image.shape[1],image.shape[0]);
cornerList = DroneUtils.getPaperByCorner(image.tostring('c'), image.shape[1], image.shape[0]);
print("The distance by corner was: %d" % distByCorner);
print("It's corner points are:");
for pt in cornerList:
	print(pt);
#Contour methods
distByContour = DroneUtils.getPaperDistContour(image.tostring('c'), image.shape[1], image.shape[0]);
contourList = DroneUtils.paperContour(image.tostring('c'), image.shape[1], image.shape[0]);
print("The distance by contour was: %d" % distByContour);
print("It's contour points are:");
for pts in contourList:
	print(pts);
#Laser methods
laserDist = DroneUtils.getLaserDist(image.tostring('c'), image2.tostring('c'), image.shape[1], image.shape[0]);
laserList = DroneUtils.getLaserLocation(image.tostring('c'), image2.tostring('c'), image.shape[1], image.shape[0]);
print("The laser's distance was: %d" % laserDist);
print("It's points are:");
for pnt in laserList:
	print(pnt);


cv2.destroyAllWindows()

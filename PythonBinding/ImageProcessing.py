#!usr/bin/python
import numpy as np
import time
import cv2
import DroneUtils


image = cv2.imread("15Feet.png")
image2 = cv2.imread("15FeetLaser.png");
lst = DroneUtils.getLaserDist(image.tostring('c'),image2.tostring('c'),image.shape[1],image.shape[0]);
print(lst)  


cv2.destroyAllWindows()
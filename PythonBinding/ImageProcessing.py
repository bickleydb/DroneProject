#!usr/bin/python
import numpy as np
import time
import cv2
import cv
import DroneUtils


image = cv2.imread("15Feet.png")

print(DroneUtils.getPaperDist(image.tostring('C'),image.shape[1],image.shape[0]));
  


cv2.destroyAllWindows()
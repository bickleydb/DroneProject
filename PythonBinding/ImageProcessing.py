#!usr/bin/python
import numpy as np
import time
import cv2
import DroneUtils


image = cv2.imread("15Feet.png")

print(DroneUtils.paperContour(image.tostring('C'),image.shape[1],image.shape[0]));
  


cv2.destroyAllWindows()
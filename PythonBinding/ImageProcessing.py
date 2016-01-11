#!usr/bin/python
import numpy as np
import time
import cv2
import DroneUtils


image = cv2.imread("15Feet.png")
lst = DroneUtils.getPaperDistByCorner(image.tostring('c'),image.shape[1],image.shape[0]);
print(lst)  


cv2.destroyAllWindows()
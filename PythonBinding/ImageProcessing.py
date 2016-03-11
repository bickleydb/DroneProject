#!usr/bin/python
import numpy as np
import time
import cv2
import DroneUtils

image = cv2.imread("17foot_BothBoxes.png");
print(DroneUtils.getBoxCorner(image.tostring('c'),image.shape[1],image.shape[0]));

#cv2.resize(image,(1000,1000))
#cv2.imshow("",image);
#cv2.waitKey(1)

cv2.destroyAllWindows()

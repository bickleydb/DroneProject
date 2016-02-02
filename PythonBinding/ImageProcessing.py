#!usr/bin/python
import numpy as np
import time
import cv2
import DroneUtils

cap = cv2.VideoCapture(0)
while(True):
    _,image = cap.read()
    lst = DroneUtils.getAllBoxes(image.tostring('c'),image.shape[1],image.shape[0]);
    for box in lst:
        cv2.rectangle(image,(box[0],box[1]),(box[0]+box[2],box[1]+box[3]),(255,0,0),5)
        print(lst)  
    cv2.resize(image,(1000,1000))
    cv2.imshow("",image);
    cv2.waitKey(1)

cv2.destroyAllWindows()

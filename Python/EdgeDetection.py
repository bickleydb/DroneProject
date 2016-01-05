#! usr/bin/python

import cv2
import numpy as no

cap = cv2.VideoCapture(0)
#cap.set(3,1000)
#cap.set(4,1000)
while True:
    _,frame = cap.read()
    #frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    #frame = cv2.GaussianBlur(frame,(9,9),0,0)
    frame = cv2.Canny(frame,100,200)
    frame = cv2.resize(frame,(1920,1080),interpolation=cv2.INTER_LINEAR)
    cv2.imshow('Edge Detection',frame)
    cv2.waitKey(10)

#!usr/bin/python
import numpy as np
import time
import cv2
import cv
import ImgProc

cap = cv2.VideoCapture(0)

while True:
      _,frame = cap.read()
      cv2.imshow('Diff',frame)
      t = cv.fromarray(frame)
      ImgProc.testing(t)
      if cv2.waitKey(1) & 0xFF == ord('q'):
      	 break

cap.release()
cv2.destroyAllWindows()
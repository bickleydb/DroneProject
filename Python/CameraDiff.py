#!usr/bin/python
import time
import cv2
import numpy as np

def getFrame():
    cap = cv2.VideoCapture(0)
    _,frame = cap.read()
    frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    frame = cv2.GaussianBlur(frame,(7,7),0,0)
    cap.release()
    return frame

time.sleep(2)

img1 = getFrame()
cv2.imshow('Img1',img1)
cv2.imwrite('Image1.png',img1)

time.sleep(10)
img2 = getFrame()
cv2.imshow('Img2',img2)
cv2.imwrite('Image2.png', img2)
imgDiff = np.absolute(cv2.subtract(img2,img1))
cv2.imwrite('ImgDiff.png',imgDiff)
#ret, imgDiff = cv2.threshold(imgDiff,0,255,cv2.THRESH_BINARY_INV)
cv2.imshow('Diff',imgDiff)
print(np.amax(imgDiff))
cv2.waitKey(0) & 0xFF



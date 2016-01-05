#!usr/bin/python
import cv2
import numpy as np
img = cv2.imread('imgDiff.png',0)
cv2.imshow('Diff',cv2.imread('imgDiff.png',0))
cv2.waitKey(0)

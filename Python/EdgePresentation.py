import numpy as np
import sys
sys.path.append('/usr/local/lib/python2.7/site-packages')
from PIL import Image
import zbar
import time
import cv2


cap = cv2.VideoCapture(0)
time.sleep(0.1)

while(True):
    _,rawImage = cap.read()


    #image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    #pil_im = Image.fromarray(image)
    image = cv2.cvtColor(rawImage, cv2.COLOR_BGR2GRAY)    
    image = cv2.GaussianBlur(image,(7,7),0)
    edged = cv2.Canny(image,50,150)
    edged = cv2.resize(edged,(1920,1080))
    cv2.imshow('Edges',edged)
    cv2.waitKey(1) & 0xFF

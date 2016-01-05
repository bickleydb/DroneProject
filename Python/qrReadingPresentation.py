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
    height, width = image.shape[:2]
    zbar_image = zbar.Image(width, height, 'Y800', image.tostring())

    scanner = zbar.ImageScanner()
    scanner.parse_config('enable')
    scanner.scan(zbar_image)
    yPos = 100
    sleep = True
    for symbol in zbar_image:
        print 'decoded', symbol.type, 'symbol', '"%s"' % symbol.data
        cv2.putText(rawImage, symbol.data, (100, yPos),
                    cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,255), 2)
        if sleep:
            time.sleep(.5)
            sleep = False
        yPos += 100
    rawImage = cv2.resize(rawImage,(1920,1080))
    cv2.imshow("Frame", rawImage)
    key = cv2.waitKey(1) & 0xFF
#    rawCapture.truncate(0)

    if key == ord("q"):
        break
   

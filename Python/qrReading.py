import numpy as np
import sys
sys.path.append('/usr/local/lib/python2.7/site-packages')
from PIL import Image
import zbar
import time
import cv2


#camera = PiCamera()
#camera.resolution = (640,480)
#camera.framerate = 64
#rawCapture = PiRGBArray(camera, size=(640, 480))
#time.sleep(0.1)

#for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
#rawImage = frame.array
camera = cv2.VideoCapture(0)

while(True):
    (grabbed, rawImage) = camera.read()
    #image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    #pil_im = Image.fromarray(image)
    image = cv2.cvtColor(rawImage, cv2.COLOR_BGR2GRAY)    
    height, width = image.shape[:2]
    zbar_image = zbar.Image(width, height, 'Y800', image.tostring())

    scanner = zbar.ImageScanner()
    scanner.parse_config('enable')
    scanner.scan(zbar_image)
    xPos = 100;
    for symbol in zbar_image: 
        print 'decoded', symbol.type, 'symbol', '"%s"' % symbol.data
        cv2.putText(rawImage, symbol.data, (100, xPos),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,0,255), 2)
        time.sleep(1.0)
        xPos+=100

    cv2.imshow("Frame", rawImage)
    key = cv2.waitKey(1) & 0xFF
    #rawCapture.truncate(0)

    if key == ord("q"):
        break
   

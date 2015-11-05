#!usr/bin/python
import cv2
import cv2.cv as cv
import numpy as np

cap = cv2.VideoCapture(0)
prev = np.empty((1920,640,3))
diff = np.empty((480,640,3))
avgBright = 0;
detector = cv2.SimpleBlobDetector()


ranThrough = False


    
def getFrame():
    _,frame = cap.read()
    frame = cv2.GaussianBlur(frame,(9,9),0,0)
    frame = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
   # _,_,b = cv2.split(frame)
    return frame

def getCoor(frame):
    vals = (31,29,23,19,17,13,11,7)
    for y in vals:
        for x in range(0,frame.shape[1]):
            if np.any(frame[x*frame.shape[1]:x*frame.shape[1]+frame.shape[0]:y]):
                print(x)

def reduceBrightness(img):
    lower_brightness = np.array([000,0,200])
    upper_brightness = np.array([200,200,300])
    mask = cv2.inRange(img,lower_brightness,upper_brightness)
    img = cv2.bitwise_and(img,img,mask=mask)
    return img

def removeNonRed(img):
    img = cv2.cvtColor(img,cv2.COLOR_HSV2BGR)
    b,g,r = cv2.split(img)
    deltaB = cv2.subtract(r,b)
    deltaG = cv2.subtract(r,g)
    r2 = cv2.bitwise_and(r,r,mask=deltaB)
    g2 = cv2.bitwise_and(r2,r2,mask=deltaG)
    img = cv2.merge((r2,r2,r2))
    cv2.imshow('test',img)
    img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    return img;



def getLocation(img):
    test = img.reshape(img.size)
    locationY = []
    locationX = []
    for x in [17,13,11,7,5]:
        if np.any(img[x:(x*img.shape[0]+1):5]):
            testing = np.where(img != 0)
            locationY = testing[0]
            break
    
    if(len(locationY) > 0):
        for x in [17,13,11,7,5]:
            columns = np.array( range(0,img.shape[1],x))
            #print(img[locationY[0]],columns)
            if np.any(img[locationY[0],columns]):
                testing = np.where(img[locationY[0],columns] != 0)
                print(testing)
                locationX = testing[0]
                locationX = locationX * x
                #print(locationX[0])
                break
    if(len(locationX) > 0 and len(locationY) > 0):
        return locationX[0],locationY[0]
    else:
        return -1,-1

def edgeDetection(img):
    img2 = cv2.Canny(img,100,200)
    cv2.imshow('edges',img2)

while True:
    img = getFrame()
    cv2.imshow('Raw',img)
    #img = normalizeBrightness(img)
    img = reduceBrightness(img)
    cv2.imshow('afterbright',img)
    img = removeNonRed(img)
    cv2.imshow('afterRed',img)
    _,_,img = cv2.split(img)
    maxVal = np.amax(img)

    print(getLocation(img))
   
 #   imgCpy = np.reshape(img,img.size)
 #   arr = np.where(img == maxVal)
   # print(arr)

  #  img = cv2.cvtColor(img,cv2.COLOR_HSV2BGR)

    #cv2.imshow('Prev',img)
    x,y = getLocation(img)

    if y != -1  and x != -1:
        cv2.line(img,(0,y),(img.shape[1],y),255,10)
        cv2.line(img,(x,0),(x,img.shape[0]),255,10)
        if(x-img.shape[0]/2 < 50 and x-img.shape[0]/2 > -50):
            print("Forward")
        elif (x-img.shape[0]/2 > 0):
            print("Right")
        else:
            print("Left")

    cv2.imshow('testing', img)
    if ranThrough:
        diff = cv2.subtract(img,prev)
 #       cv2.imshow('Diff',diff)

#    cv2.imshow('Test',img)
    cv2.waitKey(1)
    prev = img
    ranThrough = True



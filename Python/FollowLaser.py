#!usr/bin/python
import cv2
import numpy as np

cap = cv2.VideoCapture(0)
prev = np.empty((1920,640,3))
diff = np.empty((480,640,3))
avgBright = 0;
#clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
#stuff = cv2.createBackgroundSubtractorMOG()
ranThrough = False


    
def getFrame():
    _,frame = cap.read()
    #cv2.imshow('Most raw',frame)
    #frame = cv2.GaussianBlur(frame,(9,9),0,0)
    frame = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
   # _,_,b = cv2.split(frame)
    return frame

def getCoor(frame):
    vals = (31,29,23,19,17,13,11,7)
    for y in vals:
        for x in range(0,frame.shape[1]):
            if np.any(frame[x*frame.shape[1]:x*frame.shape[1]+frame.shape[0]:y]):
                return x

def reduceBrightness(img):
    #lower_brightness = np.array([100,0,200])
    #upper_brightness = np.array([200,200,300])
    _,_,b = cv2.split(img)
    mean,stdDev = cv2.meanStdDev(b)
    mean = int(mean[0])
    stdDev = int(stdDev[0])
    lower_brightness = np.array([-1000,-1000,mean+stdDev])
    upper_brightness = np.array([2000,1000,300])
    mask = cv2.inRange(img,lower_brightness,upper_brightness)
    img = cv2.bitwise_and(img,img,mask=mask)
    return img

def reduceHue(img):
    lower_hue = np.array([100,-1000,-1000])
    upper_hue = np.array([150,1000,3000])
    mask = cv2.inRange(img,lower_hue,upper_hue)
    img = cv2.bitwise_and(img,img,mask=mask)
    return img
    
def reduceSat(img):
    _,s,_ = cv2.split(img)
    mean, stdDev = cv2.meanStdDev(s)
    mean = int(mean[0])
    stdDev = int(stdDev[0])
    lower_sat = np.array([-1000,50,-1000])
    upper_sat = np.array([1000,200,3000])
    mask = cv2.inRange(img,lower_sat,upper_sat)
    img = cv2.bitwise_and(img,img,mask=mask)
    return img



def removeNonRed(img):
    img = cv2.cvtColor(img,cv2.COLOR_HSV2BGR)
    #cv2.imshow('tasdf',img)
    b,g,r = cv2.split(img)

    diffB = cv2.subtract(r,b)
    diffG = cv2.subtract(r,g)
    diffMatrix = cv2.max(diffB,diffG)
    
   # maxR = np.amax(r)
    #lowerRed = np.array([0,0,maxR-1])
    #upperRed = np.array([255,255,maxR])
    #r = cv2.inRange(img,lowerRed,upperRed)
    imgTest = cv2.bitwise_and(r,r,mask=diffMatrix)
   # cv2.imshow('asdf',imgTest)
   # r2 = cv2.bitwise_and(r,r,mask=deltaB)
   # g2 = cv2.bitwise_and(r2,r2,mask=deltaG)
   # b = cv2.bitwise_and(b,b,mask=r2)
   # g = cv2.bitwise_and(g,g,mask=r2)
    #cv2.imshow('testinaswdf',r2)
    #img = cv2.merge((b,g,r2))
    #cv2.imshow('test',img)
    img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    return imgTest;



def removeRedSecondPass(img):
    img = cv2.cvtColor(img,cv2.COLOR_HSV2BGR)
    _,_,r = cv2.split(img)
    maxR = np.amax(r)
    t,b = cv2.threshold(r,maxR-5,0,cv2.THRESH_TOZERO)
    #b = cv2.addWeighted(b,1,b,1,5oper0)
    #b = cv2.equalizeHist(b)
   # cv2.imshow('test2',b)
    return b;




def edgeDetection(img):
    img2 = cv2.Canny(img,100,200)
    cv2.imshow('edges',img2)


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

while True:
    imgList = []
    #for i in range(0,3):
    img = getFrame()
 #   img2 = stuff.apply(img)
    
#    cv2.imshow('Raw',img2)
    for i in range(0,5):
        img = reduceBrightness(img)
        h,s,v = cv2.split(img)
        v = cv2.equalizeHist(v)
        img = cv2.merge((h,s,v))
            #img = cv2.GaussianBlur(img,(9,9),0,0)

#    cv2.imshow('afterbright',img)
    img = reduceSat(img)
 #   cv2.imshow('aftersat',img)
    img = removeNonRed(img)
    imgList.append(img)
  #  cv2.imshow('afterRed',img)

    #cv2.imshow('img',img)
    #b = removeRedSecondPass(img)
    #cv2.imshow('img',b)
    #imgList.append(b)
    
   # curImage = imgList[0]
   # for x in imgList[1::]:
    #    ret,mask = cv2.threshold(x,100,255,cv2.THRESH_BINARY)
     #   curImage = cv2.bitwise_or(curImage,curImage,mask=mask)

    #cv2.imshow('Testing',curImage)
    #cv2.waitKey(1)
    #    diff = cv2.bitwise_and(diff,diff,mask=b)
     #   cv2.imshow('Diff',diff)

  ##  cv2.imshow('Diff',diff)
   # _,_,img = cv2.split(img)
#    x,y = getLocation(img)
    
#    keyPoints =detector.detect(img)
#    with_points = cv2.drawKeypoints(img,keyPoints,np.array([]), (0,0,255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
#    cv2.imshow("Keypoints",with_points)
    
    if y != -1  and x != -1:
        cv2.line(img,(0,y),(img.shape[1],y),255,10)
        cv2.line(img,(x,0),(x,img.shape[0]),255,10)
        if(x-img.shape[0]/2 < 50 and x-img.shape[0]/2 > -50):
            print("Forward")
        elif (x-img.shape[0]/2 > 0):
            print("Right")
        else:
            print("Left")

    cv2.imshow('Testing',img)
 #   cv2.waitKey(1)

     #       prev = img
     #       ranThrough = True



#!/usr/bin/python
import numpy as np
import socket
import time
import cv2

def getFrame():
    cap = cv2.VideoCapture(0)
    frame = np.empty((480,640,3))
    _, frame = cap.read()
    
    #frame = cv2.GaussianBlur(frame, (7,7), 3)
    cap.release()
    return frame
    

def changeFrame(frame):
    output = np.empty((480,640,3))
    b,g,red = cv2.split(frame)
    #output= cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
    return red


def onEnd():
    cap.release()
    cv2.destroyAllWindows()


#toServer = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
#toServer.connect(('localhost',50003))

time.sleep(1)
cur = getFrame()
cur = changeFrame(cur)
cv2.imshow('First Image',cur)
cv2.waitKey(0)
cur2 = getFrame()
cur2 = changeFrame(cur2)
cv2.imshow('Second Image', cur2)
cv2.waitKey(0)
cur3 = cv2.subtract(cur,cur2)
cv2.imshow('Diff',cur3)

cv2.waitKey(0)

for i in range(0,3):
    cur= getFrame()
    cur = changeFrame(cur)
    np.copyto(delta,cur)
    cur2 = cv2.bitwise_xor(cur,previousFrame)
    previousFrame = cur
  #  cur2 = cv2.bitwise_xor(cur,previousFrame)
    cv2.imshow('Test'+i,cur2)
    cv2.waitKey(1)
    #np.copyto(previousFrame,delta)
    #stuff, stuff2 = cv2.imencode('.jpg',workOn)
    #print(np.nanmax(workOn))
    #print(np.nanmin(workOn))
    #data = np.array(stuff2)
    #toSend = data.tostring()
    #toServer.send(str(len(toSend)).ljust(16))
    #toServer.send(toSend)
    
    #byteArr = workOn.tostring()
    #print(workOn.shape)
   # toServer.send(str(len(byteArr)))
   # toServer.send('ENDOFLENGTH')
   # toServer.send(str(workOn.shape[0]))
   # toServer.send('ENDFIRSTTUPLE')
   # toServer.send(str(workOn.shape[1]))
   # toServer.send('ENDSECONDTUPLE')
   # toServer.send(byteArr)
    #print(len(byteArr))
   # toServer.send('ENDBYTEARR')

#display = changeFrame(workOn)
#cv2.imshow('test',display)

#!/usr/bin/python
import numpy as np
import socket
import time
import cv2

def getFrame():
    ret, frame = cap.read()
    cv2.imshow('test',frame)
    return cv2.resize(frame,(200,200))
    

def changeFrame(frame):
    cv2.cvtColor(frame,output,CV_RGB2GRAY)
    #cv2.adaptiveThreshold(output, newVar,THRESH_BINARY,
    return frame


def onEnd():
    cap.release()
    cv2.destroyAllWindows()

cap = cv2.VideoCapture(0)
toServer = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
toServer.connect(('localhost',50003))

while(True):
    workOn = getFrame()
    stuff, stuff2 = cv2.imencode('.jpg',workOn)
    data = np.array(stuff2)
    toSend = data.tostring()
    toServer.send(str(len(toSend)).ljust(16))
    toServer.send(toSend)
    
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

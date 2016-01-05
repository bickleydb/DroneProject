 #!/usr/bin/python
import numpy as np
import struct
import socket
import cv2
import io


serverSocket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
serverSocket.bind(('',50000))
serverSocket.listen(5)
connection = serverSocket.accept()[0].makefile('rb')



def dealWithClient(clientSocket):
        print('Found Someone!')
        prev = np.empty((400,400,3))
        while True:
#                print("A")
                img = getImage()
                if not img.size != 0:
                        return
                cv2.imshow('Raw From Pi',img)
                #displayDelta(img,prev)
                prev = img
                cv2.waitKey(1)

def getImage():
        image_len = struct.unpack('<L', connection.read(struct.calcsize('<L')))[0]
        if not image_len:
                return
        image_stream = io.BytesIO()
        image_stream.write(connection.read(image_len))
        image_stream.seek(0)
 #       print('B')
        #img = np.fromstring(image_stream.getvalue(),dtype=np.uint8)
        img = cv2.imdecode(np.fromstring(image_stream.getvalue(), dtype=np.uint8),1)
        
        return img

def displayDelta(img, prev):
        gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
        ret, mask = cv2.threshold(prev,10,255,cv2.THRESH_BINARY)
        img2 = cv2.bitwise_and(gray,gray,mask=mask)
        cv2.imshow('Delta',img2)
        return img;


def displayEdges(img):
        img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        img = cv2.GaussianBlur(img,(5,5),0)
        img = cv2.Canny(img,100,-200)
        cv2.imshow('Edges',img)
        cv2.waitKey(1)
        
dealWithClient(connection)    


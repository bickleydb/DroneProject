 #!/usr/bin/python
import numpy as np
import socket
import cv2


cv2.namedWindow('DID THIS WORK')
serverSocket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
serverSocket.bind(('192.168.1.114',50003))
serverSocket.listen(5)
(clientSocket, addr) = serverSocket.accept()


def dealWithClient(clientSocket):
        print('Found Someone!')
        while(True):
                length = clientSocket.recv(16)
                print(length)
                data = clientSocket.recv(int(length))
                stuff = np.fromstring(data, dtype='uint8')
                decode = cv2.imdecode(stuff,1)
                cv2.imshow('DID THIS WORK',decode)
                if(cv2.waitKey(1) & 0xFF == ord('q')):
                   return
                print("AAAAAAAAAAAAAA")
        #numBytesRcv = 0
        #dataStr = ''
        #while('ENDBYTEARR' not in dataStr):
        #    dataByte = clientSocket.recv(1)
        #    dataStr += dataByte 
        #endOfTotalSize = dataStr[0:dataStr.find('ENDOFLENGTH')]
        #numBytes = len(endOfTotalSize)
        #dataStr = dataStr[numBytes+11:]
        #endOfFirstTuple = dataStr[0:dataStr.find('ENDFIRSTTUPLE')]
        #print(endOfFirstTuple)
        #numBytes = len(endOfFirstTuple)
        #dataStr = dataStr[numBytes+13:]
        #endOfSecondTuple = dataStr[0:dataStr.find('ENDSECONDTUPLE')]
        #print(endOfSecondTuple);
        #numBytes = len(endOfSecondTuple)
        #dataStr = dataStr[numBytes+14:]
        #dataStr = dataStr[0:dataStr.find('ENDBYTEARR')]
        #numpyArr = np.fromstring(dataStr)
      #  cv2.imdecode(stuff,1)
        #print(numpyArr.shape)
       # img = stuff.reshape((100,100))
        #cv2.imshow('',stuff)              
       # cv2.waitKey(0)


dealWithClient(clientSocket)    


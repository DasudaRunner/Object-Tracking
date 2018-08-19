import cv2
import numpy

cap = cv2.VideoCapture(0)
cnt=0
while True:
    ret,frame = cap.read()
    key=cv2.waitKey(5)
    if key & 0xff==ord('s'):
        cnt+=1
        cv2.imwrite('./imgs/'+str(cnt)+'.jpg',frame)
        print('saved!!!')
    if key & 0xff==ord('q'):
        break
    cv2.imshow('out',frame)

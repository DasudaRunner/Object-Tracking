import cv2
import numpy
import os
cap = cv2.VideoCapture(0)
cnt=0
print('Perss q to exit the program !')
print('Perss s to save image !')
for i in os.listdir('../imgs'):
	if i[-4:]=='.jpg':
		os.remove('../imgs/'+i)
while True:
    ret,frame = cap.read()
    cv2.imshow('out',frame)
    key=cv2.waitKey(5)
    if (key & 0xff)==ord('s'):
        cnt+=1
        cv2.imwrite('../imgs/'+str(cnt)+'.jpg',frame)
        print('image has been saved in imgs/'+str(cnt)+'.jpg')
    if key & 0xff==ord('q'):
        break

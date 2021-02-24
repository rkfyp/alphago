import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import numpy as np
import os
import cv2


'''
cap = cv2.VideoCapture('pics/%04d.jpg'  )
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter(r'pics/mysavedmovie.mp4',fourcc, 20, (512,512))
while(True):
    ret, frame = cap.read()
    if not ret:
        break  
    
    out.write(frame)
    cv2.imshow('frame',frame)
    if cv2.waitKey(30) & 0xFF == ord('q'):
        break
out.release()
cap.release()
cv2.destroyAllWindows()
'''
print (cv2.__version__)

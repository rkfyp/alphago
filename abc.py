import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image


lower_red = np.array([50,50,50])
upper_red = np.array([180,255,255])
kernel= None
foog = cv2.createBackgroundSubtractorMOG2()
# initlize video capture object
canny_on_video = False
path_of_image = 'depth/0034.jpg'

if canny_on_video :
    cap = cv2.VideoCapture('depth/depthvideo.mp4')




# you can set custom kernel size if you want


# initilize background subtractor object


while(1):
    if canny_on_video:
        ret, frame = cap.read()
        if not ret:
            break
    else:
        frame = cv2.imread(path_of_image,1)
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_red, upper_red)
    # apply background subtraction
    fgmask = foog.apply(mask)
    
    # get rid of the shadows
    ret, fgmask = cv2.threshold(fgmask, 250, 255, cv2.THRESH_BINARY)
    
    # apply some morphological operations to make sure you have a good mask
    fgmask = cv2.erode(fgmask,kernel,iterations = 1)
    fgmask = cv2.dilate(fgmask,kernel,iterations = 2)
    
    # Detect contours in the frame
    image, contours, heirarchy = cv2.findContours(fgmask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    
    # loop over each contour in a given frame
    for cnt in contours:
        
        # make sure the contour area is somewhat hihger than some threshold to make sure its a car and not some noise.
        if cv2.contourArea(cnt) > 600:
            
            # Draw a bounding box around the car and labet it as car detected
            x,y,w,h = cv2.boundingRect(cnt)
            cv2.rectangle(frame,(x ,y),(x+w,y+h),(0,0,255),2)
            cv2.putText(frame,'Car Detected',(x,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0,255,0), 1, cv2.LINE_AA)

 
    # Stack all both frames and show the image
    fgmask_3 = cv2.cvtColor(fgmask, cv2.COLOR_GRAY2BGR)
    stacked = np.hstack((fgmask_3,frame))
    

    plt.figure(figsize=[20,20])
    plt.subplot(121);plt.imshow(fgmask_3[:,:,::-1]);plt.title("Original Image")
    plt.subplot(122);plt.imshow(frame[:,:,::-1]);plt.title("Modified Image")



    #cv2.imshow('All three',cv2.resize(stacked,None,fx=0.65,fy=0.65))
    #cv2.imshow('detected',fgmask_3)
    '''
    k = cv2.waitKey() & 0xff
    if k == ord('q'):
        break
    '''
if canny_on_video:
    cap.release()


#cv2.destroyAllWindows()
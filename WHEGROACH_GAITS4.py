#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 20 00:57:19 2020

@author: Rafi & Kab
"""
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import vrep
from vrep import *
import vrepConst
import sys
import time
import math
import numpy as np
import cv2


########################################## INITIALIZATION VALUES ##################################################


################ Initialization of handles. Do not change the following section ###################################
vrep.simxFinish(-1)
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5)

#tf.keras.backend.set_learning_phase(False)

if clientID!=-1:
    print ("connected to remote api server")
    vrep.simxSynchronous(clientID, 1)
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait)
   
################################################_HANDLES_############################################################
    
ret1,joint2 = vrep.simxGetObjectHandle(clientID,'joint2',vrep.simx_opmode_oneshot_wait)
ret2,joint4 = vrep.simxGetObjectHandle(clientID,'joint4',vrep.simx_opmode_oneshot_wait)
ret3,joint6 = vrep.simxGetObjectHandle(clientID,'joint6',vrep.simx_opmode_oneshot_wait)
    
ret4,joint1 = vrep.simxGetObjectHandle(clientID,'joint1',vrep.simx_opmode_oneshot_wait)
ret5,joint3 = vrep.simxGetObjectHandle(clientID,'joint3',vrep.simx_opmode_oneshot_wait)
ret6,joint5 = vrep.simxGetObjectHandle(clientID,'joint5',vrep.simx_opmode_oneshot_wait)

ret7,lidJoint = vrep.simxGetObjectHandle(clientID,'lidJoint',vrep.simx_opmode_oneshot_wait)

ret8,KJoint0 = vrep.simxGetObjectHandle(clientID,'Kinect_J0',vrep.simx_opmode_oneshot_wait)
ret9,KJoint1 = vrep.simxGetObjectHandle(clientID,'Kinect_J1',vrep.simx_opmode_oneshot_wait)
ret10,KJoint2 = vrep.simxGetObjectHandle(clientID,'Kinect_J2',vrep.simx_opmode_oneshot_wait)


ret12,kinect_rgb = vrep.simxGetObjectHandle(clientID,'kinect_rgb',vrep.simx_opmode_oneshot_wait)
ret11,kinect_depth = vrep.simxGetObjectHandle(clientID,'kinect_depth',vrep.simx_opmode_oneshot_wait)

#########################################################################################################
#   TRIPOD GAITS
#=============================================================================

def moveBackwardT():
    ccw_F= False
    a1=0
    b1=0
    while (True):

        if (ccw_F == False):   
            vrep.simxSetJointTargetPosition(clientID,joint2,b1*(-360*math.pi/180),vrep.simx_opmode_oneshot_wait)
            vrep.simxSetJointTargetPosition(clientID,joint4,b1*(-360*math.pi/180),vrep.simx_opmode_oneshot_wait)
            vrep.simxSetJointTargetPosition(clientID,joint6,b1*(-360*math.pi/180),vrep.simx_opmode_oneshot_wait)
            b1=b1+1
            time.sleep(1)
            ccw_F= True
            
            
        elif (ccw_F ==True): 
            vrep.simxSetJointTargetPosition(clientID,joint1,a1*(-360*math.pi/180),vrep.simx_opmode_oneshot_wait)
            vrep.simxSetJointTargetPosition(clientID,joint3,a1*(-360*math.pi/180),vrep.simx_opmode_oneshot_wait)
            vrep.simxSetJointTargetPosition(clientID,joint5,a1*(-360*math.pi/180),vrep.simx_opmode_oneshot_wait)
            a1=a1+1
            time.sleep(1)
            ccw_F=False
            

def moveForwardT(a):
    ccw_F= False
    a1=0
    b1=0
    
    while (True):

        if (ccw_F == False): 
            b1+=1 
            vrep.simxSetJointTargetPosition(clientID,joint2,b1*(360*math.pi/180),vrep.simx_opmode_oneshot_wait)
            vrep.simxSetJointTargetPosition(clientID,joint4,b1*(360*math.pi/180),vrep.simx_opmode_oneshot_wait)
            vrep.simxSetJointTargetPosition(clientID,joint6,b1*(360*math.pi/180),vrep.simx_opmode_oneshot_wait)
            imgdepth()
            ccw_F= True
            time.sleep(0.9)
            
        elif (ccw_F ==True): 
            a1+=1
            vrep.simxSetJointTargetPosition(clientID,joint1,a1*(360*math.pi/180),vrep.simx_opmode_oneshot_wait)
            vrep.simxSetJointTargetPosition(clientID,joint3,a1*(360*math.pi/180),vrep.simx_opmode_oneshot_wait)
            vrep.simxSetJointTargetPosition(clientID,joint5,a1*(360*math.pi/180),vrep.simx_opmode_oneshot_wait)
            imgdepth()
            ccw_F=False
            time.sleep(1)
            #yield a1

        if a==True:
            break

        #yield a1
    #yield a1
        

def moveLeftT():
    c1=0
    while (True): 
        vrep.simxSetJointTargetPosition(clientID,joint4,c1*(360*math.pi/180),vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetPosition(clientID,joint6,c1*(360*math.pi/180),vrep.simx_opmode_oneshot_wait)
        c1=c1+1
        time.sleep(1)
        #yield c1
        #if a==True:
        #    break
           
def moveRightT():
    a1=0
    b1=0
    while (True): 
        vrep.simxSetJointTargetPosition(clientID,joint1,b1*(360*math.pi/180),vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetPosition(clientID,joint3,b1*(360*math.pi/180),vrep.simx_opmode_oneshot_wait)
        b1=b1+1
       
#==============================================================================================================

#########################################################################################################
#   QUADRUPED GAITS
#=============================================================================
def stairAscDsc():
    a1=0
    b1=0
    c1=0
    a=0
    b=0
    c=0
    while (True):

        if (c==0): 
            vrep.simxSetJointTargetPosition(clientID,joint3,c1*(360*math.pi/180),vrep.simx_opmode_oneshot_wait)
            vrep.simxSetJointTargetPosition(clientID,joint6,c1*(360*math.pi/180),vrep.simx_opmode_oneshot_wait)
            c1=c1+1
            time.sleep(1)
            b=1
 
        if (b ==1): 
            vrep.simxSetJointTargetPosition(clientID,joint2,a1*(360*math.pi/180),vrep.simx_opmode_oneshot_wait)
            vrep.simxSetJointTargetPosition(clientID,joint5,a1*(360*math.pi/180),vrep.simx_opmode_oneshot_wait)
            a1=a1+1
            time.sleep(1)
            a=1
            
        if (a == 1): 
            vrep.simxSetJointTargetPosition(clientID,joint1,b1*(360*math.pi/180),vrep.simx_opmode_oneshot_wait)
            vrep.simxSetJointTargetPosition(clientID,joint4,b1*(360*math.pi/180),vrep.simx_opmode_oneshot_wait)
            b1=b1+1
            time.sleep(1)
            c=0
#==============================================================================================================

#########################################################################################################
#   Vision LiDar Controlling
#=============================================================================

def ldrCtr():
    simxSetJointPosition(clientID,lidJoint,0.1,vrep.simx_opmode_oneshot_wait)

def kntCtr():
    simxSetJointPosition(clientID,KJoint0,0.08,vrep.simx_opmode_oneshot_wait)
    time.sleep(1)
    simxSetJointPosition(clientID,KJoint1,-0.125,vrep.simx_opmode_oneshot_wait)
    time.sleep(1)
    a1=0
    b1=0
    
    images = []
    stk=[]  
    ccw_F= False

    while (True):
        if (ccw_F == False): 
            simxSetJointPosition(clientID,KJoint2,90*math.pi/180,vrep.simx_opmode_oneshot_wait)
            ccw_F= True
            time.sleep(0.5)

        elif (ccw_F ==True):
            
            while (True):
                
                ret121 ,arrayresolution,arrayimage=vrep.simxGetVisionSensorImage(clientID,kinect_depth,0,vrep.simx_opmode_streaming)
                time.sleep(0.5)
                ret121 ,arrayresolution,arrayimage=vrep.simxGetVisionSensorImage(clientID,kinect_depth,0,vrep.simx_opmode_buffer)
                
                img2=np.array(arrayimage,dtype=np.uint8)
                img2.resize([arrayresolution[0],arrayresolution[1],3 ])
                aaa=cv2.flip(img2, -1)
                bb1=cv2.flip(aaa, 1)
                bb = cv2.cvtColor(bb1, cv2.COLOR_BGR2RGB)
                images.append(bb)
                b1=b1+1 
                if b1 >=3.5:
                    break
                simxSetJointPosition(clientID,KJoint2,-b1+(90*math.pi/180),vrep.simx_opmode_oneshot_wait)
                time.sleep(0.5)
            time.sleep(0.5)
            simxSetJointPosition(clientID,KJoint2,0*math.pi/180,vrep.simx_opmode_oneshot_wait)
            time.sleep(1.5)
            simxSetJointPosition(clientID,KJoint1,0,vrep.simx_opmode_oneshot_wait)
            time.sleep(1.5)
            simxSetJointPosition(clientID,KJoint0,-0,vrep.simx_opmode_oneshot_wait)
            
            stk_img = np.hstack((images[0],images[1],images[2],images[3]))
            cv2.imwrite('img.jpg', stk_img)
        
            hsv = cv2.cvtColor(stk_img, cv2.COLOR_BGR2HSV)
            lower_red_0 = np.array([-9,245,215])
            upper_red_0 = np.array([11,265,295])

            lower_red_1 = np.array([168,245,202])
            upper_red_1 = np.array([188,265,282])

            lower_red_2 = np.array([-4,245,215])
            upper_red_2 = np.array([16,265,295])

            hsv_L=[lower_red_0,lower_red_1,lower_red_2]
            hsv_U=[upper_red_0,upper_red_1,upper_red_2]

            for i in range(3):
                mask = cv2.inRange(hsv, hsv_L[i], hsv_U[i])
                #fgmask = cv2.erode(mask,None,iterations = 3)
                
                #contours, heirarchy = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                '''
                for cnt in contours:
                    if cv2.contourArea(cnt) > 600:
                        #x,y,w,h = cv2.boundingRect(cnt)
                        #cv2.rectangle(fgmask,(x ,y),(x+w,y+h),(100,150,200),3)
                        #cv2.putText(fgmask,'Object Detected',(x,y+20), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (120,255,80), 1, cv2.LINE_AA)
                '''
                stk.append(mask)
                #cv2.imwrite("dd/%04i.jpg" %i, mask)
            add_img = (stk[0]+stk[1]+stk[2])
            add_img = cv2.erode(add_img,None,iterations = 3)
            cv2.imwrite('mask11.jpg', add_img)
            #return fgmask
            
            break
    #return fgmask
            
#==================================================================================================================
    


    
#########################################################################################################
#   Video and Image Capture
#=============================================================================
def imgrgb():
    img1=[]
    i=0
    while True:
        ret122 ,arrayresolution1,arrayimage1=simxGetVisionSensorImage(clientID,kinect_rgb,0,vrep.simx_opmode_streaming)
        time.sleep(0.5)
        ret122 ,arrayresolution1,arrayimage1=simxGetVisionSensorImage(clientID,kinect_rgb,0,vrep.simx_opmode_buffer)
        
        img1=np.array(arrayimage1,dtype=np.uint8)
        img1.resize([arrayresolution1[0],arrayresolution1[1],3 ])
        aa=cv2.flip(img1, -1)
        bbb=cv2.flip(aa, 1)
        ccc = cv2.cvtColor(bbb, cv2.COLOR_BGR2RGB)
        
        #cv2.imshow("camera", ccc)
        cv2.imwrite("rgb/%04i.jpg" %i, ccc)


        k = cv2.waitKey(30) & 0xFF
        i=i+1
        if k == ord('q'):
            break
        cv2.destroyAllWindows()


def vidrgb():   
    a=0
    #Initilize Video capture Object and points towards a video file for depth images
    cap = cv2.VideoCapture('rgb/%04d.jpg')
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(r'rgb/rgbvideo.mp4',fourcc, 20, (512,512))
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


def imgdepth():
    obj=False
    img2=[]
    #i=0
    while True:
        ret121 ,arrayresolution,arrayimage=vrep.simxGetVisionSensorImage(clientID,kinect_depth,0,vrep.simx_opmode_streaming)
        time.sleep(0.5)
        ret121 ,arrayresolution,arrayimage=vrep.simxGetVisionSensorImage(clientID,kinect_depth,0,vrep.simx_opmode_buffer)
        
        img2=np.array(arrayimage,dtype=np.uint8)
        print(img2) 
        img2.resize([arrayresolution[0],arrayresolution[1],3 ])
        aaa=cv2.flip(img2, -1)
        bb1=cv2.flip(aaa, 1)
        bb = cv2.cvtColor(bb1, cv2.COLOR_BGR2RGB)
        #cv2.imwrite("dd/imag3.jpg" , bb)

        hsv = cv2.cvtColor(bb, cv2.COLOR_BGR2HSV)
        lower_red = np.array([-9,245,215])
        upper_red = np.array([1,265,295])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        fgmask = cv2.erode(mask,None,iterations = 3)
        contours, heirarchy = cv2.findContours(fgmask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            if cv2.contourArea(cnt) > 600:
                #x,y,w,h = cv2.boundingRect(cnt)
                #cv2.rectangle(fgmask,(x ,y),(x+w,y+h),(100,150,200),3)
                #cv2.putText(fgmask,'Object Detected',(x,y+20), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (120,255,80), 1, cv2.LINE_AA)
                obj=True
              
        if obj == True:
            kntCtr()
            #print(fg1mask) 
            obj=False           
           
        if contours == []:
            break

        break 

def viddepth():   
    d=0
    #Initilize Video capture Object and points towards a video file for depth images
    cap1 = cv2.VideoCapture('depth/%04d.jpg')
    #fourcc = cv2.VideoWriter_fourcc(*'XVID')
    #out1 = cv2.VideoWriter(r'depth/depthvideo.mp4',fourcc, 10, (512,512),0)
    while(True):
        ret0, frame1 = cap1.read()
        if not ret0:
            break  
        


        #out1.write(fgmask)
        #cv2.imshow('frame',fgmask)
        #if cv2.waitKey(150) & 0xFF == ord('q'):
        #   break
    #out1.release()
    #cap1.release()
    #cv2.destroyAllWindows()






#==================================================================================================================
    

        




     


#======================================== FUNCTION END ===============================================================
#=======================================================================================================

vrep.simxSynchronousTrigger(clientID)
vrep.simxGetPingTime(clientID)

#ldrCtr()
#kntCtr()
#moveRightT()
#moveLeftT()
#moveForwardT(True)
#moveBackwardT()
#stairAscDsc()
#imgrgb()
#vidrgb()
#imgdepth()
#viddepth()
for aa in moveForwardT(False):
    pass

'''
a=False

path_of_image = 'depth/0000.jpg'

frame1 = cv2.imread(path_of_image,1)
kernel = None
hsv = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)
lower_red = np.array([-9,245,215])
upper_red = np.array([1,265,295])
mask = cv2.inRange(hsv, lower_red, upper_red)
fgmask = cv2.erode(mask,kernel,iterations = 3)
image, contours,heirarchy = cv2.findContours(fgmask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
print(contours)


for cnt in contours:
    if cv2.contourArea(cnt) >2500:
        x,y,w,h = cv2.boundingRect(cnt)
        cv2.rectangle(fgmask,(x ,y),(x+w,y+h),(255,255,255),2)
        cv2.putText(fgmask,'Object Detected',(x,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255,255,255), 1, cv2.LINE_AA)
        for aa in moveForwardT(False):
            if aa==4:
                moveForwardT(True)
                break
        time.sleep(1.5)
        th=100
        offset=x-th
        moveLeftT()

'''
'''
a1=moveForwardT(bb)
print(a1)
if a1==4:
    a1=moveForwardT(True)
    print(a1)
'''



'''
cv2.imshow("img1",frame1)
cv2.imshow("img",fgmask)
k = cv2.waitKey(1000) & 0xFF
cv2.destroyAllWindows()
'''
'''
if a==True:
    th=100
    offset=x-th
    a1=moveForwardT(False)
    if a1==4:
        moveForwardT(True)
'''

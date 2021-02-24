#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 20 00:57:19 2020

@author: Rafi
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
import os
from os.path import isfile, join




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
            

def moveForwardT():
    ccw_F= False
    a1=0
    b1=0
    while (True):

        if (ccw_F == False):   
            vrep.simxSetJointTargetPosition(clientID,joint2,b1*(360*math.pi/180),vrep.simx_opmode_oneshot_wait)
            vrep.simxSetJointTargetPosition(clientID,joint4,b1*(360*math.pi/180),vrep.simx_opmode_oneshot_wait)
            vrep.simxSetJointTargetPosition(clientID,joint6,b1*(360*math.pi/180),vrep.simx_opmode_oneshot_wait)
            b1=b1+1
            ccw_F= True
            time.sleep(1)
            
        elif (ccw_F ==True): 
            vrep.simxSetJointTargetPosition(clientID,joint1,a1*(360*math.pi/180),vrep.simx_opmode_oneshot_wait)
            vrep.simxSetJointTargetPosition(clientID,joint3,a1*(360*math.pi/180),vrep.simx_opmode_oneshot_wait)
            vrep.simxSetJointTargetPosition(clientID,joint5,a1*(360*math.pi/180),vrep.simx_opmode_oneshot_wait)
            a1=a1+1
            ccw_F=False
            time.sleep(1)
        if a1==4 :
            break
    time.sleep(1)
    moveLeftT()

def moveLeftT():
    a1=0
    b1=0
    while (True): 
        vrep.simxSetJointTargetPosition(clientID,joint4,b1*(360*math.pi/180),vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetPosition(clientID,joint6,b1*(360*math.pi/180),vrep.simx_opmode_oneshot_wait)
        b1=b1+1
            
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
    ccw_F= False
    
    while (True):
        if (ccw_F == False): 
            simxSetJointPosition(clientID,KJoint2,90*math.pi/180,vrep.simx_opmode_oneshot_wait)
            
            ccw_F= True
            time.sleep(0.5)
        elif (ccw_F ==True):
            
            while (True):
                b1=b1+0.04
                simxSetJointPosition(clientID,KJoint2,-b1+(90*math.pi/180),vrep.simx_opmode_oneshot_wait)
                
                time.sleep(0.1)
                if b1 >=3.1:
                    break
            time.sleep(0.5)
            simxSetJointPosition(clientID,KJoint2,0*math.pi/180,vrep.simx_opmode_oneshot_wait)
            time.sleep(1.5)
            simxSetJointPosition(clientID,KJoint1,0,vrep.simx_opmode_oneshot_wait)
            time.sleep(1.5)
            simxSetJointPosition(clientID,KJoint0,-0,vrep.simx_opmode_oneshot_wait)
            break
#==================================================================================================================
    


    
#########################################################################################################
#   Video and Image Capture
#=============================================================================




#==================================================================================================================
    

        




     


#======================================== FUNCTION END ===============================================================
#=======================================================================================================

vrep.simxSynchronousTrigger(clientID)
vrep.simxGetPingTime(clientID)

#ldrCtr()
#kntCtr()
#moveRightT()
#moveLeftT()
#moveForwardT()
#moveBackwardT()
#stairAscDsc()


#time.sleep(0.5)




img1=[]
img2=[]
i=0
while True:



#cv2.imshow('img',bb)
#cv2.imwrite('img.png', bb)

    ret122 ,arrayresolution1,arrayimage1=simxGetVisionSensorImage(clientID,kinect_rgb,0,vrep.simx_opmode_streaming)
    time.sleep(0.1)
    ret122 ,arrayresolution1,arrayimage1=simxGetVisionSensorImage(clientID,kinect_rgb,0,vrep.simx_opmode_buffer)
    
    img1=np.array(arrayimage1,dtype=np.uint8)
    img1.resize( [arrayresolution1[0],arrayresolution1[1],3 ])
    aa=cv2.flip(img1, -1)
    bbb=cv2.flip(aa, 1)
    ccc = cv2.cvtColor(bbb, cv2.COLOR_BGR2RGB)
    #cv2.imshow('img1',ccc)
    #cv2.imwrite('img1.png', img2.append(ccc)    )
    
    cv2.imshow("camera", ccc)
    cv2.imwrite("pics/%04i.jpg" %i, ccc)


    k = cv2.waitKey(30) & 0xFF
    i=i+1
    if k == ord('q'):
        break
    cv2.destroyAllWindows()
    



"""
cap = cv2.VideoCapture( )
while (True):
    ret ,frame = cap.read()
    if not ret:
        break
    frame = cv2.flip( frame, 1 )
    cv2.imshow("img",frame)
    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        break
    if k ==ord('s'):
        cv2.imwrite('media/M1/mysavedpicc.jpg',frame)
        cv2.imshow('Picture Saved',frame)
cap.release()
cv2.destroyAllWindows()
"""

'''
if clientID!=-1:
    ret11,kinect_depth = vrep.simxGetObjectHandle(clientID,'kinect_depth',vrep.simx_opmode_oneshot_wait)
    ret121 ,arrayresolution,arrayimage=vrep.simxGetVisionSensorImage(clientID,kinect_depth,0,vrep.simx_opmode_streaming)
    while (vrep.simxGetConnectionId(clientID) != -1):
        ret121 ,arrayresolution,arrayimage=vrep.simxGetVisionSensorImage(clientID,kinect_depth,0,vrep.simx_opmode_buffer)
        if ret121 == vrep.simx_return_ok:
            print ("image OK!!!")
            img=np.array(arrayimage,dtype=np.uint8)
            img.resize( [arrayresolution[1],arrayresolution[0],3])
            #aaa=cv2.flip(img, -1)
            #bb=cv2.flip(aaa, 1)
            cv2.imshow('img',img)
            if cv2.waitKey(5) & 0xFF == ord('q'):
                break
        elif ret121 == vrep.simx_return_novalue_flag:
            print ("no image yet")
            pass
        else:
            print (ret121)
else:
    print ("Failed to connect to remote API Server")
    vrep.simxFinish(clientID)
cap.release()
cv2.destroyAllWindows()
'''
'''
while (True):
    ret ,frame = ccc.read()
    if not ret:
        break
    frame = cv2.flip( frame, 1 )
    cv2.imshow("img",frame)
    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        break
    if k ==ord('s'):
        #cv2.imwrite('media/M1/mysavedpicc.jpg',frame)
        cv2.imshow('Picture Saved',frame)
ccc.release()
cv2.destroyAllWindows()
'''

'''

pathOut = 'video.avi'
fps = 0.5
frame_array = []
files = [f for f in os.listdir(ccc) if isfile(join(ccc, f))]
#for sorting the file names properly
files.sort(key = lambda x: x[5:-4])
files.sort()
frame_array = []
files = [f for f in os.listdir(ccc) if isfile(join(ccc, f))]
#for sorting the file names properly
files.sort(key = lambda x: x[5:-4])
for i in range(len(files)):
    filename=ccc + files[i]
    #reading each files
    img = cv2.imread(filename)
    height, width, layers = img.shape
    size = (width,height)
    
    #inserting the frames into an image array
    frame_array.append(img)
out = cv2.VideoWriter(pathOut,cv2.VideoWriter_fourcc(*'DIVX'), fps, size)
for i in range(len(frame_array)):
    # writing to a image array
    out.write(frame_array[i])
out.release()
'''
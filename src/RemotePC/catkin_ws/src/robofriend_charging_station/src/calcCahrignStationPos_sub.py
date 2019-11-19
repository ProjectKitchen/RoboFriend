#!/usr/bin/env python
'''calcCahrignStationPos ROS Node'''
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from roboFriendMsgs.msg import irCamData
'''
Camera Matrix
'''

'''
focalLength = 1350.0;
	
	cuboidWidth = 80.0f;
	cuboidHeight = 104.0f;
	cuboidDepth = 88.0f;
	resWidth = RESWIDTH;
	resHeight = RESHEIGHT;
	halfResWidth = resWidth/2;
	halfResHeight = resHeight/2;



modelPoints.push_back(cvPoint3D32f(
		(float) 0.0, 
		(float) 0.0, 
		(float) 0.0));
	modelPoints.push_back(cvPoint3D32f(
		(float) 0.0, 
		(float) -cuboidHeight, 
		(float) cuboidDepth));
	modelPoints.push_back(cvPoint3D32f(
		(float) cuboidWidth, 
		(float) -cuboidHeight, 
		(float) 0.0));
	modelPoints.push_back(cvPoint3D32f(
		(float) cuboidWidth, 
		(float) 0.0, 
		(float) cuboidDepth));
	positObject = cvCreatePOSITObject(&modelPoints[0],4);
'''

FX = 1350 #1087 #1350
FY = 1350 #1006 # 1350
#principle point, maybe middle of image
CX = 1024 / 2#  width
CY = 768 / 2#   hight

mtx = np.float32([[FX, 0, CX],
        [0,FY,CY],
        [0,0,1]]);

dist_coef = np.zeros(4)


#charging station
#object coordinates in cm
cuboidWidth = 80.0
cuboidHeight = 104.0
cuboidDepth = 88.0
#oben links
x0=0.0
y0=0.0
z0=0.0

#oben rechts
x1=0.0
y1=-cuboidHeight
z1=cuboidDepth

#unten rechts
x2=cuboidWidth
y2=-cuboidHeight
z2=0.0

#unten links
x3=cuboidWidth
y3=0.0
z3=cuboidDepth

token = [[0,0],[0,0],[0,0],[0,0]]

def sortPoints():
  global token
  localTok=token
  sortTok=token  

  highestX=0
  highestY=0
  secHighX=0
  secHighY=0
  
  for i in range(0,4):
    #X Values
    if localTok[i][0] > secHighX:
      if localTok[i][0] > highestX:
        secHighX=highestX        
        highestX=localTok[i][0]
      else:
        secHighX=localTok[i][0]
    # y Values
    if localTok[i][1] > secHighY:
      if localTok[i][1] > highestY:
        secHighY=highestX        
        highestY=localTok[i][1]
      else:
        secHighY=localTok[i][1]

  for i in range(0,4):
    if localTok[i][0] >= secHighX:
      if localTok[i][1] >= secHighY:
        sortTok[0]=localTok[i]
      else:
        sortTok[1]=localTok[i]
    else:
      if localTok[i][1] >= secHighY:
        sortTok[3]=localTok[i]
      else:
        sortTok[2]=localTok[i]

  token=sortTok

def callback(data):
    global token
    
    for i in range(0,4):
        x= 1023-data.tokens[i].x
        y=1023-data.tokens[i].y
        token[i]=[x,y]
    sortPoints()
        

def listener():
    global token
    '''calcCahrignStationPos Subscriber'''
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('calcCahrignStationPos', anonymous=True)

    rospy.Subscriber("roboIrToken", irCamData, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        objectPoint = np.float32([[x0, y0, z0],[x1,y1,z1],[x2,y2,z2],[x3,y3,z3]])
        imgPoint = np.float32([[token[0][0], token[0][1]],[token[1][0], token[1][1]],[token[2][0], token[2][1]],[token[3][0], token[3][1]]])
        rvec=np.float32([-1.5,-0.5,2.0])
        tvec=np.float32([5.0,-2.0,30.0])
        retval, rvec, tvec = cv2.solvePnP(objectPoint, imgPoint, mtx, dist_coef,rvec,tvec,True)
        print("X:",tvec[0]," Y:" ,tvec[1], " Z:",tvec[2])
        print(rvec)
        rate.sleep()


if __name__ == '__main__':
    listener()

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
FX = 1087
FY = 1006
#principle point, maybe middle of image
CX = 511
CY = 511

mtx = np.float32([[FX, 0, CX],
        [0,FY,CY],
        [0,0,1]]);

dist_coef = np.zeros(4)


#charging station
#object coordinates in cm
#oben links
x0=0.0
y0=0.0
z0=0.0

#oben rechts
x1=10.3
y1=8.3
z1=0.0

#unten rechts
x2=10.3
y2=0.0
z2=8.8

#unten links
x3=0.0
y3=8.3
z3=8.8

token = [[0,0],[0,0],[0,0],[0,0]]

def callback(data):
    global token
    
    for i in range(0,4):
        x= 1023-data.tokens[i].x
        y=1023-data.tokens[i].y
        token[i]=[x,y]
        

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
        retval, rvec, tvec = cv2.solvePnP(objectPoint, imgPoint, mtx, dist_coef)
        print("X:",tvec[0]," Y:" ,tvec[1], " Z:",tvec[2])
        rate.sleep()


if __name__ == '__main__':
    listener()

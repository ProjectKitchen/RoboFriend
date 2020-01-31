#!/usr/bin/env python
'''calcCahrignStationPos ROS Node

calc the position of the charging station from the positions of the IR tokens in camera image without opencv functions
'''

##################################################################### IMPORTS
import rospy
import numpy as np
from std_msgs.msg import String
from roboFriendMsgs.msg import irCamData, chargingStationValues
import math
import sys

##################################################################### globals
MAX_DELTA_X = 5

FX = 1280
FY = 1280
CX = 1024 / 2
CY = 768 / 2

FIX_LENGTH_X = 100.0 #real distance between teokens of the left and right side
FIX_LENGTH_Y = 35.0 #real distance in mm between upper and lower token

token = [[0,0],[0,0],[0,0],[0,0]]
valid_msg=False

######################################################################### functions

""" sort Points function

the points need to sort to get a stable and correct calculation of the chargingstation position
"""
def sortPoints():
  global token
  localTok=token[:][:]
  sortTok=token[:][:]

  highestX=0
  highestY=0
  
  for i in range(0,4):
    #X Values
    if localTok[i][0] > highestX:       
      highestX=localTok[i][0]
    if localTok[i][1] > highestY:       
      highestY=localTok[i][1]

  pair_1=0
  pair_2=0
  try:
    for i in range(0,4):
      #X Values
      if (localTok[i][0] >= highestX-MAX_DELTA_X):
        sortTok[pair_1]=localTok[i]
        pair_1+=1
      else:
        sortTok[pair_2+2]=localTok[i]
        pair_2+=1
   
    token=sortTok
    return True
  except:
    return False

""" callback for the ir Token msg

save msg data to global variables
"""
def callback(data):
    global token, valid_msg
    
    for i in range(0,4):
        x= 1024-data.tokens[i].x
        y=data.tokens[i].y
        token[i]=[x,y]
    if sortPoints() == False:
      valid_msg=False

""" calc alpha 

calc the angle between image plane and chragingstation
"""
def calc_alpha(dis1, dis2):
  global valid_msg
  try:
    delta_d = dis1 - dis2
    alpha = math.atan2(delta_d, (math.sqrt(FIX_LENGTH_X * FIX_LENGTH_X - delta_d * delta_d)))
  except:
    alpha=-180.0  
    valid_msg=False
  return alpha        

""" listener node main function

node subscribe to ir Token data and published position to chargingstion
"""
def listener():
    global token
    rospy.init_node('calcCahrignStationPos', anonymous=True)

    rospy.Subscriber("roboIrToken", irCamData, callback)
    pub=rospy.Publisher("roboChragingStationValues",chargingStationValues,queue_size=1)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        valid_msg=True
        dis1=0.0
        dis2=0.0
        delta_x_robot=0.0
        meterPerPixel=0.0
        try:
          if math.fabs(token[0][0]-token[1][0]) != 0:
            dis1=(FIX_LENGTH_Y*FY)/(math.fabs(token[0][1]-token[1][1]))
          if math.fabs(token[2][0]-token[3][0]) != 0:
            dis2=(FIX_LENGTH_Y*FY)/(math.fabs(token[2][1]-token[3][1]))

          delta_dis=dis1-dis2
          yreal_m=math.sqrt(FIX_LENGTH_X*FIX_LENGTH_X - delta_dis*delta_dis)
          y_pixel = math.fabs(token[0][0]-token[2][0])
          print(yreal_m)
          print(y_pixel)
          if y_pixel != 0:
            meterPerPixel=yreal_m/y_pixel
          else:
            rospy.logwarn("ERROR Translation")
            valid_msg=False
            
          """AENDERUNG wegen Versetzen der IR Token Punkte
          pair_1_x=math.fabs(CX-token[0][0])
          pair_2_x=math.fabs(CX-token[2][0])
          delta_x_pixel=pair_1_x-pair_2_x
          delta_x_robot=(delta_x_pixel*meterPerPixel)/2000.0 #TODO:warum halbe (/1000 wegen umrechnung zu meter, rest rechnet anscheinend in mm)
          """
          delta_x_pixel=CX-token[2][0] #pixel left
          delta_x_robot=delta_x_pixel*meterPerPixel/1000.0
        except:
          rospy.logwarn("Distance calculation fail")
          valid_msg=False

        alpha=calc_alpha(dis1,dis2)
        print("Distance1: "+str(dis1)+" Distance2: "+str(dis2))
        print("Twist: "+str(math.degrees(alpha)))
        print("Tranlation X: "+str(delta_x_robot))
        msg=chargingStationValues()
        msg.distance=dis1
        msg.twist=alpha
        msg.translation=delta_x_robot
        msg.valid=valid_msg
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    listener()

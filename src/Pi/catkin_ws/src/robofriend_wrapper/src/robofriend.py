#!/usr/bin/env python

##### ROS STUFF
'''robofriend_wrapper ROS Node'''

################################################################## Imports
# ROS modules
import rospy
from std_msgs.msg import String, Bool, Float32
from roboFriendMsgs.msg import robofriendDrive
from roboFriendMsgs.msg import robofriendOdom
from roboFriendMsgs.msg import irCamData, irToken

# external modules
import signal
import time

# own modules
import python.faceModule as faceModule
import python.rfidModule as rfidModule
import python.webserverModule as webserverModule
import python.statusModule as statusModule
import python.gameCommunicator as gameCommunicator
import python.keyboardModule as keyboardModule
import python.teensyCommunicator as teensyCommunicator
import python.ioWarriorModule as ioWarriorModule
import python.speechModule as speechModule

#################################################################### Globals
runFlag = True
pubIrCamFlag = False

driveLeft = 0
driveRight = 0
odomLeft=0.0
odomRight=0.0
driveDuration=0
##################################################################### Functions
"""Teensy Communicator Manager

read pipe and send one msg and wait for response, ###############test if the teensy hangout come from the commincation
"""
def teensyManger(cmd):
  global driveLeft, driveRight, driveDuration, odomLeft, odomRight
  if cmd == "drive":
    teensyCommunicator.moveOdom(driveLeft, driveRight, driveDuration)
    # wait for response
  if cmd == "odom":
    odomLeft, odomRight = parser(teensyCommunicator.getOdom())

"""IR Token Parser

parse input string of IR Tokens to IR Token values and return ir Token msg
"""
def irTokenParser(string):
  data=string.split(",")
  irData = irCamData()
  for i in range (0,4):
    irData.tokens[i].x=int(data[i*4+2])
    irData.tokens[i].y=int(data[i*4+3])
    irData.tokens[i].size=int(data[i*4+4])
  return irData

""" Odom parser

parse input string of odometry to motor speed of left and right motor
"""
def parser(string):
  try:
    msg=string[5:]
    left, right = msg.split("r")
    c,msg=right.split("=")
    right,c=msg.split("\n")
    return float(left), float(right)
  except:
    left="0"
    right="0"
    print("ERROR Odom")
    return 0.0, 0.0

""" callback for robodrive msg

send drive command to microcontroller
"""
def callbackDrive(data):
  global driveLeft, driveRight, driveDuration
  rospy.loginfo("Drive:%d,%d,%d", data.left, data.right, data.duration)
  driveLeft=data.left
  driveRight=data.right
  driveDuration=data.duration

""" callback for IR Camera Flag

turn IR Cam on or off
"""
def callbackIrCam(data):
  global pubIrCamFlag
  pubIrCamFlag = data.data
  if pubIrCamFlag:
    rospy.loginfo("IrCam Data: ON")
  else:
    rospy.loginfo("IrCam Data: OFF")

""" callback for robospeak msg

send msg data to speech module
"""
def callbackSpeak(data):
  rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
  speechModule.speak(data.data)

""" stop function

stop all modules and shutdown the robofriend software
"""
def stop():
  global runFlag

  print "*** shutting down ... ***"
  #rfidModule.stop()
  webserverModule.stop()
  statusModule.stop()
  gameCommunicator.stop()
  keyboardModule.stop()
  teensyCommunicator.stop()
  ioWarriorModule.stop()
  speechModule.stop()
  faceModule.close()
  runFlag = False
  print "*** graceful shutdown completed! ***"

""" handler for the stop signal
"""
def handler_stop_signals(signum, frame):
  stop()

""" main function

start robofriend software modules and robofriend rosnode
"""
def main():
  global runFlag
  global pubIrCamFlag
  global driveLeft, driveRight, driveDuration, odomLeft, odomRight

  pub = rospy.Publisher('roboOdom', robofriendOdom, queue_size=10)
  pubIrCam = rospy.Publisher('roboIrToken', irCamData, queue_size=10)
  rospy.init_node('robofriend_wrapper', anonymous=True)
  rate = rospy.Rate(5) # 5Hz
  rospy.Subscriber("robospeak", String, callbackSpeak)
  rospy.Subscriber("robodrive", robofriendDrive, callbackDrive)
  rospy.Subscriber("roboIrCam", Bool, callbackIrCam)
  pubBat = rospy.Publisher("roboBattery", Float32, queue_size=2)
  # starting modules
  #rfidModule.start()
  webserverModule.start()
  statusModule.start()
  gameCommunicator.start()
  keyboardModule.start()
  faceModule.drawFace()
  print "init done! register signal handlers..."

  # setting up signal handlers for shutdown
  signal.signal(signal.SIGINT, handler_stop_signals)
  signal.signal(signal.SIGTERM, handler_stop_signals)
  print "*** startup completed! ***"

  while not rospy.is_shutdown() and runFlag:
    teensyManger("odom")
    odom = robofriendOdom()
    odom.left=int(odomLeft)
    odom.right=int(odomRight)
    pub.publish(odom)
    teensyManger("drive")

    try:
      batVolt=Float32()
      batVolt.data=float(statusModule.getBatteryVoltage())
      pubBat.publish(batVolt)
    except:
      print("ERROR batVolt")

    if pubIrCamFlag:
      irString=teensyCommunicator.sendSerial("IR",True)
      irCamMsg = irCamData()
      irCamMsg=irTokenParser(irString)
      pubIrCam.publish(irCamMsg) 

    rate.sleep()

if __name__ == '__main__':
  main()

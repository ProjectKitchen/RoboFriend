#!/usr/bin/env python

##### ROS STUFF
'''robofriend_wrapper ROS Node'''
# license removed for brevity
import rospy
from std_msgs.msg import String, Bool
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

# globals
runFlag = True
pubIrCam = False

def irTokenParser(string):
  data=string.split(",")
  
def parser(string):
  try:
    msg=string[5:]
    left, right = msg.split("r")
    c,msg=right.split("=")
    right,c=msg.split("\n")
  except:
    left="0"
    right="0"
    print("ERROR Odom")

  return float(left), float(right)

def callbackDrive(data):
  rospy.loginfo("Drive:%d,%d,%d", data.left, data.right, data.duration)
  teensyCommunicator.moveOdom(data.left, data.right, data.duration)

def callbackIrCam(data):
  global pubIrCam
  pubIrCam = data.data
  if pubIrCam:
    rospy.loginfo("IrCam Data: ON")
  else:
    rospy.loginfo("IrCam Data: OFF")

def callbackSpeak(data):
  '''robofriend_wrapper Callback Function'''
  rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
  speechModule.speak(data.data)

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

def handler_stop_signals(signum, frame):
  stop()

def main():
  global runFlag
  global 

  pub = rospy.Publisher('roboOdom', robofriendOdom, queue_size=10)
  pubIrCam = rospy.Publisher('roboIrToken', irCamData, queue_size=10)
  rospy.init_node('robofriend_wrapper', anonymous=True)
  rate = rospy.Rate(10) # 10hz
  rospy.Subscriber("robospeak", String, callbackSpeak)
  rospy.Subscriber("robodrive", robofriendDrive, callbackDrive)
  rospy.Subscriber("roboIrCam", Bool, callbackIrCam)

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

  #while runFlag: time.sleep(0.5) # keep program running until stopped
  while not rospy.is_shutdown() and runFlag:
    #hello_str = "hello world %s" % rospy.get_time()
    #rospy.loginfo(hello_str)
    #pub.publish(hello_str)
    odom = robofriendOdom()
    left, right = parser(teensyCommunicator.getOdom())
    #print("left: "+str(left)+" right: "+str(right)+"\n")
    odom.left=int(left)
    odom.right=int(right)
    pub.publish(odom)
    if pubIrCam:
      irString=teensyCommunicator.sendSerial("IR",True)
    rate.sleep()

if __name__ == '__main__':
  main()

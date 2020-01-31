#!/usr/bin/env python
""" robofriend_set_goal node

managed commands to drive to a given pre saved position (list is given in a text file)
"""

############################################################### IMPORTS
import rospy
from std_msgs.msg import String
import socket
from roboFriendMsgs.srv import setGoal, setGoalResponse

################################################################ GLOBALS
UDP_IP = "172.22.0.136"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

################################################################# Functions

""" robofirned set goal node

read positions text file and start the actionserver
"""
def robofriend_snips():
    global x,y,new_goal
    rospy.init_node('robofriend_snips', anonymous=False)
    rate = rospy.Rate(10)  
    ser_set_goal=rospy.ServiceProxy("/setGoal",setGoal)

    while not rospy.is_shutdown():
        data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
        print data
        data_ar=data.split(" ")
        location=data_ar[1]
        location=location.decode("utf-8")
        location=repr(location.replace(unichr(252), 'ue'))[2:-1]
        print location
        rospy.wait_for_service('/setGoal')
        resp=ser_set_goal(location)

        rate.sleep()

""" main function

start the ros node
"""
if __name__ == '__main__':
    try:
        robofriend_snips()
    except rospy.ROSInterruptException:
        pass


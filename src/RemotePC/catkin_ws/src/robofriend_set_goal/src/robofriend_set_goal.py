#!/usr/bin/env python
""" robofriend_set_goal node

managed commands to drive to a given pre saved position (list is given in a text file)
"""

############################################################### IMPORTS
import rospy
from std_msgs.msg import String

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from roboFriendMsgs.srv import setGoal, setGoalResponse

################################################################ GLOBALS
x = 0.0
y = 0.0
new_goal = False
dict_pos={}

################################################################# Functions

""" file parser function

parse the given text file and create a dictonary with the given position and position name
"""
def file_parser(file_path):
  global dict_pos
  print(file_path)
  try:  
    f=open(file_path)
  except:
    print("no valid Path")
    return -1
  while True:
    line=f.readline()
    if line == "":
        break
    if line[0]=='#':
        continue
    
    args=line.split(",")
    dict_pos[args[0]]= args[1:]
  f.close()

""" get value function

get the position values of the given position name
"""
def get_value(key,typ):
  value=dict_pos.get(key)
  print(value)
  if typ == "point":
    return float(value[0]),float(value[1]),-1,-1
  if typ == "rot":
    return float(value[2]),float(value[3]),float(value[4]),float(value[5])

""" get value function

get the position values of the given position name
"""
def parser(string):
    x,y,z,w = get_value(string,"point")  
    return x,y,True

"""set_goal

set a new goal for the nav stack
"""
def set_goal(x,y):
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()   

""" handler for set goal

set a new goal for the given position
return true if position is reached otherwise false
"""
def handle_set_goal(req):
    global x,y,new_goal
    if new_goal == False:
        x,y,valid = parser(req.goal)
        if valid:        
            new_goal = True
            rospy.loginfo("new goal is set")
            if new_goal==True:
                result = set_goal(x,y)          
            if result:
                rospy.loginfo("Goal execution done!")
                new_goal = False
                return setGoalResponse(True)
            else:
                return setGoalResponse(False)
        else:
            return setGoalResponse(False)

""" robofirned set goal node

read positions text file and start the actionserver
"""
def robofriend_set_goal():
    global x,y,new_goal
    rospy.init_node('robofriend_set_goal', anonymous=False)
    rate = rospy.Rate(10)
    s = rospy.Service("setGoal",setGoal,handle_set_goal)    

    file_parser("/home/adrian/GIT/RoboFriend/src/RemotePC/catkin_ws/src/robofriend_set_goal/src/positions.txt")
    while not rospy.is_shutdown():
        
        rate.sleep()

""" main function

start the ros node
"""
if __name__ == '__main__':
    try:
        robofriend_set_goal()
    except rospy.ROSInterruptException:
        pass


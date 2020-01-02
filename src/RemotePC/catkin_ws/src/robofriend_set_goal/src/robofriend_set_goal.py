#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import String

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from roboFriendMsgs.srv import setGoal, setGoalResponse

x = 0.0
y = 0.0
new_goal = False

####################################### Position Stuff ##
dict_pos={}

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
    #print(line)
    
    args=line.split(",")
    #print(args)
    #print(args[0])
    dict_pos[args[0]]= args[1:]
  f.close()

def get_value(key,typ):
  value=dict_pos.get(key)
  print(value)
  if typ == "point":
    return float(value[0]),float(value[1]),-1,-1
  if typ == "rot":
    return float(value[2]),float(value[3]),float(value[4]),float(value[5])

################################################################
def parser(string):
    x,y,z,w = get_value(string,"point")  
    return x,y,True


def set_goal(x,y):

   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
   # Move 0.5 meters forward along the x axis of the "map" coordinate frame 
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
   # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0

   # Sends the goal to the action server.
    client.send_goal(goal)
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   

def handle_set_goal(req):
    global x,y,new_goal
    if new_goal == False:
        x,y,valid = parser(req.goal)
        if valid:        
            new_goal = True
            rospy.loginfo("new goal is set")
            if new_goal==True:
            #set new goal and send it to navStack
                result = set_goal(x,y)          
            if result:
                #goal is reached
                rospy.loginfo("Goal execution done!")
                new_goal = False
                return setGoalResponse(True)
            else:
                return setGoalResponse(False)
        else:
            return setGoalResponse(False)

def robofriend_set_goal():
    global x,y,new_goal
    #sub = rospy.Subscriber('robo_set_goal', String, callback)
    rospy.init_node('robofriend_set_goal', anonymous=False)
    rate = rospy.Rate(10)
    s = rospy.Service("setGoal",setGoal,handle_set_goal)    

    file_parser("/home/adrian/GIT/RoboFriend/src/RemotePC/catkin_ws/src/robofriend_set_goal/src/positions.txt")
    while not rospy.is_shutdown():
        
        rate.sleep()

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        robofriend_set_goal()
    except rospy.ROSInterruptException:
        pass


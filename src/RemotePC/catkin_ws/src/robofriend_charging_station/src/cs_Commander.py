#!/usr/bin/env python
'''calcCahrignStationPos ROS Node'''
import rospy
from std_msgs.msg import String, Bool
from roboFriendMsgs.srv import driveInChargingStation, driveInChargingStationResponse
from roboFriendMsgs.srv import setGoal, setGoalResponse

'''
publisher uer /roboIrCam , bool, true (IR CAM aktivieren)
publisher robo_set_goal, String, "CS" (zu Ladestationfahren)
serviceCaller driveInStation, driveInChargingStation, True, RESPONSE should True, otherwise not in station

subscriber /driveInCS, bool
'''
new_com = False

def callback(data):
    global new_com
    if new_com == False:
        new_com=data.data
        print("Recive Com to drive in Charging STation\n")


def commander():
    global new_com
    rospy.init_node('cs_commander', anonymous=True)
    rate=rospy.Rate(10)

    
    rospy.Subscriber("driveInCS", Bool, callback)
    pubIrCam=rospy.Publisher("/roboIrCam",Bool,queue_size=1)
    ser_set_goal=rospy.ServiceProxy("/setGoal",setGoal)
    ser_driveIn=rospy.ServiceProxy('/driveInStation', driveInChargingStation)
    while not rospy.is_shutdown(): 
        if new_com == True:
            print("Start Driving to CS\n")
            rospy.wait_for_service('/setGoal')
            resp=ser_set_goal("CS")
            print(resp.execute)
            if resp.execute == True:
                pubIrCam.publish(True)
                rospy.wait_for_service('/driveInStation')
                print("Drive in CS\n")            
                resp1 = ser_driveIn(True)
                print(resp1)
                if resp1==True:
                    print("In CS\n\n")
                    new_com = False
                else:
                    print("Drive in CS not possible\n")
                    new_com = False
            else:
                print("Drive to CS not possible\n") 
                new_com = False   
        else:
            rate.sleep()

if __name__ == '__main__':
    commander()

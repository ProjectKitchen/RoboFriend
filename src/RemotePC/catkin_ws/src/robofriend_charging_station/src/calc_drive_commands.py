#!/usr/bin/env python
'''calcCahrignStationPos ROS Node'''
import rospy
#import cv2
import numpy as np
from std_msgs.msg import String
from roboFriendMsgs.msg import irCamData, chargingStationValues
from roboFriendMsgs.srv import driveInChargingStation, driveInChargingStationResponse
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


#values in SI-Einheit
TURN_SPEED = 0.4
FORWARD_SPEED=-0.09
END_DISTANCE_INFRONT_OF_CHARGINGSTATION = 0.4
beta=0.0
irTokenFound=False
covered_distance=0.0
covered_twist=0.0

twist_to_correct=0.0

new_callback = False

global_charging_data= chargingStationValues()
pub=rospy.Publisher("cmd_vel",Twist,queue_size=1)   


def search_ir_Tokens():
    global global_charging_data
    print("search Token")
    turn_msg=Twist()
    irTokenFound = global_charging_data.valid
    if irTokenFound==False :
        turn_msg.linear.x=0.0
        turn_msg.linear.y=0.0
        turn_msg.linear.z=0.0
        turn_msg.angular.x=0.0
        turn_msg.angular.y=0.0
        turn_msg.angular.z=TURN_SPEED
        start=rospy.get_time()
        pub.publish(turn_msg)
        print("positiv Turn")
        while True:
            if global_charging_data.valid:
                irTokenFound=True
                break
            if rospy.get_time() - start > 1.0:
                turn_msg.angular.z=0.0
                pub.publish(turn_msg)
                rospy.sleep(0.5)
                turn_msg.angular.z=-TURN_SPEED
                start=rospy.get_time()
                pub.publish(turn_msg)
                print("back to start")
                while rospy.get_time() - start < 1.0:
                    pass;
                break
        turn_msg.angular.z=0.0
        pub.publish(turn_msg)
        rospy.sleep(0.5)
        if irTokenFound==False:
            turn_msg.angular.z=-TURN_SPEED
            start=rospy.get_time()
            pub.publish(turn_msg)
            print("negativ Turn")
            while True:
                if global_charging_data.valid:
                    irTokenFound=True
                    break
                if rospy.get_time() - start > 1.0:
                    turn_msg.angular.z=0.0
                    pub.publish(turn_msg)
                    rospy.sleep(0.5)
                    turn_msg.angular.z=TURN_SPEED
                    start=rospy.get_time()
                    pub.publish(turn_msg)
                    while rospy.get_time() - start < 1.0:
                        pass;
                    break
        turn_msg.angular.z=0.0
        pub.publish(turn_msg)

timestamp_old=0.0
delta_dis=0.0
timestamped_old=0.0
delta_turn=0.0
def callback_odom(data):
    global covered_distance,covered_twist,old_timestamp, global_charging_data, delta_dis,timestamped_old,delta_turn
    timestamp_new = rospy.get_time()
    covered_distance=data.twist.twist.linear.x*(timestamp_new-timestamped_old)
    covered_twist = data.twist.twist.angular.z*(timestamp_new-timestamped_old)
    timestamped_old=timestamp_new
    delta_dis+=covered_distance
    delta_turn+=covered_twist
    #print(delta_dis)
    #print(timestamp_new-timestamped_old)

def callback(data):
    global global_charging_data, new_callback
    #if new_callback != True:
    global_charging_data.distance=data.distance
    global_charging_data.twist=data.twist
    global_charging_data.translation=data.translation
    global_charging_data.valid=data.valid
    #new_callback = True

def calc_values():
    global beta, global_charging_data
    trans_y = global_charging_data.translation
    trans_x = global_charging_data.distance - END_DISTANCE_INFRONT_OF_CHARGINGSTATION*1000
    alpha = global_charging_data.twist
    forward_path=math.sqrt(trans_y*trans_y + trans_x*trans_x)
    
    phi=math.atan2(trans_y,trans_x)
    
    if alpha > phi:
        beta=-(alpha-phi)
    else:
        beta=-(phi-alpha)

    return forward_path,beta

def turn(twist):
    global global_charging_data,delta_turn
    turn_time=math.fabs(twist/TURN_SPEED)
    #print(twist, TURN_SPEED)
    #print(turn_time)
    turn_msg=Twist()
    turn_msg.linear.x=0.0
    turn_msg.linear.y=0.0
    turn_msg.linear.z=0.0
    turn_msg.angular.x=0.0
    turn_msg.angular.y=0.0
    if twist > 0.0:
        turn_msg.angular.z=TURN_SPEED
    else:
        turn_msg.angular.z=-TURN_SPEED

    pub.publish(turn_msg)
    delta_turn=0.0
    if twist > 0.0:
        while delta_turn<twist:
            pass;
    else:
        while delta_turn > twist:
            pass;

    turn_msg.linear.x=0.0
    turn_msg.linear.y=0.0
    turn_msg.linear.z=0.0
    turn_msg.angular.x=0.0
    turn_msg.angular.y=0.0
    turn_msg.angular.z=0.0
    pub.publish(turn_msg)
    


def drive_forward(path, forward=True):
    global covered_distance,covered_twist,twist_to_correct, global_charging_data, delta_dis 
    #covered_twist=0.0
    #drive_time = math.fabs((path/1000)/FORWARD_SPEED)
    #print(drive_time)
    turn_msg=Twist()
    if forward:
        turn_msg.linear.x=FORWARD_SPEED
        path=-path
    else:
        turn_msg.linear.x=-FORWARD_SPEED
    turn_msg.linear.y=0.0
    turn_msg.linear.z=0.0
    turn_msg.angular.x=0.0
    turn_msg.angular.y=0.0
    turn_msg.angular.z=0.0
    pub.publish(turn_msg)
    #rospy.sleep(drive_time)
    
    delta_dis=0.0
    if forward:
        while delta_dis>(path/1000):
          pass;
    else:
        while delta_dis<(path/1000):
          pass;

    turn_msg.linear.x=0.0
    turn_msg.linear.y=0.0
    turn_msg.linear.z=0.0
    turn_msg.angular.x=0.0
    turn_msg.angular.y=0.0
    turn_msg.angular.z=0.0
    pub.publish(turn_msg)
    twist_to_correct=covered_twist

def drive_in_charging_station(dis):
    global global_charging_data
    drive_time = math.fabs((dis/1000)/FORWARD_SPEED)
    print(dis,FORWARD_SPEED)
    print(drive_time)
    turn_msg=Twist()
    turn_msg.linear.x=FORWARD_SPEED
    turn_msg.linear.y=0.0
    turn_msg.linear.z=0.0
    turn_msg.angular.x=0.0
    turn_msg.angular.y=0.0
    turn_msg.angular.z=0.0
    pub.publish(turn_msg)
    rospy.sleep(drive_time)

    turn_msg.linear.x=0.0
    turn_msg.linear.y=0.0
    turn_msg.linear.z=0.0
    turn_msg.angular.x=0.0
    turn_msg.angular.y=0.0
    turn_msg.angular.z=0.0
    pub.publish(turn_msg)
    twist_to_correct=covered_twist

def handle_drive_in_Station(req):
    print("\nDrive in Station start")
    global twist_to_correct, global_charging_data, irTokenFound
    rate = rospy.Rate(1)
    if req.start==True:
        counter=0
        while True:
            search_ir_Tokens()
            if irTokenFound==False:
                counter+=1
                if counter > 2:
                    break 
                    '''need controll why irTokenFound never true'''
                    return driveInChargingStationResponse(False)
                    
            else:
                break
  
        counter=0
        while True:
            trans_y = global_charging_data.translation
            trans_x = global_charging_data.distance - END_DISTANCE_INFRONT_OF_CHARGINGSTATION*1000
            path,twist=calc_values()  
            print("Turn"+str(twist))
            if counter > 5:
                return driveInChargingStationResponse(False) 
            elif twist > 1.0 or twist < -1.0:
                counter+=1
                
            elif path > 5000:
                counter+=1        
            else:
                break

        print("Drive to 1st Position"+str(path))
        turn(twist)
        rospy.sleep(0.5)
        drive_forward(path)
        rospy.sleep(0.5)
        turn(-twist)
        rospy.sleep(0.5)

        
        search_ir_Tokens()
        if irTokenFound == True:
            rospy.sleep(0.5)
            #path,twist=calc_values()
            print("Twist: "+str(global_charging_data.translation))
            #turn(-global_charging_data.twist)
            if twist > 1.0 or global_charging_data.translation < -1.0:
                print("NO Last Turn")#return driveInChargingStationResponse(False)
            else:
                turn(global_charging_data.translation)
                rospy.sleep(0.5)   
        print("drive in Satation:"+str(global_charging_data.distance-100))
        if global_charging_data.distance-100.0 > 0.0:          
          drive_forward(global_charging_data.distance-100.0)
        else:
          drive_forward(300.0)
        print("In Station\n")
        
        return driveInChargingStationResponse(True)        

def listener():
    global new_callback,covered_distance,covered_twist,timestamp_old
    global twist_to_correct
    rospy.init_node('driveInStation', anonymous=False)
    rospy.Subscriber("roboChragingStationValues", chargingStationValues, callback)
    rospy.Subscriber("odom", Odometry, callback_odom)
    s = rospy.Service("driveInStation",driveInChargingStation,handle_drive_in_Station)

    timestamp_old=rospy.get_time()
    rate = rospy.Rate(1)
    rospy.spin()
    
if __name__ == '__main__':
    listener()

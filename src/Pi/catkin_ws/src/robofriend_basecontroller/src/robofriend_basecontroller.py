#!/usr/bin/env python
'''robofriend_basecontroller ROS Node'''
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
from roboFriendMsgs.msg import robofriendDrive

vl=0.0
vr=0.0
width_robot=0.33

def callback(data):
    global vl
    global vr

    rospy.loginfo("CALLBACK")
    twist = Twist()
    twist = data;
    vel_x = twist.linear.x;
    vel_th = twist.angular.z;
    right_vel = 0.0;
    left_vel = 0.0;

    if vel_x == 0 :
        # turning
        right_vel = vel_th * width_robot / 2.0;
        left_vel = (-1) * right_vel;
    elif vel_th == 0 :
        #forward / backward
        left_vel = right_vel = vel_x;
    else:
        #moving doing arcs
        left_vel = vel_x - vel_th * width_robot / 2.0;
        right_vel = vel_x + vel_th * width_robot / 2.0;
    
    vl = left_vel*1000;
    vr = right_vel*1000;
    

def base_controller():
    global vl
    global vr
    '''robofriend_basecontroller Publisher'''
    pub = rospy.Publisher('robodrive',robofriendDrive , queue_size=10)
    sub = rospy.Subscriber("/cmd_vel", Twist, callback)

    rospy.init_node('robofriend_basecontroller', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        rospy.loginfo("Float %f,%f", vr,  vl)
        msg = robofriendDrive()
        msg.left = int( vl)
        msg.right = int( vr)
        msg.duration=0
        rospy.loginfo("Send %d,%d", msg.left, msg.right)
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        base_controller()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python

'''robofriend_odom ROS Node

subcribe to Odomdata of left and right motor and create odometry nav msg out of it
'''

######################################################################## IMPORTS
import rospy
#from std_msgs.msg import String
from roboFriendMsgs.msg import robofriendOdom
from nav_msgs.msg import Odometry

import math
from math import sin, cos, pi

import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

######################################################################### Globals
left_odom=0.0   # leftmotor odom data
right_odom=0.0 # rightmotor odom data
width_robot=0.33 # wheelbase of robofriend

######################################################################## Functions

""" callback for incoming roboOdom msg

save incoming data to global values
"""
def callback(odom_data):
    global left_odom
    global right_odom

    left_odom = odom_data.left/1000.0
    right_odom = odom_data.right/1000.0

""" robofriend_odom function

create ros node
subcribe to odomdata of the motors (coming from microcontroller) and published nav_msgs odometry on /odom topic
"""
def robofriend_odom():
    global left_odom
    global right_odom

    '''robofriend_odom Publisher'''
    sub = rospy.Subscriber('roboOdom', robofriendOdom, callback)
    pub = rospy.Publisher('odom',Odometry,queue_size=5)
    rospy.init_node('robofriend_odom', anonymous=False)
    odom_broadcaster = tf.TransformBroadcaster()

    rate = rospy.Rate(30) # 10hz

    # initial position  
    x = 0.0
    y = 0.0
    th = 0.0

    vx = 0.0
    vy = 0.0
    vth = 0.0

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        vx=(left_odom+right_odom)/2.0
        vth= ((right_odom - left_odom)/width_robot)

        dt = (current_time - last_time).to_sec()
        delta_x = (vx * cos(th) - vy * sin(th)) * dt
        delta_y = (vx * sin(th) + vy * cos(th)) * dt
        delta_th = vth * dt

        x += delta_x
        y += delta_y
        th += delta_th

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        # first, we'll publish the transform over tf
        odom_broadcaster.sendTransform(
            (x, y, 0.),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        # publish the message
        pub.publish(odom)

        last_time = current_time
        #rospy.sleep(0.3)
	rate.sleep()
       
""" main function

start robofriend_odom Node
"""
if __name__ == '__main__':
    try:
        robofriend_odom()
    except rospy.ROSInterruptException:
        pass

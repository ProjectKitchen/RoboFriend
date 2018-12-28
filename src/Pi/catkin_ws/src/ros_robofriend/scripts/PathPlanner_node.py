#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from robofriend.msg import LandmarkDistance
from turtlesim.msg import Pose

def distance(x1, y1, x2, y2):
    xd = x1 - x2
    yd = y1 - y2
    return math.sqrt(xd*xd + yd*yd)

class LandmarkMonitor(object):
    def __init__(self, pub, landmarks):
        self._pub = pub
        self._landmarks = landmarks

    def path_data_cb(self, data):
        x = data.x
        y = data.y

        closest_name = None
        closest_distance = None

        for l_name, l_x, l_y in self._landmarks:
            dist = distance(x, y, l_x, l_y)
            if closest_distance is None or dist < closest_distance:
                closest_name = l_name
                closest_distance = dist

        ld = LandmarkDistance()
        ld.name = closest_name
        ld.distance = closest_distance
        self._pub.publish(ld)

        if closest_distance < 0.5:
            rospy.loginfo(rospy.get_caller_id() + " i\'m near the {}".format(closest_name))
    
def main():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'pathplanner' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('pathplanner', anonymous=True)

    pub_c = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    pub_l = rospy.Publisher('/robofriend/closest_landmark', LandmarkDistance, queue_size = 10) # landmark data
    
    landmarks = []
    landmarks.append(("Cube", 1.31, 1.99));
    landmarks.append(("Dumpster", 2.11, 2.42));
    landmarks.append(("Cylinder", 3.14, 3.88));
    landmarks.append(("Barrier", 4.59, 4.83));
    landmarks.append(("Bookshelf", 5.09, 5.53));

    monitor = LandmarkMonitor(pub_l, landmarks)
    rospy.Subscriber("/turtle1/pose", Pose, monitor.path_data_cb)

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

    msg = Twist()
    l_x = 0.5
    nrel = 0
    a_z = 0.3
    msg.linear.x = l_x
    msg.linear.y = nrel
    msg.linear.z = nrel
    msg.angular.x = nrel
    msg.angular.y = nrel
    msg.angular.z = a_z

    fwd = 1

    rate = rospy.Rate(5) # 1hz
    while not rospy.is_shutdown():

        pub_c.publish(msg)

        if fwd is 1:
        	# msg.linear.x += 0.1
        	# msg.angular.z += 0.1
        	if msg.angular.z >= 1:
        		fwd = 0
        else:
        	# msg.linear.x -= 0.1
        	# msg.angular.z -= 0.1
        	if msg.angular.z <= 0:
        		fwd = 1

        rate.sleep() # make sure the publish rate maintains at the needed frequency

if __name__ == '__main__':
    main()
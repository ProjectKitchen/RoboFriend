#!/usr/bin/env python
import rospy
from robofriend.msg import PCBSensorData

# import ros modules
from PCBSensorDataHandler import *

def shutdown():
    rospy.signal_shutdown("Stopping PCB Sensor Data Handler node!")

def main():
    rospy.loginfo("Starting PCB Sensor Data Handler node!")
    rospy.init_node('robofriend/pcb_sensor_data', anonymous = True)

    pub = rospy.Publisher("/robofriend/pcb_sensor_data", PCBSensorData, queue_size = 2)

    dh = PCBSensorDataHandler(pub)

    rate = rospy.Rate(1) # 1hz

    while not rospy.is_shutdown():
        dh.requestSensorValues()
        rate.sleep() # make sure the publish rate maintains at the needed frequency

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
import rospy
import TeensyDataHandler
import threading

# import ros service
from ros_robofriend.srv import BatInfData

# import ros messages
from ros_robofriend.msg import TeensyMotorData

def node_start():
    print("[INFO] ROS Teensy Communicator Node started")

    try:
        teensy_serial = serial.Serial("/dev/ttyACM0", 9600, timeout = 1)
        print("[INFO] Serial for Teensy opened!")
    except:
        print("[INFO] Serial for Teensy could not opened!")
        serial = None

    teensy = TeensyDataHandler.TeensyDataHandler(serial)

    # declare service
    serv = rospy.Service('S_BAT_INF_DATA', BatInfData, teensy.service_handler)

    # declare Subscriber callback
    rospy.Subscriber("T_TEENSY_MOTOR_DATA", TeensyMotorData, teensy.motor_process_data)

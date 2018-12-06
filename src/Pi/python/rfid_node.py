import serial
import threading
import gameCommunicator
from std_msgs.msg import String
import rospy

# global variables
runFlag = True

def node_stop():
    global runFlag
    runFlag = False

def node_start():
    global runFlag

    print("[INFO] ROS RFID Node started!\n")

    pub = rospy.Publisher('T_RFID_DATA', String, queue_size = 10)

    try:
        serial_rfid = serial.Serial("/dev/ttyUSB0", 9600)
        print("[INFO] Serial for RFID reader opened!")

        # thread to handle the rfid date
        rfid_thread = threading.Thread(
            target = serial_rfid_read,
            args = (serial_rfid, pub,  )
        )

        # start rfid thread
        rfid_thread.start()
    except:
        print("[INFO] Serial for RFID could not be opened!!!")

        if runFlag == False:
            serial_rfid.close()

def serial_rfid_read(serial, pub):
    global runFlag

    data = ""
    readRFIDnumber = None

    try:
        while runFlag:
            data = str(serial.read(16))
            data = data.strip("b'")
            data = data.replace("\\x02", "").replace("\\x03", "").replace("\\x0a", "").replace("\\x0d", "").replace("\\r\\n", "")
            #data = data.replace("\x02", "" )
            #data = data.replace("\x03", "" )
            #data = data.replace("\x0a", "" )
            #data = data.replace("\x0d", "" )
            readRFIDnumber = data

            if readRFIDnumber != "empty":
                print("[INFO] Published RFID message: {}".format(readRFIDnumber))
                #queue.put(readRFIDnumber)
                pub.publish(readRFIDnumber)
                readRFIDnumber = "empty"
    finally:
        serial.close()

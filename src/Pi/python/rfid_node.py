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
    print("[INFO] Stopping rfid node!")

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

        # set thread as a daemon
        rfid_thread.daemon = True 
         
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
            
            readRFIDnumber = data

            if readRFIDnumber != "empty":
                print("[INFO] Published RFID message: {}".format(readRFIDnumber))
                pub.publish(readRFIDnumber)
                readRFIDnumber = "empty"      
        print("[INFO] END SERIAL RFID THREAD")
    finally:
        serial.close()

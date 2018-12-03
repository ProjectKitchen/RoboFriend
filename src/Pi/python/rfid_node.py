import serial
import threading
import gameCommunicator
from std_msgs import String

# global variables
runFlag = True

def node_stop():
    global runFlag
    runFlag = False

def node_start():
    global runFlag

    received_message = ""

    print("[INFO] ROS RFID Node started!\n")

    pub = rospy.Publisher('rfid_topic', String, queue_size = 10)
    #rospy.init('rfid_node', anonymous = True)

    # queue to ensure a communication system between rfid thread and publisher
    #thread_queue = queue.Queue()

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

        # while runFlag:
        #     received_messsage = thread_queue.get()
        #     transmit_message = message_merge(received_message)
        #     print("[INFO] Transmitted data from rfid node: {}".format(transmit_message))
        #     game.Communicator(transmit_message))
        #     pub.publish(transmit_message)
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
            data = serial.read(16)
            data = data.replace("\x02", "" )
            data = data.replace("\x03", "" )
            data = data.replace("\x0a", "" )
            data = data.replace("\x0d", "" )
            readRFIDnumber = data

            if readRFIDnumber != "empty":
                print("[INFO] Published RFID message: {}".format(readRFIDnumber))
                #queue.put(readRFIDnumber)
                pub.publish(readRFIDnumber)
                readRFIDnumber = "empty"
    finally:
        serial.close()

def message_merge(command):
    return "rfid;" + command

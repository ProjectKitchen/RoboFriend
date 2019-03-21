#!/usr/bin/env python
import rospy, socket, sys

# import user modules
import constants

# import ros messages
from robofriend.msg import PCBSensorData

# import ros services
from robofriend.srv import SrvRFIDData
from robofriend.srv import SrvTeensySerialData
from robofriend.srv import SrvFaceDrawData

# globals
IP = ''
UDP_PORT = 9000 # socket port
UDP_SOCKET = None
BAT_VOLT = 0
TEENSY_SRV_REQ = None
face_req = None

def dataListener():
    """ 
    this function is used as a pseudo thread to always listen if there is a client (gamegui) sending commands
    """
    global IP
 
    startFlag = ":RUN:"
    endFlag = ":EOL:"
    receivedData = ""
    
    try: 
        tempData, addr = UDP_SOCKET.recvfrom(50)
    except Exception as inst:
        # we want to have debug level here since we have a non-blocking socket which 
        # will make the socket always 'temporarily  unavaiable' in the loop 
        rospy.logdebug('{%s} - this is a controlled catch.', rospy.get_caller_id())
        rospy.logdebug('{%s} - could not receive data from udp socket.', rospy.get_caller_id())
        rospy.logdebug('{%s} - exception type: %s', rospy.get_caller_id(), type(inst))
        rospy.logdebug('{%s} - exception argument: %s', rospy.get_caller_id(), inst.args[1])
        return
        
    receivedData += tempData.decode("utf-8")
    startFlagIndex = receivedData.find(startFlag)
    if startFlagIndex != -1:
        receivedData = receivedData[startFlagIndex + len(startFlag):]
 
    endFlagIndex = receivedData.find(endFlag)
    if endFlagIndex != -1:
        receivedData = receivedData[:endFlagIndex]
    else:
        rospy.logwarn("{%s} - expected string format: \":RUN:xxx:EOL:\", your string: %s", 
                      rospy.get_caller_id(), receivedData)
        return
 
    # make sure we answer the caller
    IP = addr[0]
    rospy.loginfo("{%s} - received data: \"%s\", from app: %s", 
                  rospy.get_caller_id(), receivedData, str(IP))
    chooseAction(receivedData)

def chooseAction(data):
    global currentStatus
    dataArray = data.split(';') # Nachricht wird in ein Array gespeichert
    action = dataArray[0] # erstes Argument
    dataArray = dataArray[1:] # restliche Argumente
    if action == "move":
        # echo -n ":RUN:move;forward:EOL:" >/dev/udp/localhost/9000
        # echo -n ":RUN:move;forward;backward:EOL:" >/dev/udp/localhost/9000
        move(dataArray)
    # ZAHEDIM: adapt when merging codes
    # MOMOKARL: how do we solve this?
    # best way would be by implementing a service
    # see the example above with move
#     elif action == "say":
#         speechModule.speak(dataArray[0])
#     elif action == "sound":
#         info = dataArray[0]
#         dataArray = dataArray[1:]
#         if info == "play":
#             soundModule.playsound(dataArray)
    elif action == "face":
        face_manipulation(dataArray)
    elif action == "get":
        info = dataArray[0]
        # echo -n ":RUN:get;status:EOL:" >/dev/udp/localhost/9000
        if info == "status" and BAT_VOLT > 0:
            sendToGUI("battery;" + str(round(BAT_VOLT, 2)))
    elif action == "IPcheck":
        pass

def face_manipulation(array):
    global TEENSY_SRV_REQ
    face_object = ["smile", "eyes"]
    param = []
    
    face_ob, command = array[:2]
    if face_ob in face_object:
        face_node_service_request(command, param)
    elif face_ob in "answer":
        if command in "correct":
            param = [60]
            face_node_service_request(constants.SET_SMILE, param)
        elif command in "wrong":
            param = [-60]
            face_node_service_request(constants.SET_SMILE, param)
    else:
        rospy.logwarn("{%s} - wrong command / face_manipulation", 
                    rospy.get_caller_id())
    TEENSY_SRV_REQ = 'H'
            
def face_node_service_request(action = "", param = []):
    global face_req
    response = None
    
    response = face_req(action. param)
    if response.resp:
        rospy.logdebug("{%s} - Successfull response from Face Node",
                    rospy.get_caller_id())
    else:
        rospy.logwarn("{%s} - Erroneous response", 
                    rospy.get_caller_id())
    

def move(dataArray):
    global TEENSY_SRV_REQ
    rospy.logdebug("{%s} - received move array: %s", rospy.get_caller_id(), dataArray)
    dir = dataArray[0]
    # step informationen vorhanden, tablet toucheingabe verwendet
    if len(dataArray) > 1:
        # TODO: fuer spaeter hier erweiterung moeglich,
        # auf laenge der steps eingehen, jetzt nur standardwert verwendet
        # dataArray = dataArray[1:]
        # step = dataArray[0] 
        options = {'forward':       constants.MOVE_STEP_FWD,
                   'right':         constants.MOVE_STEP_RYT,
                   'backward':      constants.MOVE_STEP_BCK,
                   'left':          constants.MOVE_STEP_LFT
                   }
        if dir in options:
            TEENSY_SRV_REQ = options[dir]
    # keine step informationen vorhanden, daher loop, joystick verwendet
    else: 
        options = {'forward':       constants.MOVE_LOOP_FWD,
                   'right':         constants.MOVE_LOOP_RYT,
                   'backward':      constants.MOVE_LOOP_BCK,
                   'left':          constants.MOVE_LOOP_LFT,
                   'forward_right': constants.MOVE_LOOP_FWD_RYT,
                   'forward_left':  constants.MOVE_LOOP_FWD_LFT,
                   'backward_right':constants.MOVE_LOOP_BCK_RYT,
                   'backward_left': constants.MOVE_LOOP_BCK_LFT,
                   'stop':          constants.STOP_MOVING
                   }
        if dir in options:
            TEENSY_SRV_REQ = options[dir]
            
def provideBatteryVoltage(args):
    if args.voltage <= 0:
        return
    global BAT_VOLT
    BAT_VOLT = args.voltage
    sendToGUI("battery;" + str(round(BAT_VOLT, 2)))

def provideRFIDNumber(args):
    if args is None:
        return 
    if args.data is '':
        return
    
    sendToGUI("rfid;" + str(args.data))

def sendToGUI(data):
    """ 
    this function is used to send information to the gamegui and can be called 
    by the the methode provideRFIDNumber (for RFID data) and chooseAction (for battery information)
    """
    global IP, UDP_PORT, UDP_SOCKET

    # init if not already initialized
    # start()

    if IP == '':
        return
    bytesToSend = bytes(":RUN:" + str(data) + ":EOL:")
    rospy.logdebug("{%s} - sending data to gui: %s", rospy.get_caller_id(), bytesToSend)
    try:
        UDP_SOCKET.sendto(bytesToSend, (IP, UDP_PORT + 1))
    except OSError as inst:
        IP = ''
        rospy.logwarn('{%s} - this is a controlled catch.', rospy.get_caller_id())
        rospy.logwarn('{%s} - failed to send data via udp socket.', rospy.get_caller_id())
        rospy.logwarn('{%s} - exception type: %s', rospy.get_caller_id(), type(inst))
        rospy.logwarn('{%s} - exception argument: %s', rospy.get_caller_id(), inst.args[1])

def shutdown():
    if UDP_SOCKET is not None:
        UDP_SOCKET.close()
    rospy.loginfo("{%s} - stopping game communicator node.", rospy.get_caller_id())
    rospy.signal_shutdown("controlled shutdown.")

def GameCommunicator():
    global UDP_SOCKET, TEENSY_SRV_REQ, face_req
    
    rospy.init_node("robofriend_game_communicator", log_level = rospy.INFO)
    rospy.loginfo("{%s} - starting game communicator handler node!", rospy.get_caller_id())
    rospy.on_shutdown(shutdown)
    
    rospy.Subscriber("/robofriend/pcb_sensor_data", PCBSensorData, provideBatteryVoltage)
    
    # create service to communicate with face node
    face_req = rospy.ServiceProxy('/robofriend/face', SrvFaceDrawData)
    
    UDP_IP = ''
    # handle commandline arguments to get ip address
    if len(sys.argv) == 2:
        try:
            # check for a valid input
            UDP_IP = sys.argv[1]
            socket.inet_aton(UDP_IP)
        except Exception as inst:
            rospy.logwarn('{%s} - this is a controlled catch.', rospy.get_caller_id())
            rospy.logwarn('{%s} - invalid ip address (\'%s\'), try again.', rospy.get_caller_id(), UDP_IP)
            rospy.logwarn('{%s} - exception type: %s', rospy.get_caller_id(), type(inst))
            rospy.logwarn('{%s} - exception argument: %s', rospy.get_caller_id(), inst.args[0])
            sys.exit()
    else:
        UDP_IP = ''
    
    # connect the udp port
    try: 
        UDP_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
        UDP_SOCKET.bind((UDP_IP, UDP_PORT))
        UDP_SOCKET.setblocking(0)
        rospy.loginfo("{%s} - udp socket opened.", rospy.get_caller_id())
    except Exception as inst:
        rospy.logwarn('{%s} - this is a controlled catch.', rospy.get_caller_id())
        rospy.logwarn('{%s} - could not open udp socket.', rospy.get_caller_id())
        rospy.logwarn('{%s} - exception type: %s', rospy.get_caller_id(), type(inst))
        rospy.logwarn('{%s} - exception argument: %s', rospy.get_caller_id(), inst.args[1])

    rate = rospy.Rate(1) # 1hz    
    
    while not rospy.is_shutdown():
        # listen to udp port
        dataListener()
        
        # get data from rfid reader
        rfid_srv_resp = None
        rospy.wait_for_service('/robofriend/get_rfid_number')
        try:
            request = rospy.ServiceProxy('/robofriend/get_rfid_number', SrvRFIDData)
            rfid_srv_resp = request(True)
        except rospy.ServiceException:
            rospy.logwarn("{%s} - service call failed. check the rfid serial data.", rospy.get_caller_id())
        
        # process the response
        provideRFIDNumber(rfid_srv_resp)
        
        # send data to teensy per service request param
        if TEENSY_SRV_REQ is not None:
            rospy.wait_for_service('/robofriend/teensy_serial_data')
            try:
                request = rospy.ServiceProxy('/robofriend/teensy_serial_data', SrvTeensySerialData)
                request(TEENSY_SRV_REQ, False)
                TEENSY_SRV_REQ = None
            except rospy.ServiceException:
                rospy.logwarn("{%s} - service call failed. check the teensy serial data.", rospy.get_caller_id())
        
        rate.sleep() # make sure the publish rate maintains at the needed frequency
        
if __name__ == '__main__':
    try:
        GameCommunicator()
    except rospy.ROSInterruptException:
        pass

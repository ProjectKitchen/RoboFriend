#!/usr/bin/env python
import rospy, socket, sys

# import ros services
from robofriend.srv import SrvRFIDData

# globals
IP = ''
UDP_IP = ''
UDP_PORT = 9000 # socket port
UDP_SOCKET = None

""" 
This function is used to send information to the gamegui and can be called 
by the Thread serialRFIDread (for RFID data) and chooseAction (for battery information)
"""

def provideRFIDNumber(args):
    if args is None:
        return 
    
    rospy.logdebug("{%s} - sending rfid number to gui: %s", rospy.get_caller_id(), args.rfid_number)
    # sendToGUI("rfid;" + str(args.data))

def sendToGUI(data):
    global IP, UDP_PORT, UDP_SOCKET

    # init if not already initialized
    # start()

    if IP == '':
        return
    bytesToSend = bytes(":RUN:" + str(data) + ":EOL:")
    try:
        UDP_SOCKET.sendto(bytesToSend, (IP, UDP_PORT + 1))
    except OSError as inst:
        IP = ''
        rospy.logwarn('{%s} - this is a controlled catch.', rospy.get_caller_id())
        rospy.logwarn('{%s} - failed to send data via udp socket.', rospy.get_caller_id())
        rospy.logwarn('{%s} - exception type: %s', rospy.get_caller_id(), type(inst))
        rospy.logwarn('{%s} - exception argument: %s', rospy.get_caller_id(), inst.args[1])

""" 
This function is used as Thread to always listen if there is a client (gamegui) sending commands
"""
def data_listener():
    global IP
 
    startFlag = ":RUN:"
    endFlag = ":EOL:"
    receivedData = ""
    
    try: 
        tempData, addr = UDP_SOCKET.recvfrom(50)
    except Exception as inst:
        rospy.logwarn('{%s} - this is a controlled catch.', rospy.get_caller_id())
        rospy.logwarn('{%s} - could not receive data from udp socket.', rospy.get_caller_id())
        rospy.logwarn('{%s} - exception type: %s', rospy.get_caller_id(), type(inst))
        rospy.logwarn('{%s} - exception argument: %s', rospy.get_caller_id(), inst.args[1])
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
 
    IP = addr[0]
    rospy.loginfo("{%s} - received data: %s, from app: %s", 
                  rospy.get_caller_id(), receivedData, str(IP))
    chooseAction(receivedData)

def chooseAction(data):
    global currentStatus
    dataArray = data.split(';') # Nachricht wird in ein Array gespeichert
    action = dataArray[0] # erstes Argument
    dataArray = dataArray[1:] # restliche Argumente
    if action == "move":
        # MZAHEDI: not tested yet
        # echo -n ":RUN:move;forward;backward:EOL:" >/dev/udp/localhost/9000
        # echo -n ":RUN:move;forward:EOL:" >/dev/udp/localhost/9000
        move(dataArray)
    # MZAHEDI: adapt when merging codes
#     elif action == "say":
#         print('saying :' + dataArray[0])
#         speechModule.speak(dataArray[0])
#     elif action == "sound":
#         info = dataArray[0]
#         dataArray = dataArray[1:]
#         if info == "play":
#             soundModule.playsound(dataArray)
#     elif action == "face":
#         faceModule.faceManipulation(dataArray)
    elif action == "get":
        info = dataArray[0]
        # MZAHEDI: not defined yet
        # echo -n ":RUN:get;status:EOL:" >/dev/udp/localhost/9000
        if info == "status" and currentStatus: # wenn status abgefragt wird 
            sendToGUI("battery;" + str(currentStatus['batVolt']))
    elif action == "IPcheck":
        pass

""" 
this function is called by chooseAction if the robot has to move
"""
def move(dataArray):
    print(dataArray)
    dir = dataArray[0]
    if len(dataArray) > 1: # step informationen vorhanden, Tablet Toucheingabe verwendet
        print ("step")
        dataArray = dataArray[1:]
        step = dataArray[0] # fuer spaeter hier Erweiterung moeglich auf Laenge der steps eingehen, jetzt nur standardwert verwendet
        if dir == "forward":
            teensyCommunicator.moveForwardStep()
        elif dir == "backward":
            teensyCommunicator.moveBackStep()
        elif dir == "left":
            teensyCommunicator.moveLeftStep()
        elif dir == "right":
            teensyCommunicator.moveRightStep()
    else: # keine step informationen vorhanden, daher loop, Joystick verwendet
        print ("loop")
        if dir == "forward":
            teensyCommunicator.moveForwardLoop()
        elif dir == "backward":
            teensyCommunicator.moveBackLoop()
        elif dir == "left":
            teensyCommunicator.moveLeftLoop()
        elif dir == "right":
            teensyCommunicator.moveRightLoop()
        elif dir == "forward_right":
            teensyCommunicator.moveForwardRightLoop()
        elif dir == "forward_left":
            teensyCommunicator.moveForwardLeftLoop()
        elif dir == "backward_right":
            teensyCommunicator.moveBackRightLoop()
        elif dir == "backward_left":
            teensyCommunicator.moveBackLeftLoop()
        elif dir == "stop":
            teensyCommunicator.stopMovement()

def shutdown():
    if UDP_SOCKET is not None:
        UDP_SOCKET.close()
    rospy.loginfo("{%s} - stopping game communicator node.", rospy.get_caller_id())
    rospy.signal_shutdown("controlled shutdown.")

def GameCommunicator():
    global UDP_IP, UDP_SOCKET
    
    rospy.init_node("robofriend_game_communicator", log_level = rospy.INFO)
    rospy.loginfo("{%s} - starting game communicator handler node!", rospy.get_caller_id())
    rospy.on_shutdown(shutdown)
    
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
        rospy.loginfo("{%s} - udp socket opened.", rospy.get_caller_id())
    except Exception as inst:
        rospy.logwarn('{%s} - this is a controlled catch.', rospy.get_caller_id())
        rospy.logwarn('{%s} - could not open udp socket.', rospy.get_caller_id())
        rospy.logwarn('{%s} - exception type: %s', rospy.get_caller_id(), type(inst))
        rospy.logwarn('{%s} - exception argument: %s', rospy.get_caller_id(), inst.args[1])

    rate = rospy.Rate(1) # 1hz    
    
    while not rospy.is_shutdown():
        srv_resp = None
        rospy.wait_for_service('/robofriend/get_rfid_number')
        
        try:
            request = rospy.ServiceProxy('/robofriend/get_rfid_number', SrvRFIDData)
            srv_resp = request(True)
        except rospy.ServiceException:
            rospy.logwarn("{%s} - service call failed.", rospy.get_caller_id())
   
        provideRFIDNumber(srv_resp)
        
        data_listener()
        rate.sleep() # make sure the publish rate maintains at the needed frequency
        
if __name__ == '__main__':
    try:
        GameCommunicator()
    except rospy.ROSInterruptException:
        pass
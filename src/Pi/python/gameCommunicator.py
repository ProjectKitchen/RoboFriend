import socket
import threading
import sys

# own modules
import teensyCommunicator
import faceModule
import soundModule
import speechModule
import gameCommunicator

# globals
UDP_PORT = 9000 #socket port
AppListener = None
SendToApp = None
IP = ''
UDP_IP = ''
initDone = False
runFlag = True

# This function is used to send information to the gamegui and can be called by the Thread serialRFIDread (for RFID data) and chooseAction (for battery information)
def sendtogui(i):
    global UDP_PORT, SendToApp, IP

    #init if not already initialized
    start()

    if IP == '':
        return
    BytesToSend = bytes(":RUN:" + str(i) + ":EOL:")
    try:
        SendToApp.sendto(BytesToSend, (IP, UDP_PORT+1))
    except OSError as msg:
        print("Error Code: {}, Message: {}".format(str(msg[0]), str(msg[1])))
        IP = ''
# This function is used as Thread to always listen if there is a client (gamegui) sending commands
def data_listener():
    global IP, runFlag

    #Connect the UDP_Port
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
        sock.bind((UDP_IP, UDP_PORT))
        print('***** UDP SOCKET opened ******')
    except:
        print('Could not open UDP SOCKET!')

    startFlag = ":RUN:"
    endFlag = ":EOL:"
    receivedData = ""
    while runFlag:
        tempData, addr = sock.recvfrom(50)
        receivedData += tempData
        startFlagIndex = receivedData.find(startFlag)
        if startFlagIndex != -1:
            receivedData = receivedData[startFlagIndex + len(startFlag):]

        endFlagIndex = receivedData.find(endFlag)
        if endFlagIndex == -1:
            continue

        receivedData = receivedData[:endFlagIndex] # ab dieser Zeile ist nur noch Befehlskette da (ohne start/endflag)
        IP = addr[0]
        print('***** received data from app (' + str(IP) + ') ******')
        chooseAction(receivedData)

    # after endless thread-loop stopped
    sock.close()

def chooseAction(data):
    global currentStatus
    dataArray = data.split(';') # Nachricht wird in ein Array gespeichert
    print(dataArray)
    action = dataArray[0] # erstes Argument
    dataArray = dataArray[1:] # restliche Argumente
    if action == "move":
        move(dataArray)
    elif action == "say":
        print('saying :' + dataArray[0])
        speechModule.speak(dataArray[0])
    elif action == "sound":
        info = dataArray[0]
        dataArray = dataArray[1:]
        if info == "play":
            soundModule.playsound(dataArray)
    elif action == "face":
        faceModule.faceManipulation(dataArray)
    elif action == "get":
        info = dataArray[0]
        if info == "status" and currentStatus: #wenn status abgefragt wird
            gameCommunicator.sendtogui("battery;"+str(currentStatus['batVolt']))
    elif action == "IPcheck":
        pass

# this function is called by chooseAction if the robot has to move
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

def start():
    global UDP_IP, AppListener, SendToApp, initDone

    if not initDone:
        initDone = True
        print("starting gameCommunicator...")
        SendToApp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
        if (len(sys.argv) == 2):
            # Handle command line arguments to get IP address
            try:
                UDP_IP = sys.argv[1]
                socket.inet_aton(UDP_IP)
            except:
                sys.exit('Invalid IP address, Try again')
        else:
            UDP_IP = ''

        AppListener = threading.Thread(target=data_listener)
        AppListener.daemon = True
        AppListener.start()

def stop():
    global runFlag, SendToApp
    print("stopping gameCommunicator...")
    runFlag = False
    SendToApp.close()

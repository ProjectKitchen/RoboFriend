import socket
import threading
import sys

# own modules
import python.legacyApiModule as legacyApiModule

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
        print(BytesToSend)
    except socket.error, msg:
        print 'Error Code: ' + str(msg[0]) + 'Message ' + msg[1]
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
        legacyApiModule.chooseAction(receivedData)

    # after endless thread-loop stopped
    sock.close()

def start():
    global UDP_IP, AppListener, SendToApp, initDone

    if not initDone:
        initDone = True
        print "starting gameCommunicator..."
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
    print "stopping gameCommunicator..."
    runFlag = False
    SendToApp.close()

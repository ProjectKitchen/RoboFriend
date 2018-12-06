import serial
import threading
import gameCommunicator

# globals
serRFID = None
readRFIDnumber = None
RFIDReader = None
runFlag = True

# This function is used as Thread to always listen if there is incoming RFID data
def serialRFIDread():
    global serRFID, readRFIDnumber, runFlag
    try:
        while runFlag:
            data = str(serRFID.read(16))
            #print(type(data))
            print("[INFO] First serial RFID Data: {}".format(data))
            data = data.strip("b'")
            data = data.replace("\\x02", "").replace("\\x03", "").replace("\\x0a", "").replace("\\x0d", "").replace("\\r\\n", "")
            #daten = daten.replace("\x02", "" )
            #daten = daten.replace("\x03", "" )
            #daten = daten.replace("\x0a", "" )
            #daten = daten.replace("\x0d", "" )
            #lock
            readRFIDnumber = data
            #release
            if readRFIDnumber!="empty": #if rfid not locked: if rfid != empty, lock. --- release
                print('***Serial RFID received:')
                print("[INFO] Send RFID Data: {}".format(readRFIDnumber))
                gameCommunicator.sendtogui("rfid;"+str(readRFIDnumber))
                readRFIDnumber="empty"
    finally:
        serRFID.close()

def start():
    global serRFID, RFIDReader

    print("starting rfidModule...")
    try:
        serRFID = serial.Serial("/dev/ttyUSB0", 9600)
        print('***Serial for RFID reader opened***')
        RFIDReader = threading.Thread(target=serialRFIDread)
        RFIDReader.daemon = True
        RFIDReader.start()
    except:
        print('***Serial for RFID could not be opened***')

def stop():
    global serRFID, runFlag
    print("stopping rfidModule...")
    runFlag = False
    serRFID.close()

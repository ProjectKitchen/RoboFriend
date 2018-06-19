import serial
import threading

# globals
send_lock = threading.Lock()
ser = None

def init():
    global ser
    try:
        ser = serial.Serial("/dev/ttyACM0", 9600, timeout=1)
        print('***Serial for Teensy opened***')
    except:
        print('***Serial for Teensy could not be opened***')

def close():
    global ser
    ser.close()

def moveRightStep():
    stopMovement()
    sendSerial("3")

def moveRightLoop():
    stopMovement()
    sendSerial("e")

def moveLeftStep():
    stopMovement()
    sendSerial("4")

def moveLeftLoop():
    stopMovement()
    sendSerial("f")

def moveForwardStep():
    stopMovement()
    sendSerial("1")

def moveForwardLoop():
    stopMovement()
    sendSerial("c")

def moveBackStep():
    stopMovement()
    sendSerial("2")

def moveBackLoop():
    stopMovement()
    sendSerial("d")

def moveBackLeftLoop():
    stopMovement()
    sendSerial("6")

def moveBackRightLoop():
    stopMovement()
    sendSerial("5")

def moveForwardRightLoop():
    stopMovement()
    sendSerial("a")

def moveForwardLeftLoop():
    stopMovement()
    sendSerial("b")

def stopMovement():
    sendSerial("7")

def shakeHeadForNo():
    sendSerial("9")

def getStatus():
    response = sendSerial("8", True)
    batteryValue = -1
#    try:
#        batteryValue = int(response.strip(' \t\n\r'))
#    except:
#        print("error parsing battery value: " + str(batteryValue))
#    return {'battery': batteryValue }
    return response

def sendSerial(commandString, readResponse=False):
    global send_lock, ser
    response = None
    send_lock.acquire()
    print("sending serial command: " + str(commandString))
    try:
        ser.write(str(commandString))
        if readResponse:
            response = str(ser.readline())
    except:
        print('***Serial write error ***')
    finally:
        send_lock.release()

    return response

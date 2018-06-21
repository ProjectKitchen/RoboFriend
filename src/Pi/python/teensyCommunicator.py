import serial
import threading
from time import sleep

# globals
send_lock = threading.Lock()
ser = None
driveDuration = 50

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
    global driveDuration
    stopMovement()
    sendSerial("D 128 -128 " + str(driveDuration))

def moveRightLoop():
    stopMovement()
    sendSerial("D 128 -128 0")

def moveLeftStep():
    global driveDuration
    stopMovement()
    sendSerial("D -128 128 " + str(driveDuration))

def moveLeftLoop():
    stopMovement()
    sendSerial("D -128 128 0")

def moveForwardStep():
    global driveDuration
    stopMovement()
    sendSerial("D 255 255 " + str(driveDuration))

def moveForwardLoop():
    stopMovement()
    sendSerial("D 255 255 0")

def moveBackStep():
    global driveDuration
    stopMovement()
    sendSerial("D -255 -255 " + str(driveDuration))

def moveBackLoop():
    stopMovement()
    sendSerial("D -255 -255 0")

def moveBackLeftLoop():
    stopMovement()
    sendSerial("D -128 0 0")

def moveBackRightLoop():
    stopMovement()
    sendSerial("D 0 -128 0")

def moveForwardRightLoop():
    stopMovement()
    sendSerial("D 128 0 0")

def moveForwardLeftLoop():
    stopMovement()
    sendSerial("D 0 128 0")

def stopMovement():
    stopMovement()
    sendSerial("D")

def shakeHeadForNo():
    sendSerial("D 50 -50 10")
    sleep(0.5)
    sendSerial("D -50 50 10")

def getStatus():
    response = sendSerial("R", True)
    print "status: " + response
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

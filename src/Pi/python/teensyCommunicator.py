"""
TODO: Replace print function with the origin function
      (return sendSerial("R", True)) when teensy connected
"""

import serial
import threading
from time import sleep

# globals
send_lock = threading.Lock()
ser = None
driveDuration = 50

#init
print("initializing teensyCommunicator...")
try:
    ser = serial.Serial("/dev/ttyACM0", 9600, timeout=1)
    print('***Serial for Teensy opened***')
except:
    print('***Serial for Teensy could not be opened***')

def close():
    global ser
    ser.close()

def move(left, right, duration):
    stopMovement()
    sendSerial("D " + str(left) + " " + str(right) + " " + str(duration))

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
    import statusModule
    statusModule.setNonIdle()
    sendSerial("D")

def shakeHeadForNo():
    sendSerial("D 50 -50 10")
    sleep(0.5)
    sendSerial("D -50 50 10")

def getRawStatus():
#    return sendSerial("R", True)
#    print('***Faking Teensy Values !! ***')
    return ("Sensors,0500,0200,0100,0300\n")

def sendSerial(commandString, readResponse=False):
    global send_lock, ser
    response = None
    send_lock.acquire()
    print("sending serial command: " + str(commandString))
    try:
        ser.write(str(commandString) + "\r")
        if readResponse:
            response = str(ser.readline())
    except:
        print('***Serial write error ***')

    send_lock.release()

    return response

def stop():
    global ser
    print("stopping teensyCommunicator...")
    ser.close()

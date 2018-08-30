# external modules
import time
from time import sleep
import threading
import os

#own modules
import teensyCommunicator
import gameCommunicator
import speechModule

# globals
currentStatus = {}
StatusThread = None
refreshIntervalMs = 1000
keyBat = 'batVolt'
keyIrL = 'irLeft'
keyIrM = 'irMiddle'
keyIrR = 'irRight'
keyScreenshotTimestamp = 'screenshotTimestamp'
screenshotTimestamp = None
runFlag = True
batCoversionConstant =  0.04783948
batMovingAverageN = 20

def getStatus():
    return currentStatus

def getBatteryVoltage():
    global currentStatus, keyBat
    return currentStatus[keyBat] if keyBat in currentStatus else None

def getIRLeft():
    global currentStatus, keyIrL
    return currentStatus[keyIrL] if keyIrL in currentStatus else None

def getIRMiddle():
    global currentStatus, keyIrM
    return currentStatus[keyIrM] if keyIrM in currentStatus else None

def getIRRight():
    global currentStatus, keyIrR
    return currentStatus[keyIrR] if keyIrR in currentStatus else None

def getScreenshotTimestamp():
    global currentStatus, keyScreenshotTimestamp
    return currentStatus[keyScreenshotTimestamp] if keyScreenshotTimestamp in currentStatus else None

def setScreenshotTimestamp(timestamp = None):
    if not timestamp:
        timestamp = time.time()
    global currentStatus, keyScreenshotTimestamp
    currentStatus[keyScreenshotTimestamp] = timestamp

# This function is used as Thread to periodically update the battery information
def StatusInfo():
    global currentStatus, refreshIntervalMs, keyBat, keyIrL, keyIrM, keyIrR, runFlag
    while runFlag:
        rawStatus = teensyCommunicator.getRawStatus()
        currentStatus = updateFromRawStatus(rawStatus)
        gameCommunicator.sendtogui("battery;"+str(getBatteryVoltage()))

        print ("Battery= " + str(getBatteryVoltage()) + " Volt)")
        if getBatteryVoltage() < 12.0:
            speechModule.speakBatteryLow()
            print ("LOW BATTERY - Please Recharge! Robofriend will shutdown automatically at 11.8 Volt!")
        if getBatteryVoltage() < 11.8:
            speechModule.speakBatteryShutdown()
            sleep(5)
            os.system("init 0")
        print ("IRSensors="+str(getIRLeft())+"/"+str(getIRMiddle())+"/"+str(getIRRight()))
        time.sleep(refreshIntervalMs / 1000.0)

def updateFromRawStatus(rawStatus):
    global currentStatus, keyBat, keyIrL, keyIrM, keyIrR, batCoversionConstant, batMovingAverageN

    batVolt = getBatteryVoltage()
    irSensorLeft = -1
    irSensorMiddle = -1
    irSensorRight = -1
    try:
        statusArray = rawStatus.split(',')
        if (statusArray[0] == "Sensors"):
            newbatVolt = int(statusArray[1]) * batCoversionConstant
            batVolt = getMovingAverage(newbatVolt, batVolt, batMovingAverageN)
            irSensorLeft = int(statusArray[2])
            irSensorMiddle = int(statusArray[3])
            irSensorRight = int(statusArray[4])
    except:
        print("error parsing status values: " + rawStatus)

    currentStatus[keyBat] = batVolt
    currentStatus[keyIrL] = irSensorLeft
    currentStatus[keyIrM] = irSensorMiddle
    currentStatus[keyIrR] = irSensorRight
    return currentStatus

def getMovingAverage(newValue, currentMean, n):
    if not currentMean:
        return newValue
    newMean = currentMean - currentMean / n
    newMean = newMean + newValue / n
    return newMean

def start():
    global StatusThread

    print "starting statusModule..."
    StatusThread = threading.Thread(target=StatusInfo)
    StatusThread.daemon = True
    StatusThread.start()

def stop():
    global runFlag
    print "stopping statusModule..."
    runFlag = False
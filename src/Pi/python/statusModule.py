# external modules
import time
from time import sleep
import os

#own modules
import speechModule
#import gameCommunicator

# globals
refreshIntervalMs = 1000
keyBat = 'batVolt'
keyBatPercent = 'batPercent'
keyIrL = 'irLeft'
keyIrM = 'irMiddle'
keyIrR = 'irRight'
keyScreenshotTimestamp = 'screenshotTimestamp'
keyIsIdle = 'isIdle'
currentStatus = {keyIsIdle: False}
screenshotTimestamp = None
idleTimestamp = time.time()
idleThresholdSeconds = 90
runFlag = True
# batCoversionConstant =  0.04783948
batMovingAverageN = 30
statusCount = 0
batWasLow = False

def getStatus():
    return currentStatus

def getScreenshotTimestamp():
    global currentStatus, keyScreenshotTimestamp
    return currentStatus[keyScreenshotTimestamp] if keyScreenshotTimestamp in currentStatus else None

def setScreenshotTimestamp(timestamp = None):
    if not timestamp:
        timestamp = time.time()
    global currentStatus, keyScreenshotTimestamp
    currentStatus[keyScreenshotTimestamp] = timestamp

def isIdle():
    global currentStatus, idleTimestamp, keyIsIdle
    return currentStatus[keyIsIdle]

def setNonIdle():
    global currentStatus, idleTimestamp, keyIsIdle
    currentStatus[keyIsIdle] = False
    idleTimestamp = time.time()


# This function is used as Thread to periodically update the battery information
def StatusInfo():
    global currentStatus, refreshIntervalMs, keyBat, keyIrL, keyIrM, keyIrR, runFlag, batWasLow, idleTimestamp, idleThresholdSeconds, keyIsIdle, statusCount, batMovingAverageN
    while runFlag:
        statusCount += 1
        if time.time() - idleTimestamp > idleThresholdSeconds:
            currentStatus[keyIsIdle] = True
#         rawStatus = teensyCommunicator.getRawStatus()
        currentStatus = updateFromRawStatus(rawStatus)
        gameCommunicator.sendToGUI("battery;"+str(getBatteryVoltage()))

        if statusCount >= batMovingAverageN and getBatteryVoltage() < 12.0:
            batWasLow = True
            speechModule.speakBatteryLow()
            print ("LOW BATTERY - Please Recharge! Robofriend will shutdown automatically at 11.8 Volt!")
        if statusCount >= batMovingAverageN and getBatteryVoltage() < 11.8: #shutdown
            speechModule.speakBatteryShutdown()
            sleep(5)
            os.system("init 0")
        if batWasLow and getBatteryVoltage() > 12.15:   # recharging
            batWasLow = False
            speechModule.speakOnRecharge()
        time.sleep(refreshIntervalMs / 1000.0)

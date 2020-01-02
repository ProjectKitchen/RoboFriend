# external modules
import time
from time import sleep
import threading
import os

#own modules
import speechModule
import utils

# globals
StatusThread = None
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
"""
# this is the voltage divider result in volts (between 0 - ADC_EXTERNAL_VREF)
    volt_div_val = constants.ADC_EXTERNAL_VREF / float(constants.ADC_RESOLUTION) * args.bat_voltage
    # this is the actual battery voltage (between 0 - 12V)
    act_bat_volt = (volt_div_val * (constants.VOLT_DIV_R1 + constants.VOLT_DIV_R2)) / constants.VOLT_DIV_R2

"""
#batCoversionConstant = 0.014388151648768962 #0.04783948 #0.004003910068426197
VOLT_DIV_R1 = 7320
VOLT_DIV_R2 = 2340  # 2780
BAT_UPPER_THRESHOLD = 14.7
BAT_LOWWER_THREDSHOLD = 10.7
ADC_EXTERNAL_VREF = 4.096
ADC_RESOLUTION = 1023
batVolt = 12.0
batPercent = 100


batMovingAverageN = 30
statusCount = 0
batWasLow = False

def getStatus():
    return currentStatus

def getBatteryVoltage():
    global currentStatus, keyBat
    return currentStatus[keyBat] if keyBat in currentStatus else None

#def getBatteryPercent():
#    global currentStatus, keyBatPercent
#    return currentStatus[keyBatPercent] if keyBatPercent in currentStatus else None

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

def isIdle():
    global currentStatus, idleTimestamp, keyIsIdle
    return currentStatus[keyIsIdle]


def setNonIdle():
    global currentStatus, idleTimestamp, keyIsIdle
    currentStatus[keyIsIdle] = False
    idleTimestamp = time.time()


# This function is used as Thread to periodically update the battery information
def StatusInfo():
    import gameCommunicator
    import teensyCommunicator
    global currentStatus, refreshIntervalMs, keyBat, keyIrL, keyIrM, keyIrR, runFlag, batWasLow, idleTimestamp, idleThresholdSeconds, keyIsIdle, statusCount, batMovingAverageN, batVolt, batPercent
    while runFlag:
        statusCount += 1
        if time.time() - idleTimestamp > idleThresholdSeconds:
            currentStatus[keyIsIdle] = True
        rawStatus = teensyCommunicator.getRawStatus()
        currentStatus = updateFromRawStatus(rawStatus)
        gameCommunicator.sendtogui("battery;"+str(getBatteryVoltage()))

        print ("Battery= " + str(batPercent) + "% ("  +str(batVolt) + " Volt)")
        if statusCount >= batMovingAverageN and batVolt < 12.0:
            batWasLow = True
            speechModule.speakBatteryLow()
            print ("LOW BATTERY - Please Recharge! Robofriend will shutdown automatically at 11.8 Volt!")
        if statusCount >= batMovingAverageN and batVolt < 11.8:
            speechModule.speakBatteryShutdown()
            sleep(5)
            #os.system("init 0")
        if batWasLow and batVolt > 12.15:
            batWasLow = False
            speechModule.speakOnRecharge()
        print ("IRSensors="+str(getIRLeft())+"/"+str(getIRMiddle())+"/"+str(getIRRight()))
        time.sleep(refreshIntervalMs / 1000.0)

def updateFromRawStatus(rawStatus):
    global currentStatus, keyBat, keyBatPercent, keyIrL, keyIrM, keyIrR, batMovingAverageN
    global batVolt, batPercent

    batVolt=getBatteryVoltage()
    irSensorLeft = -1
    irSensorMiddle = -1
    irSensorRight = -1
    try:
        statusArray = rawStatus.split(',')
        if (statusArray[0] == "Sensors"):
            #newbatVolt = int(statusArray[1]) * batCoversionConstant
            #batVolt = getMovingAverage(newbatVolt, batVolt, batMovingAverageN)
            #batVolt=getBatteryVoltage()
            volt_div_val = ADC_EXTERNAL_VREF / float(ADC_RESOLUTION) * int(statusArray[1])
            print(str(volt_div_val))
            act_bat_volt = (volt_div_val * (VOLT_DIV_R1 + VOLT_DIV_R2)) / VOLT_DIV_R2
            print(str(act_bat_volt))
            batVolt = getMovingAverage(act_bat_volt, batVolt, batMovingAverageN)
            print(str(batVolt))
            batPercent = getBatteryPercent(batVolt)
            print(str(batPercent))

            irSensorLeft = int(statusArray[2])
            irSensorMiddle = int(statusArray[3])
            irSensorRight = int(statusArray[4])
    except:
        print("error parsing status values: " + str(rawStatus))

    currentStatus[keyBat] = batVolt
    currentStatus[keyBatPercent] = batPercent#utils.getBatPercent(batVolt)
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

def getBatteryPercent(batVoltage):
    batVoltageRounded = round(batVoltage, 2)

    # dont multiply by 100 cause sensor_msgs/BatteryState.percentage
    # is the charge percentage on 0 to 1 range
    percent = round(
        (batVoltageRounded - BAT_LOWWER_THREDSHOLD) / 
        (BAT_UPPER_THRESHOLD - BAT_LOWWER_THREDSHOLD), 2)
    return percent

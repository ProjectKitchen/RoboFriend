import os
import threading
import time
import random

# globals
runFlag = False
RandomThread = None
refreshIntervalMs = 300
cameraPos = 140
earColorR = 0
earColorG = 10
earColorB = 10
randMaxInc = 1
randFactorR = 1
randFactorG = 1
randFactorB = 1

def sendToIOWarrior(earColorR, earColorG, earColorB, cameraPos = None):
    cmd = 'sudo ./iowarrior/iow ' + str(int(round(earColorR))) + ' ' + str(int(round(earColorG))) + ' ' + str(int(round(earColorB)))
    if cameraPos:
        cmd = cmd + ' ' + str(cameraPos)
    print("starting cmd: {}".format(cmd))
    os.system(cmd)

def changeCameraPos(diff):
    global earColorR, earColorG, earColorB, cameraPos
    if 10 <= cameraPos + diff <= 150:
        cameraPos = cameraPos + diff
        sendToIOWarrior(earColorR, earColorG, earColorB, cameraPos)

def setEarColor(r, g, b):
    global earColorR, earColorG, earColorB
    stopRandomEarColor()
    earColorR = r
    earColorG = g
    earColorB = b
    print(r, g, b)
    sendToIOWarrior(r, g, b)

def setEarColorSeries(seriesRGB, repeatNum=1, waitTimeMs=500):
    global earColorR, earColorG, earColorB
    for x in range(repeatNum):
        for rgb in seriesRGB:
            setEarColor(rgb[0], rgb[1], rgb[2])
            time.sleep(waitTimeMs / 1000.0)

def startRandomEarColor():
    global runFlag, RandomThread

    if not runFlag:
        runFlag = True
        RandomThread = threading.Thread(target=randomThreadMethod)
        RandomThread.daemon = True
        RandomThread.start()

def stopRandomEarColor():
    global runFlag
    runFlag = False

def randomThreadMethod():
    global runFlag, refreshIntervalMs, earColorR, earColorG, earColorB, randFactorR, randFactorG, randFactorB
    while runFlag:
        earColorR, randFactorR = getNewRandomColor(earColorR, randFactorR)
        earColorG, randFactorG  = getNewRandomColor(earColorG, randFactorG)
        earColorB, randFactorB  = getNewRandomColor(earColorB, randFactorB)
        sendToIOWarrior(earColorR, earColorG, earColorB)
        time.sleep(refreshIntervalMs / 1000.0)

def getNewRandomColor(oldColor, factor):
    newColor = oldColor + random.uniform(0, 1) * factor
    if newColor > 15:
        newColor = 15
        factor *= -1
    elif newColor < 0:
        newColor = 0
        factor *= -1
    return newColor, factor

def stop():
    global runFlag
    print("stopping ioWarriorModule...")
    stopRandomEarColor()

#init
print("initializing ioWarriorModule...")
sendToIOWarrior(earColorR, earColorG, earColorB, cameraPos)

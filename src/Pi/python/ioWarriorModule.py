import os

# globals
cameraPos = 140
earColorR = 0
earColorG = 10
earColorB = 10

def sendToIOWarrior(earColorR, earColorG, earColorB, cameraPos):
    os.system('sudo ./iowarrior/iow ' + str(earColorR) + ' ' + str(earColorG) + ' ' + str(earColorB) + ' ' + str(cameraPos))

def changeCameraPos(diff):
    global earColorR, earColorG, earColorB, cameraPos
    if 10 <= cameraPos + diff <= 160:
        cameraPos = cameraPos + diff
        sendToIOWarrior(earColorR, earColorG, earColorB, cameraPos)

def setEarColor(r, g, b):
    global earColorR, earColorG, earColorB, cameraPos
    earColorR = r
    earColorG = g
    earColorB = b
    sendToIOWarrior(r, g, b, cameraPos)

#init
print("initializing ioWarriorModule...")
sendToIOWarrior(earColorR, earColorG, earColorB, cameraPos)
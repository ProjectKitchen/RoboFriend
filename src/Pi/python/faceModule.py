import pygame
import pygame.gfxdraw

import teensyCommunicator
import utils

"""TODO: restore FULLSCREEN Flag """

#init
print("initializing faceModule...")
pygame.init()
#screen = pygame.display.set_mode((654, 380), pygame.FULLSCREEN)
screen = pygame.display.set_mode((654, 380))
screen.fill((0, 0, 0))

# globals
eyex = 0
eyey = 0
smilePercent = 60
eyestep = 10
screenshotFilename = 'screenshot.jpg'

def close():
    pygame.quit()

def getScreenshotFilename():
    global screenshotFilename
    return screenshotFilename

#set smile in percent, +100 is most happy, -100 is most sad, 0 is neutral
def setSmile(percent):
    global smilePercent
    smilePercent = utils.restrictRange(percent, -100, 100)
    drawFace()

def increaseSmile():
    global smilePercent
    smilePercent = utils.restrictRange(smilePercent + 10, -100, 100)
    drawFace()

def decreaseSmile():
    global smilePercent
    smilePercent = utils.restrictRange(smilePercent - 10, -100, 100)
    drawFace()

def setEyes(xPercent, yPercent):
    global eyex, eyey
    if xPercent < -100:
        xPercent = -100
    elif xPercent > 100:
        xPercent = 100
    if yPercent < -100:
        yPercent = -100
    elif yPercent > 100:
        yPercent = 100

    eyex = int(xPercent * (40.0 / 100))
    eyey = int(yPercent * (40.0 / 100))
    drawFace()

def eyesUp():
    global eyey, eyestep
    if eyey > -40:
        eyey=eyey-eyestep
    drawFace()

def eyesDown():
    global eyey, eyestep
    if eyey < 40:
        eyey=eyey+eyestep
    drawFace()

def eyesLeft():
    global eyex, eyestep
    if eyex > -40:
        eyex=eyex-eyestep
    drawFace()

def eyesRight():
    global eyex, eyestep
    if eyex < 40:
        eyex=eyex+eyestep
    drawFace()

def isSad():
    global smilePercent
    return smilePercent < 0

def drawFace():
    import statusModule
    global screen, eyex, eyey, screenshotFilename
    screen.fill((0, 0, 0))
    pygame.draw.circle(screen, (100,250,250), (163,100), 60) #lefteye
    pygame.draw.circle(screen, (100,250,250), (491,100), 60) #righteye
    pygame.draw.circle(screen, (10,10,10), (163+eyex,100+eyey), 20) #leftpupil
    pygame.draw.circle(screen, (10,10,10), (491+eyex,100+eyey), 20) #rightpupil
    drawMouth()
    pygame.display.flip()
    pygame.image.save(screen, screenshotFilename)
    statusModule.setScreenshotTimestamp()

def drawMouth():
    global smilePercent

    if smilePercent < 0:
        radius = utils.mapRange(abs(smilePercent), 0, 100, 0.3, 0.9)
        pygame.draw.arc(screen, (100, 200, 200), (57, 300, 540, 400), 1.57 - radius, 1.57 + radius, 20)  # smile
    else:
        radius = utils.mapRange(abs(smilePercent), 0, 100, 0.3, 1.3)
        pygame.draw.arc(screen, (100, 200, 200), (57, -30, 540, 400), 4.7 - radius, 4.7 + radius, 20)

# deprecated - use the distinct methods like increaseSmile() instead
# this function is called by chooseAction if the facial expression of the robot has to change
def faceManipulation(dataArray):
    global smilePercent
    faceObject = dataArray[0] #eyes or smile
    dataArray = dataArray[1:]
    if faceObject == "smile":
        changing = dataArray[0]
        if changing == "increase":
            increaseSmile()
        elif changing == "decrease":
            decreaseSmile()
    elif faceObject == "eyes":
        changing = dataArray[0]
        if changing == "up":
            eyesUp()
        elif changing == "down":
            eyesDown()
        elif changing == "right":
            eyesRight()
        elif changing == "left":
            eyesLeft()
    elif faceObject == "answer":
        changing = dataArray[0]
        if changing == "correct":
            smilePercent = 60
        if changing == "wrong":
            smilePercent = -60
            teensyCommunicator.shakeHeadForNo()
    drawFace()

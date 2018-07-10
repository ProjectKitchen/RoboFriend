import pygame
import pygame.gfxdraw

import python.teensyCommunicator as teensySender

#init
print "initializing faceModule..."
pygame.init()
screen = pygame.display.set_mode((654, 380), pygame.FULLSCREEN)
screen.fill((0, 0, 0))

# globals
radius = 0.8
eyex = 0
eyey = 0
sadFace = 0
eyestep = 10

def close():
    pygame.quit()

def increaseSmile():
    faceManipulation("face;smile;increase")

def decreaseSmile():
    faceManipulation("face;smile;decrease")

def eyesUp():
    faceManipulation("face;eyes;up")

def eyesDown():
    faceManipulation("face;eyes;down")

def eyesLeft():
    faceManipulation("face;eyes;left")

def eyesRight():
    faceManipulation("face;eyes;right")

def isSad():
    global sadFace
    return sadFace

# Updating Facial expression of robot in case of a happy expression
def drawHappyFace():
    global screen, eyex, eyey, radius
    screen.fill((0, 0, 0))
    pygame.draw.circle(screen, (100,250,250), (163,100), 60) #lefteye
    pygame.draw.circle(screen, (100,250,250), (491,100), 60) #righteye
    pygame.draw.circle(screen, (10,10,10), (163+eyex,100+eyey), 20) #leftpupil
    pygame.draw.circle(screen, (10,10,10), (491+eyex,100+eyey), 20) #rightpupil
    pygame.draw.arc(screen, (100,200,200), (57, -30, 540, 400), 4.7-radius, 4.7+radius, 20) #smile
    pygame.display.flip()

# Updating Facial expression of robot in case of a sad expression
def drawSadFace():
    global screen, eyex, eyey, radius
    screen.fill((0, 0, 0))
    pygame.draw.circle(screen, (100,250,250), (163,100), 60) #lefteye
    pygame.draw.circle(screen, (100,250,250), (491,100), 60) #righteye
    pygame.draw.circle(screen, (10,10,10), (163+eyex,100+eyey), 20) #leftpupil
    pygame.draw.circle(screen, (10,10,10), (491+eyex,100+eyey), 20) #rightpupil
    pygame.draw.arc(screen, (100,200,200), (57, 300, 540, 400), 1.57-radius, 1.57+radius, 20) #smile
    pygame.display.flip()


# deprecated - use the distinct methods like increaseSmile() instead
# this function is called by chooseAction if the facial expression of the robot has to change
def faceManipulation(dataArray):
    global radius, eyey, eyex, sadFace, eyestep
    faceObject = dataArray[0] #eyes or smile
    dataArray = dataArray[1:]
    if faceObject == "smile":
        changing = dataArray[0]
        if changing == "increase":
            if (radius < 1.3 and sadFace == 0):
                radius=radius+0.1
            elif (sadFace ==1):
                if radius < 0.3:
                    sadFace = 0
                    radius = radius +0.1
                else:
                    sadFace = 1
                    radius = radius-0.1
        elif changing == "decrease":
            if (radius > 0.3 and sadFace == 0):
                radius=radius-0.1
            elif ((sadFace==1 or radius < 0.3) and radius < 0.8):
                sadFace=1
                radius=radius+0.1
    elif faceObject == "eyes":
        changing = dataArray[0]
        if changing == "up":
            if eyey > -40:
                eyey=eyey-eyestep
        elif changing == "down":
            if eyey < 40:
                eyey=eyey+eyestep
        elif changing == "right":
            if eyex < 40:
                eyex=eyex+eyestep
        elif changing == "left":
            if eyex > -40:
                eyex=eyex-eyestep
    elif faceObject == "answer":
        changing = dataArray[0]
        if changing == "correct":
            sadFace = 0
            radius = 0.8
        if changing == "wrong":
            sadFace = 1
            radius = 0.8
            teensySender.shakeHeadForNo()
    if sadFace == 0:
        drawHappyFace()
    elif sadFace == 1:
        drawSadFace()
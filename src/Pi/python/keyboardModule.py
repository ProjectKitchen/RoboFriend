# external modules
import pygame
import sys
import threading

# own modules
import teensyCommunicator
import faceModule
import soundModule

# globals
currentStatus = None
KeyboardThred = None
refreshIntervalMs = 1000
keyBat = 'batVolt'
keyIrL = 'irLeft'
keyIrM = 'irMiddle'
keyIrR = 'irRight'
runFlag = True

def handleKeyboard():
    global runFlag
    try:
        while runFlag:
            event = pygame.event.wait()
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN:
                print('***** Key press recognized ******')
                if event.key == pygame.K_ESCAPE or event.unicode == 'q':
                    print('quit via keyboard')
                    pygame.quit()
                    sys.exit()
                if event.unicode == '1':
                    print('smile increase via keyboard')
                    faceModule.increaseSmile()
                if event.unicode == '2':
                    print('smile decrease via keyboard')
                    faceModule.decreaseSmile()
                if event.key == pygame.K_UP:
                    print("move forward via keyboard")
                    teensyCommunicator.moveForwardStep()
                if event.key == pygame.K_DOWN:
                    print('move back via keyboard')
                    teensyCommunicator.moveBackStep()
                if event.key == pygame.K_LEFT:
                    print("move left via keyboard")
                    teensyCommunicator.moveLeftStep()
                if event.key == pygame.K_RIGHT:
                    print('move right via keyboard')
                    teensyCommunicator.moveRightStep()
                if event.key == pygame.K_RETURN:
                    print('move stop via keyboard')
                    teensyCommunicator.stopMovement()
                if event.unicode == '3':
                    print('sound play random via keyboard')
                    soundModule.playRandom()
                if event.unicode == '4':
                    print('sound play mood via keyboard')
                    soundModule.playMood()
    except Exception:
        pygame.quit()

def start():
    global KeyboardThred

    print "starting keyboardModule..."
    KeyboardThred = threading.Thread(target=handleKeyboard)
    KeyboardThred.daemon = True
    KeyboardThred.start()

def stop():
    global runFlag
    print "stopping keyboardModule..."
    runFlag = False
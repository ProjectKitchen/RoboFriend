# external modules
import pygame
import sys
import threading
import re
import traceback

# own modules
import teensyCommunicator
import faceModule
import soundModule
import speechModule
import systemModule

# globals
KeyboardThread = None
runFlag = True
speechBuffer = ''
shutdownKeyword = 'exit'
quitKeyword = 'quit'
lastSay = None


def handleKeyboard():
    global runFlag, speechBuffer, shutdownKeyword, quitKeyword, lastSay

    while runFlag:
        try:
            event = pygame.event.wait()
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYUP and event.key in [pygame.K_DOWN, pygame.K_LEFT, pygame.K_RIGHT, pygame.K_UP]:
                print('move stop via keyboard')
                teensyCommunicator.stopMovement()
            if event.type == pygame.KEYDOWN:
                print('***** Key press recognized: ')

                if event.key == pygame.K_RETURN:
                    if speechBuffer == shutdownKeyword:
                        systemModule.shutdown()
                    if speechBuffer == quitKeyword:
                        pygame.quit()
                        sys.exit()
                    elif speechBuffer:
                        speechModule.speak(speechBuffer)
                        lastSay = speechBuffer
                    speechBuffer = ''
                elif event.key == pygame.K_ESCAPE:
                    print('clearing speech buffer...')
                    speechBuffer = ''
                elif event.key == pygame.K_BACKSPACE:
                    if lastSay:
                        speechModule.speak(lastSay)

                # ------------ move ---------------
                elif event.key == pygame.K_DOWN:
                    print('move back via keyboard')
                    teensyCommunicator.moveBackLoop()
                elif event.key == pygame.K_LEFT:
                    print("move left via keyboard")
                    teensyCommunicator.moveLeftLoop()
                elif event.key == pygame.K_RIGHT:
                    print('move right via keyboard')
                    teensyCommunicator.moveRightLoop()
                elif event.key == pygame.K_UP:
                    print("move forward via keyboard")
                    teensyCommunicator.moveForwardLoop()

                # ------------ face ---------------
                elif event.unicode == ',':
                    print('smile increase via keyboard')
                    faceModule.increaseSmile()
                elif event.unicode == '.':
                    print('smile decrease via keyboard')
                    faceModule.decreaseSmile()

                # ------------ sounds ---------------
                elif event.unicode == '-':
                    soundModule.playRandom()
                elif event.unicode == '#':
                    soundModule.playLastRandom()

                # ------------ speech ---------------
                elif event.unicode == '1':
                    speechModule.speak('Hallo, wie gehts?')
                elif event.unicode == '2':
                    speechModule.speak('Danke, mir geht es gut.')
                elif event.unicode == '3':
                    speechModule.speak('Willst du etwas zum knabbern?')
                elif event.unicode == '4':
                    speechModule.speak('Bitte, gerne.')
                elif event.unicode == '5':
                    speechModule.speak('Wie heisst du?')
                elif event.unicode == '6':
                    speechModule.speak('Ich heisse Robofreund.')
                elif event.unicode == '7':
                    speechModule.speak('Mir ist langweilig')
                elif event.unicode == '8':
                    speechModule.speak('Heute ist ein schoener Tag.')
                elif event.unicode == '9':
                    speechModule.speakRandom()
                elif event.unicode == '0':
                    speechModule.speakBullshit()
                elif re.match('^[a-zA-Z ]$', event.unicode):
                    speechBuffer += event.unicode
                    print('speech buffer is now: ' + str(speechBuffer))
        except Exception as e:
            print('keyboard exception: ' + str(e))
            print(traceback.format_exc())

def start():
    global KeyboardThread

    print("starting keyboardModule...")
    KeyboardThread = threading.Thread(target=handleKeyboard)
    KeyboardThread.daemon = True
    KeyboardThread.start()

def stop():
    global runFlag
    print("stopping keyboardModule...")
    runFlag = False

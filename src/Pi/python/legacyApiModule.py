import teensyCommunicator
import faceModule
import soundModule
import speechModule
import gameCommunicator

#init
print "initializing legacyApiModule..."

# globals
# --- none ---

def chooseAction(data):
    global currentStatus
    dataArray = data.split(';') # Nachricht wird in ein Array gespeichert
    print(dataArray)
    action = dataArray[0] # erstes Argument
    dataArray = dataArray[1:] # restliche Argumente
    if action == "move":
        move(dataArray)
    elif action == "say":
        print('saying :' + dataArray[0])
        speechModule.speak(dataArray[0])
    elif action == "sound":
        info = dataArray[0]
        dataArray = dataArray[1:]
        if info == "play":
            soundModule.playsound(dataArray)
    elif action == "face":
        faceModule.faceManipulation(dataArray)
    elif action == "get":
        info = dataArray[0]
        if info == "status" and currentStatus: #wenn status abgefragt wird
            gameCommunicator.sendtogui("battery;"+str(currentStatus['batVolt']))
    elif action == "IPcheck":
        pass

# this function is called by chooseAction if the robot has to move
def move(dataArray):
    print(dataArray)
    dir = dataArray[0]
    if len(dataArray) > 1: # step informationen vorhanden, Tablet Toucheingabe verwendet
        print ("step")
        dataArray = dataArray[1:]
        step = dataArray[0] # fuer spaeter hier Erweiterung moeglich auf Laenge der steps eingehen, jetzt nur standardwert verwendet
        if dir == "forward":
            teensyCommunicator.moveForwardStep()
        elif dir == "backward":
            teensyCommunicator.moveBackStep()
        elif dir == "left":
            teensyCommunicator.moveLeftStep()
        elif dir == "right":
            teensyCommunicator.moveRightStep()
    else: # keine step informationen vorhanden, daher loop, Joystick verwendet
        print ("loop")
        if dir == "forward":
            teensyCommunicator.moveForwardLoop()
        elif dir == "backward":
            teensyCommunicator.moveBackLoop()
        elif dir == "left":
            teensyCommunicator.moveLeftLoop()
        elif dir == "right":
            teensyCommunicator.moveRightLoop()
        elif dir == "forward_right":
            teensyCommunicator.moveForwardRightLoop()
        elif dir == "forward_left":
            teensyCommunicator.moveForwardLeftLoop()
        elif dir == "backward_right":
            teensyCommunicator.moveBackRightLoop()
        elif dir == "backward_left":
            teensyCommunicator.moveBackLeftLoop()
        elif dir == "stop":
            teensyCommunicator.stopMovement()
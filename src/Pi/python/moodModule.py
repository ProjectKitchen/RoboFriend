# external modules

# own modules
import faceModule
import soundModule

# globals
# ... none

def setNeutral():
    faceModule.setEyes(0, 0)
    faceModule.setSmile(20)
    soundModule.playSoundFile('data/happy.wav')

def setHappy():
    faceModule.setEyes(9, -47)
    faceModule.setSmile(80)
    soundModule.playSoundFile('data/happy.wav')

def setAngry():
    faceModule.setEyes(-46, -40)
    faceModule.setSmile(-10)
    soundModule.playSoundFile('data/sad.wav')

def setSad():
    faceModule.setEyes(-1, -44)
    faceModule.setSmile(-70)
    soundModule.playSoundFile('data/sad.wav')
# external modules

# own modules
import faceModule
import soundModule
import ioWarriorModule

# globals
# ... none

def setNeutral():
    faceModule.setEyes(0, 0)
    faceModule.setSmile(20)
    soundModule.playSoundFile('data/mood/neutral.wav')

def setHappy():
    faceModule.setEyes(9, -47)
    faceModule.setSmile(80)
    soundModule.playSoundFile('data/mood/happy.wav')

def setAngry():
    faceModule.setEyes(-46, -40)
    faceModule.setSmile(-10)
    soundModule.playSoundFile('data/mood/angry.mp3')
    ioWarriorModule.setEarColor(255,0,0)

def setSad():
    faceModule.setEyes(-1, -44)
    faceModule.setSmile(-70)
    soundModule.playSoundFile('data/mood/sad.wav')

def setTired():
    faceModule.setEyes(-1, -44)
    faceModule.setSmile(-70)
    soundModule.playSoundFile('data/mood/tired.wav')
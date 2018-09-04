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
    ioWarriorModule.setEarColor(255,255,255)

def setHappy():
    faceModule.setEyes(9, -47)
    faceModule.setSmile(80)
    soundModule.playSoundFile('data/mood/happy.wav')
    ioWarriorModule.setEarColorSeries([[255,69,0],[255,191,0]], 5, 200)

def setAngry():
    faceModule.setEyes(-46, -40)
    faceModule.setSmile(-10)
    soundModule.playSoundFile('data/mood/angry.wav')
    ioWarriorModule.setEarColorSeries([[0,0,0],[255,0,0]], 3)

def setSad():
    faceModule.setEyes(-1, -44)
    faceModule.setSmile(-70)
    soundModule.playSoundFile('data/mood/sad.wav')
    ioWarriorModule.setEarColor(0,0,255)

def setTired():
    faceModule.setEyes(23, 21)
    faceModule.setSmile(-10)
    soundModule.playSoundFile('data/mood/tired.wav')
    ioWarriorModule.setEarColor(0,0,0)
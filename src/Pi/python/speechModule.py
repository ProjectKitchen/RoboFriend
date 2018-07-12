import pyttsx
import threading

# globals
lock = threading.Lock()

def speak(text):
    global speechEngine, lock

    if not lock.locked():
        lock.acquire()
        print "speaking: " + text
        speechEngine.say(text)
        speechEngine.runAndWait()
        lock.release()

#init
print "initializing speechModule..."
speechEngine = pyttsx.init()
speechEngine.setProperty('rate', 150) #150 words per minute
speak('i am robofriend')
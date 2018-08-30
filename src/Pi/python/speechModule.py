import pyttsx
import random
import time

# globals
lastSpeakTimestamp = None
wordRate = 150 #words per minute

def speak(text, minPause = 0):
    global speechEngine, wordRate, lastSpeakTimestamp
    if lastSpeakTimestamp and time.time() - lastSpeakTimestamp > minPause:
        print "speaking: " + text
        speechEngine = pyttsx.init()
        speechEngine.setProperty('rate', wordRate)
        speechEngine.say(text)
        speechEngine.runAndWait()

def speakBatteryLow():
    minimumPause = 30 #minimum pause of 30 seconds
    possibleTexts = ['Please recharge me', 'I am tired', 'My energy is running low', 'I am feeling exhausted', 'Do you have some energy for me?', 'I am hungry']
    speak(random.choice(possibleTexts), minimumPause)

def speakBatteryShutdown():
    possibleTexts = ['I am tired. I have to go to sleep. Bye bye.', 'My energy is too low. Bye bye.']
    speak(random.choice(possibleTexts))


#init
print "initializing speechModule..."
speak('i am robofriend')
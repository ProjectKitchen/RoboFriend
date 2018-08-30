import pyttsx
import random
import time

# globals
lastSpeakTimestamp = time.time()
wordRate = 110 #words per minute
speechEngine = pyttsx.init(debug=True)
language = 'german'

def speak(text):
    global speechEngine
    if not speechEngine.isBusy():
        print "speaking: " + text
        speechEngine.say(text)
        speechEngine.runAndWait()

def speakBatteryLow():
    global lastSpeakTimestamp
    minimumPause = 30 #minimum pause of 30 seconds
    possibleTexts = ['Please recharge me', 'I am tired', 'My energy is running low', 'I am feeling exhausted', 'Do you have some energy for me?', 'I am hungry']
    if time.time() - lastSpeakTimestamp > minimumPause:
        lastSpeakTimestamp = time.time()
        speak(random.choice(possibleTexts))

def speakOnRecharge():
    possibleTexts = ['Oh. Thank you very much!', 'Ah. This is good!', 'I am feeling refreshed.', 'Yeah. This is engergy.']
    speak(random.choice(possibleTexts))

def speakBatteryShutdown():
    possibleTexts = ['I am tired. I have to go to sleep. Bye bye.', 'My energy is too low. Bye bye.']
    speak(random.choice(possibleTexts))


#init
print "initializing speechModule..."
speechEngine.setProperty('rate', wordRate)
speak('i am robofriend')
speechEngine.setProperty('voice', language)
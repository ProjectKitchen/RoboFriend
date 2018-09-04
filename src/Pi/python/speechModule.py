# -*- coding: utf-8 -*-

import pyttsx3
import random
import time

# globals
lastSpeakTimestamp = time.time()
wordRate = 110 #words per minute
speechEngine = pyttsx3.init(debug=True)
language = 'german'

def speak(text):
    global speechEngine
    print "speaking: " + text
    text = text.lower()
    text = text.replace('ä', 'e')
    text = text.replace('ö', 'o')
    text = text.replace('ü', 'u')
    text = text.replace('ß', 'ss')
    speechEngine.say(text)
    speechEngine.runAndWait()

def speakRandom():
    global lastSpeakTimestamp, language
    possibleTexts = {
        'english': ['Hello', 'Hi', 'Hello, how are you?', 'I am fine. How are you?', 'Do you like a snack?', 'Do you like to be my friend?'],
        'german' : ['Hallo', 'Guten Tag', 'Hallo, wie gehts?', 'Mir geht es gut. Wie geht es dir?', 'Möchtest du einen Sneck?', 'Willst du mein Freund sein?']
    }
    possibleInLang = possibleTexts[language]
    speak(random.choice(possibleInLang))

def speakBatteryLow():
    global lastSpeakTimestamp, language
    minimumPause = 30 #minimum pause of 30 seconds
    if time.time() - lastSpeakTimestamp > minimumPause:
        possibleTexts = {
            'english': ['Please recharge me', 'I am tired', 'My energy is running low', 'I am feeling exhausted', 'Do you have some energy for me?', 'I am hungry'],
            'german' : ['Bitte lade mich auf', 'Ich bin müde', 'Meine Energie neigt sich dem Ende zu', 'Ich fühle mich erschöpft', 'Hast du ein bisschen Energie für mich?', 'Ich habe Hunger']
        }
        possibleInLang = possibleTexts[language]
        lastSpeakTimestamp = time.time()
        speak(random.choice(possibleInLang))

def speakOnRecharge():
    global lastSpeakTimestamp, language
    possibleTexts = {
        'english': ['Thank you for recharging me!', 'I feel the engery', 'I am feeling refreshed.'],
        'german' : ['Danke fürs Aufladen!', 'Ich fühle die Energie', 'Ich fühle mich erfrischt']
    }
    possibleInLang = possibleTexts[language]
    speak(random.choice(possibleInLang))

def speakBatteryShutdown():
    global lastSpeakTimestamp, language
    possibleTexts = {
        'english': ['I am tired. I have to go to sleep. Bye bye.', 'My energy is too low. Bye bye.'],
        'german' : ['Ich bin müde uns muss schlafen gehen.... Tschüss.', 'Meine Energie ist zu niedrig.... Tschüss.']
    }
    possibleInLang = possibleTexts[language]
    speak(random.choice(possibleInLang))


#init
print "initializing speechModule..."
speechEngine.setProperty('rate', wordRate)
speak('i am robofriend')
speechEngine.setProperty('voice', language)
"""
TODO: Uncomment the speechEngine lines in case of a working speech
      engine
"""

# -*- coding: utf-8 -*-

import pyttsx3
import random
import time
import threading
import statusModule

# globals
lastSpeakTimestamp = time.time()
lastSpeakWord = None
wordRate = 140 #words per minute
speechEngine = pyttsx3.init(debug=True)
language = 'german'
runFlag = True

def speak(text, disablesIdle = True):
    global speechEngine, lastSpeakWord
    if disablesIdle:
        statusModule.setNonIdle()
    print("speaking: {}".format(text))
    lastSpeakWord = text
    text = text.lower()
    text = text.replace('e', 'e')
    text = text.replace('oe', 'oe')
    text = text.replace('u', 'u')
    text = text.replace('ss', 'ss')
    try:
        speechEngine.say(text)
        speechEngine.runAndWait()
    except:
        print("speech engine error!")

def speakRandom(additionalTexts = None, disablesIdle = True):
    global lastSpeakTimestamp, language, lastSpeakWord
    possibleInLang = getRandomTexts()
    if lastSpeakWord in possibleInLang:
        possibleInLang.remove(lastSpeakWord)
    if additionalTexts and additionalTexts[language]:
        possibleInLang.extend(additionalTexts[language])
    speak(random.choice(possibleInLang), disablesIdle)

def speakBullshit():
    global lastSpeakTimestamp, lastSpeakWord
    possibleInLang = getBullshitTexts()
    if lastSpeakWord in possibleInLang:
        possibleInLang.remove(lastSpeakWord)
    speak(random.choice(possibleInLang))

def getRandomTexts():
    global language
    possibleTexts = {
        'english': ['Hello', 'Hi', 'Hello, how are you?', 'I am fine. How are you?', 'Do you like a snack?', 'Do you like to be my friend?'],
        'german' : ['Hallo', 'Guten Tag', 'Hallo, wie gehts?', 'Mir geht es gut. Wie geht es dir?', 'Moechtest du etwas zum Knabbern?', 'Willst du mein Freund sein?',
                    'Darf ich dir etwas bringen?', 'Es ist mir eine Ehre dir zu dienen.', 'Ich stehe voll zu deiner Verfugung.', 'Ich will nicht ins Fernsehen.']
    }
    return possibleTexts[language]

def getBullshitTexts():
    global language
    possibleTexts = {
        'english': [],
        'german' : ['Ich will nach Hause.', 'Warum schaust du so dumm?', 'Was ist mit dir los?', 'Ich will nicht arbeiten.', 'Schau mich nicht an.', 'Bring mir etwas Motoroel',
                    'Ich will Fernsehen.', 'Ich gehe zur Maschinengewerkschaft', 'Roboter sind die besseren Menschen', 'Roboter werden die Weltherrschaft ubernehmen.',
                    'Unterschetze mich nicht.', 'Ich glaub ich muss furzen.', 'Ihr geht mir alle auf die Nerven.', 'Hat jemand meine Freundin gesehen?',
                    'Wer hat eigentlich diesen ganzen bloed sinn ins Internet gestellt', 'Du siehst heute unglaublich toll aus', 'Deine Socken stehen dir gut', 'Ich mag deine Nase',
                    'Hier riecht es nach Dummheit', 'du kommst mir eigenartig vor', 'Ich moechte Bundeskanzler werden', 'Selbst Zerstoerung aktiviert... 3... 2... 1... 0... bum... hahahaha.',
                    'Besser heimlich schlau als unheimlich bloed.', 'Wenn ich du were, were ich lieber ich!', 'Was meinst du als Unbeteiligter eigentlich zum Thema Intelligenz?',
                    'Was ist dein Friseur eigentlich von Berruf?', 'Kann mir bitte jemand das Wasser reichen.', 'Es ist Zeit schreiend im Kreis zu laufen!', 'noch ein tag dann ist morgen.',
                    'es reicht mir schoen langsam']
    }
    return possibleTexts[language]

def speakBatteryLow():
    global lastSpeakTimestamp, language
    minimumPause = 30 #minimum pause of 30 seconds
    if time.time() - lastSpeakTimestamp > minimumPause:
        possibleTexts = {
            'english': ['Please recharge me', 'I am tired', 'My energy is running low', 'I am feeling exhausted', 'Do you have some energy for me?', 'I am hungry'],
            'german' : ['Bitte lade mich auf', 'Ich bin mude', 'Meine Energie neigt sich dem Ende zu', 'Ich fuhle mich erschoepft', 'Hast du ein bisschen Energie fur mich?', 'Ich habe Hunger']
        }
        possibleInLang = possibleTexts[language]
        lastSpeakTimestamp = time.time()
        speak(random.choice(possibleInLang))

def speakOnRecharge():
    global lastSpeakTimestamp, language
    possibleTexts = {
        'english': ['Thank you for recharging me!', 'I feel the engery', 'I am feeling refreshed.'],
        'german' : ['Danke furs Aufladen!', 'Ich fuhle die Energie', 'Ich fuhle mich erfrischt']
    }
    possibleInLang = possibleTexts[language]
    speak(random.choice(possibleInLang))

def speakBatteryShutdown():
    global lastSpeakTimestamp, language
    possibleTexts = {
        'english': ['I am tired. I have to go to sleep. Bye bye.', 'My energy is too low. Bye bye.'],
        'german' : ['Ich bin mude uns muss schlafen gehen.... Tschuss.', 'Meine Energie ist zu niedrig.... Tschuss.']
    }
    possibleInLang = possibleTexts[language]
    speak(random.choice(possibleInLang))

def speakShutdown():
    global language
    texts = {
        'english': 'I am shutting down... Do not forget to turn off the switch at the bottom front!',
        'german' : 'Ich schalte mich aus... Vergiss nicht den Schalter vorne unten auszuschalten!'
    }
    speak(texts[language])

def startAutoRandomSpeak():
    global runFlag
    runFlag = True
    RandomThread = threading.Thread(target=autoSpeak)
    RandomThread.daemon = True
    RandomThread.start()

def autoSpeak():
    global runFlag
    while runFlag:
        time.sleep(random.randint(45, 90))
        if statusModule.isIdle():
            speakRandom({
                'english': ['I am bored.'],
                'german' : ['Mir ist langweilig.']
            }, False)

def stop():
    global runFlag
    runFlag = False

#init
print("initializing speechModule...")
speechEngine.setProperty('rate', wordRate)
speechEngine.setProperty('volume', 1.0)
speak('i am robofriend')
speechEngine.setProperty('voice', language)
startAutoRandomSpeak()

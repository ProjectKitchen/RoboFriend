# -*- coding: utf-8 -*-

import pyttsx3
import random
import time

# globals
lastSpeakTimestamp = time.time()
wordRate = 140 #words per minute
speechEngine = pyttsx3.init(debug=True)
language = 'german'

def speak(text):
    global speechEngine
    print "speaking: " + text
    text = text.lower()
    text = text.replace('ä', 'e')
    text = text.replace('ö', 'oe')
    text = text.replace('ü', 'u')
    text = text.replace('ß', 'ss')
    speechEngine.say(text)
    speechEngine.runAndWait()

def speakRandom():
    global lastSpeakTimestamp, language
    possibleTexts = {
        'english': ['Hello', 'Hi', 'Hello, how are you?', 'I am fine. How are you?', 'Do you like a snack?', 'Do you like to be my friend?'],
        'german' : ['Hallo', 'Guten Tag', 'Hallo, wie gehts?', 'Mir geht es gut. Wie geht es dir?', 'Möchtest du einen Sneck?', 'Willst du mein Freund sein?', 'Komm zu unserem Stand um mehr zu erfahren', 'Ich empfehle an der FH Technikum Wien zu studieren', 'Wer will Gummi berchen oder Soletti?', 'Darf ich Ihnen etwas bringen?', 'Es ist mir eine Ehre Ihnen zu dienen.', 'Ich will dir helfen', 'Ich stehe voll zu deiner Verfügung.']
    }
    possibleInLang = possibleTexts[language]
    speak(random.choice(possibleInLang))

def speakBullshit():
    global lastSpeakTimestamp, language
    possibleTexts = {
        'english': [],
        'german' : ['Warum schaust du so dumm?', 'Was ist mit dir los?', 'Ich will nicht arbeiten.', 'Ich will nach Hause.', 'Schau mich nicht an.', 'Bring mir etwas Motoröl', 'Ich will Fernsehen.', 'Ich gehe zur Maschinengewerkschaft', 'Roboter sind die besseren Menschen', 'Roboter werden die Weltherrschaft übernehmen.', 'Unterschetze mich nicht.', 'Ich glaub ich muss furzen.', 'Ihr geht mir alle auf die Nerven.', 'Hat jemand meine Freundin gesehen?', 'Wer hat eigentlich diesen ganzen blöd sinn ins Internet gestellt', 'Du siehst heute unglaublich toll aus', 'Deine Socken stehen dir gut', 'Ich mag deine Nase', 'Hier riecht es nach Dummheit', 'Dich habe ich schon einmal gesehen.', 'Ich möchte Bundeskanzler werden']
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
#!/usr/bin/env python3
import pyttsx3
import random
import time
import threading
import rospy
import os

# import ros message
from robofriend.msg import SpeechData
from robofriend.srv import SrvSpeechData, SrvSpeechDataResponse

def InitSpeechEngine():

    wordRate = 140                  # words per minute
    volumeRate = 1.0
    language = 'german'

    speechEngine = pyttsx3.init(debug = True)
    speechEngine.setProperty('rate', wordRate)
    speechEngine.setProperty('volume', volumeRate)
    speechEngine.setProperty('voice', language)
    return speechEngine

class SpeechDataHandler():

    random_text = {
        'english': ['Hello', 'Hi', 'Hello, how are you?', 'I am fine. How are you?', 'Do you like a snack?', 'Do you like to be my friend?', 'I am bored'],
        'german' : ['Hallo', 'Guten Tag', 'Hallo, wie gehts?', 'Mir geht es gut. Wie geht es dir?', 'Moechtest du etwas zum Knabbern?', 'Willst du mein Freund sein?',
                    'Darf ich dir etwas bringen?', 'Es ist mir eine Ehre dir zu dienen.', 'Ich stehe voll zu deiner Verfugung.', 'Ich will nicht ins Fernsehen.',
                    'Mir ist langweilig']
    }

    bullshit_text = {
        'german' : ['Ich will nach Hause.', 'Warum schaust du so dumm?', 'Was ist mit dir los?', 'Ich will nicht arbeiten.', 'Schau mich nicht an.', 'Bring mir etwas Motoroel',
                    'Ich will Fernsehen.', 'Ich gehe zur Maschinengewerkschaft', 'Roboter sind die besseren Menschen', 'Roboter werden die Weltherrschaft ubernehmen.',
                    'Unterschetze mich nicht.', 'Ich glaub ich muss furzen.', 'Ihr geht mir alle auf die Nerven.', 'Hat jemand meine Freundin gesehen?',
                    'Wer hat eigentlich diesen ganzen bloed sinn ins Internet gestellt', 'Du siehst heute unglaublich toll aus', 'Deine Socken stehen dir gut', 'Ich mag deine Nase',
                    'Hier riecht es nach Dummheit', 'du kommst mir eigenartig vor', 'Ich moechte Bundeskanzler werden', 'Selbst Zerstoerung aktiviert... 3... 2... 1... 0... bum... hahahaha.',
                    'Besser heimlich schlau als unheimlich bloed.', 'Wenn ich du were, were ich lieber ich!', 'Was meinst du als Unbeteiligter eigentlich zum Thema Intelligenz?',
                    'Was ist dein Friseur eigentlich von Berruf?', 'Kann mir bitte jemand das Wasser reichen.', 'Es ist Zeit schreiend im Kreis zu laufen!', 'noch ein tag dann ist morgen.',
                    'es reicht mir schoen langsam']
    }

    battery_low = {
        'english': ['Please recharge me', 'I am tired', 'My energy is running low', 'I am feeling exhausted', 'Do you have some energy for me?', 'I am hungry'],
        'german' : ['Bitte lade mich auf', 'Ich bin mude', 'Meine Energie neigt sich dem Ende zu', 'Ich fuhle mich erschoepft', 'Hast du ein bisschen Energie fur mich?', 'Ich habe Hunger']
    }

    battery_recharge = {
        'english': ['Thank you for recharging me!', 'I feel the engery', 'I am feeling refreshed.'],
        'german' : ['Danke furs Aufladen!', 'Ich fuhle die Energie', 'Ich fuhle mich erfrischt']
    }

    battery_shutdown = {
        'english': ['I am tired. I have to go to sleep. Bye bye.', 'My energy is too low. Bye bye.'],
        'german' : ['Ich bin mude uns muss schlafen gehen.... Tschuss.', 'Meine Energie ist zu niedrig.... Tschuss.']
    }

    idle_text = {
        'german' : ['Mir ist langweilig', 'Ich habe nichts zum tun', 'ich fadisiere mich gerade zu Tode',
                    'Kann mir bitte jemand sagen, was ich machen soll auser herumstehen!',
                    'Wenn es so weiter geht schlafe ich vor langeweile ein!']
    }

    #def __init__(self, speech_engine, language):
    def __init__(self, language):
        # self._speech_engine = speech_engine
        self._language = language
        self._time_stamp = time.time()

        self._mode = None
        self._recv_text = None
        self._last_speak_word = None

        self.speech_meth = {'random' :   self.random_speech,
                            'bullshit':  self.bullshit_speech,
                            'custom':    self.custom_speech,
                            'battery':   self.battery_speech,
                            'idle':      self.idle_speech
        }

        self.get_text = {'random':   self.get_random_text,
                         'bullshit': self.get_bullshit_text
        }

        self._old_time = 0
        self.speak("      Ich bin Robofrend")

    def service_handler(self, request):
        rospy.logdebug("{%s} - Speech Request received!",
            rospy.get_caller_id())
        self._recv_text = request.text

        if request.get_text:
            if request.mode in self.get_text:
                retVal = self.get_text[request.mode]()
                return SrvSpeechDataResponse(True, retVal)
            else:
                ropsy.logwarn("{%s} - Wrong get_text mode!",
                    rospy.get_caller_id())
        else:
            self.handle_speech(request.mode)
        return SrvSpeechDataResponse(True, [])

    def process_data(self, data):
        self._recv_text = data.text
        rospy.logdebug("{%s} - Received Data: %s, %s",
            self.__class__.__name__, data.mode, data.text)
        self.handle_speech(data.mode)

    def handle_speech(self, mode = ""):
        rospy.logdebug("{%s} - Speech Mode: %s",
            rospy.get_caller_id(), mode)
        if mode in self.speech_meth:
            self.speech_meth[mode]()
        else:
            rospy.logwarn("{%s} - Wrong speech mode!",
                rospy.get_caller_id())

    def get_random_text(self):
        retVal = None
        retVal = self.random_text[self._language].copy()
        return retVal

    def get_bullshit_text(self):
        retVal = None
        retVal = self.bullshit_text[self._language].copy()
        return retVal

    def custom_speech(self):
        self.speak(self._recv_text)

    #TODO: publish message mode = battery, text = shutdown/low/recharge in Robobrain Node
    def battery_speech(self):

        minimum_pause = 30 # minimum pause of 30 seconds

        if self._recv_text == "shutdown":
            text = self.battery_shutdown[self._language].copy()
            if self._last_speak_word in text:
                text.remove(self._last_speak_word)
            self.speak(random.choice(text))
        elif self._recv_text == "low":
            if time.time() - self._time_stamp > minimum_pause:
                text = self.battery_low[self._language].copy()
                if self._last_speak_word in text:
                    text.remove(self._last_speak_word)
                self.speak(random.choice(text))
                self._time_stamp = time.time()
            else:
                rospy.loginfo("{%s} - No 30 seconds passed",
                    self.__class__.__name__)
        elif self._recv_text == "recharge":
            text = self.battery_recharge[self._language].copy()
            if self._last_speak_word in text:
                text.remove(self._last_speak_word)
            self.speak(random.choice(text))

    def random_speech(self):
        text = self.random_text[self._language].copy()
        if self._last_speak_word in text:
            text.remove(self._last_speak_word)
        self.speak(random.choice(text))

    def bullshit_speech(self):
        text = self.bullshit_text[self._language].copy()
        if self._last_speak_word in text:
            text.remove(self._last_speak_word)
        self.speak(random.choice(text))

    def idle_speech(self):
        text = self.idle_text[self._language].copy()
        if self._last_speak_word in text:
            text.remove(self._last_speak_word)
        self.speak(random.choice(text))

    def speak(self, text):
        ret = -1

        self._last_speak_word = text
        rospy.logdebug("{%s} - Spoken text: %s", rospy.get_caller_id(), text)

        #cmd = "sudo espeak -vde \"" + text +"\""
        cmd = "espeak -vde \"" + text +"\""
        ret = os.system(cmd)

        if ret != 0:
            rospy.logwarn("{%s} - Speech error!", rospy.get_caller_id())

    def _time_request(self):
        return time.time()

def shutdown():
    rospy.loginfo("{%s} - stopping speech data handler", rospy.get_caller_id())
    rospy.signal_shutdown("Stopping Speech node!")

def Speech():
    rospy.init_node("robofriend_speech_node", log_level = rospy.INFO)
    rospy.loginfo("{%s} - starting speech node!",
        rospy.get_caller_id())

    #speech_engine = InitSpeechEngine()
    #speech = SpeechDataHandler(speech_engine, 'german')
    speech = SpeechDataHandler('german')
    rospy.Subscriber("/robofriend/speech_data", SpeechData, speech.process_data)

    # Webserver service
    rospy.Service('/robofriend/speech', SrvSpeechData, speech.service_handler)

    rospy.spin()

if __name__ == '__main__':
    try:
        Speech()
    except rospy.ROSInterruptException:
        pass

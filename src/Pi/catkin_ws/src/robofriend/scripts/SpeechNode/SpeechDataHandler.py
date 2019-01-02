# external modules
import random
import time

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


    def __init__(self, speech_engine, language):
        self._speech_engine = speech_engine
        self._language = language
        self._time_stamp = time.time()

        self._mode = None
        self._recv_text = None
        self._last_speak_word = None

    def process_data(self, data):
        self._mode = data.mode
        self._recv_text = data.text
        print("[INFO] {} - Received Data: {} {}".format(self.__class__.__name__, self._mode, self._recv_text))

        if self._mode == "custom":
            self.custom_speech()
        elif self._mode == "random":
            self.random_speech()
        elif self._mode == "bullshit":
            self.bullshit_speech()
        elif self._mode == "battery":
            self.battery_speech()
        else:
            print("[INFO] Wrong speech mode!")

    def custom_speech(self):
        self.speak(self._recv_text)

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
                    text.remove(slef._last_speak_word)
                self.speak(random.choice(text))
                self._time_stamp = time.time()
            else:
                print("[INFO] No 30 seconds passed!")
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

    def speak(self, text):
        print("Speaking Text: {}".format(text))
        self._last_speak_word = text
        try:
            self._speech_engine.say(text)
            self._speech_engine.runAndWait()
        except:
            print("[INFO] Speech engine error!")

    # mode
    @property
    def mode(self):
        return self._mode

    # texts
    @property
    def text(self):
        return self._text

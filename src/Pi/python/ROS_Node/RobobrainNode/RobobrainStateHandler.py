from threading import Lock, Thread
from time import sleep

class RobobrainStateHandler():

    robostate = {
        'IDLE' : 1, \
        'FIND_CHARGING_STATION' : 2, \
        'AUTONOM' : 3, \
        'MANUAL' : 4
    }

    idle_elapse_time = 90         # waits 90 sec to change state if no input from webserver and keyboard

    def __init__(self, event):
        self.__state = RobobrainStateHandler.robostate["IDLE"]
        self.__lock = Lock()
        self.__start_thread()
        self.__event = event

    def __start_thread(self):
        thread = Thread(target = self.__state_hander_thread)
        thread.daemon = True
        thread.start()

    def __state_hander_thread(self):
        print("{} - Thread to handle the states started!".format(__class__.__name__))
        while True:
            if self.__state == RobobrainStateHandler.robostate["IDLE"]:
                print("{} - Waiting for input or timeout within {} seconds".format(__class__.__name__, RobobrainStateHandler.idle_elapse_time))
                event_is_set = self.__event.wait(RobobrainStateHandler.idle_elapse_time)
                if event_is_set == True:
                    print("Input from either keyboard or webserver therefore stay in IDLE state!")
                    self.__event.clear()
                else:
                    print("No input within {} sec therefore change state!".format(RobobrainStateHandler.idle_elapse_time))

    @property
    def state(self):
        self.__lock_acquire()
        retVal = None
        retVal = self.__state
        self.__lock_release()
        return retVal

    @state.setter
    def state(self, value):
        self.__lock_aquire()
        self.__state = value
        self.__lock_release()

    def __lock_acquire(self):
        print("Lock aquired!")
        self.__lock.acquire()

    def __lock_release(self):
        print("Lock released!")
        self.__lock.release()

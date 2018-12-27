from threading import Lock, Thread
from time import sleep

class RobobrainStateHandler():

    robostate = {
        'IDLE' : 1, \
        'FIND_CHARGING_STATION' : 2, \
        'AUTONOM' : 3, \
        'MANUAL' : 4
    }

    def __init__(self):
        self.__state = RobobrainStateHandler.robostate["IDLE"]
        self.__lock = Lock()

    def start_thread(self):
        thread = Thread(target = self.__state_hander_thread)
        thread.daemon = True
        thread.start()

    def __state_hander_thread(self):
        print("{} - Thread to handle the states started!".format(__class__.__name__))
        while True:
            print("Thread is ongoing")
            sleep(3)



    @property
    def state(self):
        retVal = None
        self.__lock_aquire()
        retVal = self.__state
        self.__lock_release()
        return retVal

    @state.setter
    def state(self, value):
        self.__lock_aquire()
        self.__state = value
        self.__lock_release()

    def __lock_aquire():
        print("Lock aquired!")
        self.__lock.aquire()

    def __lock_release():
        print("Lock released!")
        self.__lock.release()

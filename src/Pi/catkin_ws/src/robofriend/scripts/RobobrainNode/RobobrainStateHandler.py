from threading import Lock, Thread
from time import sleep

class RobobrainStateHandler():

    robostate = {
        'ADMIN' : 0, \
        'CHARGE' : 1, \
        'FACEDETECTION' : 2, \
        'MANUAL' : 3, \
        'AUTONOM' : 4, \
        'IDLE' : 5, \
        'SHUTDOWN' : 6
    }

    def __init__(self, event):
        self.__state = RobobrainStateHandler.robostate["IDLE"]
        self.__lock = Lock()
        self.__start_thread()
        self.__event = event
        self.__idle_elapse_time = 90         # waits 90 sec to change state if no input from webserver and keyboard

    def __start_thread(self):
        thread = Thread(target = self.__state_handler_thread)
        thread.daemon = True
        thread.start()

    def __state_handler_thread(self):
        print("{} - Thread to handle the states started!".format(self.__class__.__name__))
        while True:
            # ************************* SHUTDOWN *************************
            if self.__state == RobobrainStateHandler.robostate["SHUTDOWN"]:
                print("[INFO] Within SHUTDOWN state")
                # TODO: Speech module publishen
                # sleep(5)
                # os.system("init 0")
            # *************************** IDLE *************************** '''
            elif self.__state == RobobrainStateHandler.robostate["IDLE"]:
                print("[INFO] Within IDLE state")
                event_is_set = self.__event.wait(self.__idle_elapse_time)
                if event_is_set == True:
                    print("Input from either keyboard or webserver therefore stay in IDLE state!")
                    self.__event.clear()
                else:
                    print("No input within {} sec therefore change state to FACEDETECTION state!".format(self.__idle_elapse_time))
                    self.__state = RobobrainStateHandler.robostate["FACEDETECTION"]
            # ************************* AUTONOM ************************** '''
            elif self.__state == RobobrainStateHandler.robostate["AUTONOM"]:
                print("[INFO] Within AUTONOM state")
            # ************************** MANUAL ************************** '''
            elif self.__state == RobobrainStateHandler.robostate["MANUAL"]:
                print("[INFO] Within MANUAL state")
            # *********************** FACEDETECTION ********************** '''
            elif self.__state == RobobrainStateHandler.robostate["FACEDETECTION"]:
                print("[INFO] Within FACEDETECTION state")
                sleep(600)
                #TODO: start facedetection process
            # ************************** CHARGE ************************** '''
            elif self.__state == RobobrainStateHandler.robostate["CHARGE"]:
                print("[INFO] CHARGE state")
            # ************************** ADMIN *************************** '''
            elif self.__state == RobobrainStateHandler.robostate["ADMIN"]:
                print("[INFO] ADMIN state")
            # ************************* UNKNOWN ************************** '''
            else:
                print("[ERROR] Robobrain state unknown!")

    @property
    def state(self):
        self.__lock_acquire()
        retVal = None
        retVal = self.__state
        self.__lock_release()
        return retVal

    @state.setter
    def state(self, value):
        self.__lock_acquire()
        self.__state = value
        self.__lock_release()

    def __lock_acquire(self):
        # print("Lock aquired!")
        self.__lock.acquire()

    def __lock_release(self):
        # print("Lock released!")
        self.__lock.release()
import os, rospy, subprocess

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
        self.__batWasLow = False
        self.__lock = Lock()
        self.__idle_keyb_event = event
        self.__idle_elapse_time = 40         # waits defined sec to change state if no input from webserver and keyboard
        self.__start_thread()

    def __start_thread(self):
        thread = Thread(target = self.__state_handler_thread)
        thread.daemon = True
        thread.start()

    def __state_handler_thread(self):
        # print("{} - Thread to handle the states started!".format(self.__class__.__name__))
        while True:
            # ************************* SHUTDOWN *************************
            if self.state == RobobrainStateHandler.robostate["SHUTDOWN"]:
                rospy.logwarn("{%s} - within shutdown state", rospy.get_caller_id())
                # TODO: speakBatteryShutdown()
                sleep(5)
                subprocess.call(["sudo", "init", "0"])
            # *************************** IDLE *************************** '''
            elif self.state == RobobrainStateHandler.robostate["IDLE"]:
                rospy.loginfo("{%s} - within idle state", rospy.get_caller_id())
                self.__batWasLow = False
                # speakOnRecharge()
                event_is_set = self.__idle_keyb_event.wait(self.__idle_elapse_time)
                if event_is_set == True:
                    rospy.logdebug("{%s} - Input from keyboard node received therefore stay in IDLE Stat!")
                    self.__idle_keyb_event.clear()
                else:
                    print("No input within {} sec therefore change state to FACEDETECTION state!".format(self.__idle_elapse_time))
                    self.state = RobobrainStateHandler.robostate["FACEDETECTION"]
            # ************************* AUTONOM ************************** '''
            elif self.state == RobobrainStateHandler.robostate["AUTONOM"]:
                rospy.loginfo("{%s} - within autonom state", rospy.get_caller_id())
            # ************************** MANUAL ************************** '''
            elif self.state == RobobrainStateHandler.robostate["MANUAL"]:
                rospy.loginfo("{%s} - within manual state", rospy.get_caller_id())
            # *********************** FACEDETECTION ********************** '''
            elif self.state == RobobrainStateHandler.robostate["FACEDETECTION"]:
                rospy.loginfo("{%s} - within face detection state", rospy.get_caller_id())
                sleep(10)
            # ************************** CHARGE ************************** '''
            elif self.state == RobobrainStateHandler.robostate["CHARGE"]:
                rospy.loginfo("{%s} - within charge state", rospy.get_caller_id())
                self.__batWasLow = True
                # TODO: speakBatteryLow()
                # TODO: publish message to speach Node: mode: battery / text = low / recharge
            # ************************** ADMIN *************************** '''
            elif self.state == RobobrainStateHandler.robostate["ADMIN"]:
                rospy.loginfo("{%s} - within admin state", rospy.get_caller_id())
            # ************************* UNKNOWN ************************** '''
            else:
                rospy.logerr("{%s} - robot state unknown", rospy.get_caller_id())

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

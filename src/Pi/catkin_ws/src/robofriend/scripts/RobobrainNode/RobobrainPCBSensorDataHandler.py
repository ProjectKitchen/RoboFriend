import rospy
from RobobrainStateHandler import *

class RobobrainPCBSensorDataHandler():

    def __init__(self):
        self._voltage = 0
        self._percentage = 0
        self._power_supply_status = 0
        self._inf_left = 0
        self._inf_middle = 0
        self._inf_right = 0

        # TODO: remove
        self._batWasLow = None
        # self._robostate_obj = robostate

    def process_bs_data(self, data):
        rospy.loginfo("{%s} Volt: %.2fV Perc: %.2f%% Status: %d", 
            self.__class__.__name__,
            self._voltage,
            self._percentage * 100,
            self._power_supply_status)

        self._voltage = round(data.voltage, 2)
        self._percentage = data.percentage
        self._power_supply_status = data.power_supply_status

        # TODO: evaluate somewhere else
        # self.__data_evaluate()
    
    def process_ir_data(self, data):
        rospy.loginfo("{%s} Left: %.2f Middle: %.2f Right: %.2f", 
            self.__class__.__name__,
            self._inf_left,
            self._inf_middle,
            self._inf_right)

        self._inf_left = data.inf_left
        self._inf_middle = data.inf_middle
        self._inf_right = data.inf_right

    def __data_evaluate(self):
        if self._bat_value < 12 and self._bat_value > 11.8:
            self._batWasLow = True
            print("[INFO] {} - Battery at low level change state to FIND_CHARGING_STATION!".format(self.__class__.__name__))
            self.__robostate_obj.state = RobobrainStateHandler.robostate["FIND_CHARGING_STATION"]
        elif self._bat_value < 11.8:
            print("[INFO] {} - Battery reached critical level, going shutdown!".format(self.__class__.__name__))
            self.__robostate_obj.state = RobobrainStateHandler.robostate["SHUTDOWN"]
        if self._batWasLow == True and self._bat_value > 12.15:
            batwWasLow = False
            #print("[INFO] Robofriend is recharged!")

import rospy
from RobobrainStateHandler import *

class RobobrainPCBSensorDataHandler():

    def __init__(self):
        self._bat_value = None
        self._bat_percent = None
        # self._inf_left = None
        # self._inf_middle = None
        # self._inf_right = None

        self._voltage = 0
        self._percentage = 0
        self._power_supply_status = 0
        self._power_supply_health = 0

        self._batWasLow = None
        # self._robostate_obj = robostate

    def process_data(self, data):
        rospy.loginfo("{%s} Volt: %.2fV Perc: %.2f%% Status: %d Health: %d", 
            self.__class__.__name__,
            self._voltage,
            self._percentage * 100,
            self._power_supply_status, 
            self._power_supply_health)

        self._voltage = round(data.voltage, 2)
        self._percentage = data.percentage
        self._power_supply_status = data.power_supply_status
        self._power_supply_health = data.power_supply_health

        # self._inf_left = round(data.inf_left, 2)
        # self._inf_middle = round(data.inf_middle, 2)
        # self._inf_right = round(data.inf_right, 2)
        # print("[INFO] {} - Received message from Battery/Infrared Node : {}, {}, {}, {}, {}\n"
        #         .format(self.__class__.__name__, self._bat_value, self._bat_percent, self._inf_left,
        #         self._inf_middle, self._inf_right))

        # TODO: evaluate somewhere else
        # self.__data_evaluate()

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

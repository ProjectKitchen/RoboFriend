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

    def process_bs_data(self, data):
        rospy.loginfo("{%s} Battery Data Volt: %.2fV Perc: %.2f%% Status: %d", 
            self.__class__.__name__,
            self._voltage,
            self._percentage * 100,
            self._power_supply_status)

        self._voltage = round(data.voltage, 2)
        self._percentage = data.percentage
        self._power_supply_status = data.power_supply_status
    
    def process_ir_data(self, data):
        rospy.loginfo("{%s} Infrared Data Left: %.2f Middle: %.2f Right: %.2f", 
            self.__class__.__name__,
            self._inf_left,
            self._inf_middle,
            self._inf_right)

        self._inf_left = data.inf_left
        self._inf_middle = data.inf_middle
        self._inf_right = data.inf_right

    @property
    def power_supply_status(self):
        return self._power_supply_status

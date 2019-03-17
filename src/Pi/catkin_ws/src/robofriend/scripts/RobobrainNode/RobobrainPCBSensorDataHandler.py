import os, sys, rospy

path = os.path.dirname(os.path.abspath(__file__)) + "/.."
sys.path.append(path)
import constants

from RobobrainStateHandler import *

class RobobrainPCBSensorDataHandler(object):
    def __init__(self, sh):
        self._statehandler = sh
        self._voltage = 0
        self._percentage = 0
        self._power_supply_status = 0

    def process_data(self, data):
        rospy.logdebug("{%s} Battery Data Volt: %.2fV Perc: %.2f%% Status: %d",
            rospy.get_caller_id(),
            self._voltage,
            self._percentage * 100,
            self._power_supply_status)

        self._voltage = round(data.voltage, 2)
        self._percentage = data.percentage
        self._power_supply_status = data.power_supply_status

        self._set_robo_state()

    def _set_robo_state(self):
        if self._power_supply_status == constants.BAT_OVERCHARGED:
            rospy.logdebug("{%s} - Battery overcharged", rospy.get_caller_id())
        elif self._power_supply_status == constants.BAT_FULL:
            rospy.logdebug("{%s} - Battery full", rospy.get_caller_id())
        elif self._power_supply_status == constants.BAT_GOOD:
            rospy.logdebug("{%s} - Battery good", rospy.get_caller_id())
        elif self._power_supply_status == constants.BAT_WARNING:
            rospy.logdebug("{%s} - Battery warning", rospy.get_caller_id())
        elif self._power_supply_status == constants.BAT_CRITICAL:
            rospy.logdebug("{%s} - Battery critical", rospy.get_caller_id())
        elif self._power_supply_status == constants.BAT_UNKNOWN:
            rospy.logdebug("{%s} - Battery Unknown", rospy.get_caller_id())

        # TODO: only change in state tarnsission

        if self._power_supply_status == constants.BAT_WARNING:
            self._statehandler.state = constants.RF_CHARGE
        elif self._power_supply_status == constants.BAT_CRITICAL:
            self._statehandler.state = constants.RF_SHUTDOWN
        # else:
        #     self._statehandler.state = constants.RF_IDLE

#!/usr/bin/env python
import rospy
import constants

# import ros services
from robofriend.srv import SrvPCBSensorData

# import ros message
from robofriend.msg import IRSensorData
from sensor_msgs.msg import BatteryState

class PCBSensorDataHandler(object):
    def __init__(self, pub1, pub2):
        self._bs_pub = pub1
        self._ir_pub = pub2

        self._recv_bat_val = 0
        self._battery_moving_average = 30
        self._battery_moving_average_val = 0

    def processSensorValues(self, args):
        if args is None:
            return

        self._vbat_act = constants.ADC_INTERNAL_VREF / float(constants.ADC_RESOLUTION) * args.bat_voltage
        self._recv_bat_val = (self._vbat_act * (constants.VOLT_DIV_R1 + constants.VOLT_DIV_R2)) / constants.VOLT_DIV_R2
        self._battery_moving_average_val = self._getMovingAverage(
            self._recv_bat_val, 
            self._battery_moving_average_val, 
            self._battery_moving_average)

        # http://docs.ros.org/api/sensor_msgs/html/msg/BatteryState.html
        bs_data = BatteryState()
        bs_data.voltage = self._battery_moving_average_val
        bs_data.percentage = self._get_battery_percent(self._battery_moving_average_val)
        bs_data.power_supply_status = self._get_power_supply_status(bs_data.percentage)
        
        ir_data = IRSensorData()
        ir_data.inf_left = args.inf_left
        ir_data.inf_middle = args.inf_middle
        ir_data.inf_right = args.inf_right

        rospy.logdebug("{%s} BatteryState Data:\n%s", 
            self.__class__.__name__,
            bs_data)
        rospy.logdebug("{%s} IR Data:\n%s", 
            self.__class__.__name__,
            ir_data)

        # publish message to robobrain node
        self._bs_pub.publish(bs_data)
        self._ir_pub.publish(ir_data)

    def _getMovingAverage(self, newValue, currentMean, n):
        if not currentMean:
            return newValue
        newMean = currentMean - currentMean / n
        newMean = newMean + newValue / n
        return newMean

    def _get_battery_percent(self, batVoltage):
        batVoltageRounded = round(batVoltage, 2)

        # dont multiply by 100 cause sensor_msgs/BatteryState.percentage
        # is the charge percentage on 0 to 1 range
        percent = round(
            (batVoltageRounded - constants.BAT_LOWWER_THREDSHOLD) / 
            (constants.BAT_UPPER_THRESHOLD - constants.BAT_LOWWER_THREDSHOLD), 2)
        return percent

    def _get_power_supply_status(self, val):
        # http://docs.ros.org/jade/api/sensor_msgs/html/msg/BatteryState.html
        if val > 1.0:
            return constants.BAT_OVERCHARGED
        elif val <= 1.0 and val > 0.80:
            return constants.BAT_FULL
        elif val <= 0.80 and val > 0.20:
            return constants.BAT_GOOD
        elif val <= 0.20 and val > 0.05:
            return constants.BAT_WARNING
        elif val <= 0.5 and val > 0.0:
            return constants.BAT_CRITICAL
        else:
            return constants.BAT_UNKOWN

def shutdown():
    rospy.signal_shutdown("Stopping PCB Sensor Data Handler node!")

def SensorData():
    rospy.init_node("robofriend_pcb_sensor_data", log_level = rospy.INFO)
    rospy.loginfo("Starting PCB Sensor Data Handler node!")

    bs_pub = rospy.Publisher("/robofriend/battery_state", BatteryState, queue_size = 2)
    ir_pub = rospy.Publisher("/robofriend/infrared_data", IRSensorData, queue_size = 2)

    dh = PCBSensorDataHandler(bs_pub, ir_pub)

    rate = rospy.Rate(1) # 1hz

    while not rospy.is_shutdown():
        srv_resp = None
        rospy.wait_for_service('/robofriend/get_pcb_sensor_data')

        try:
            request = rospy.ServiceProxy('/robofriend/get_pcb_sensor_data', SrvPCBSensorData)
            srv_resp = request(True)
        except rospy.ServiceException:
            rospy.logwarn("{%s} - Service call failed", self.__class__.__name__)

        dh.processSensorValues(srv_resp) 

        rate.sleep() # make sure the publish rate maintains at the needed frequency

if __name__ == '__main__':
    try:
        SensorData()
    except rospy.ROSInterruptException:
        pass
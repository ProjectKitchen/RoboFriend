#!/usr/bin/env python
import rospy

# import ros services
from robofriend.srv import SrvPCBSensorData

# import ros message
from sensor_msgs.msg import BatteryState
from robofriend.msg import IRSensorData

class PCBSensorDataHandler(object):
    def __init__(self, pub1, pub2):
        self._bs_pub = pub1
        self._ir_pub = pub2
        # http://www.peacesoftware.de/einigewerte/spannungsteiler.html
        # Spannung U1: 14.7
        # Spannung U2: 4.096
        # Rges: 10000
        # Widerstandsreihe
        # Parall/Serienschaltung zulassen
        # Siehe also schematic 
        self._r1 = 7210
        self._r2 = 2780

        self._recv_bat_val = 0
        self._battery_moving_average = 30
        self._battery_moving_average_val = 0
        self._battery_over_volt = 14.7
        self._battery_discharge = 10.5

    def processSensorValues(self, args):
        if args is None:
            return

        self._recv_bat_val = (args.bat_voltage * (self._r1 + self._r2)) / self._r2
        self._battery_moving_average_val = self.__getMovingAverage(
            self._recv_bat_val, 
            self._battery_moving_average_val, 
            self._battery_moving_average)

        # http://docs.ros.org/api/sensor_msgs/html/msg/BatteryState.html
        bs_data = BatteryState()
        bs_data.voltage = self._battery_moving_average_val
        bs_data.percentage = self.__get_battery_percent(self._battery_moving_average_val)
        bs_data.power_supply_status = 0
        bs_data.power_supply_health = 1
        
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

    def __getMovingAverage(self, newValue, currentMean, n):
        if not currentMean:
            return newValue
        newMean = currentMean - currentMean / n
        newMean = newMean + newValue / n
        return newMean

    def __get_battery_percent(self, batVoltage):
        batVoltageRounded = round(batVoltage, 2)
        # dont multiply by 100 cause sensor_msgs/BatteryState.percentage
        # is the charge percentage on 0 to 1 range

        percent = round(
            (batVoltageRounded - self._battery_discharge) / 
            (self._battery_over_volt - self._battery_discharge), 2)
        return percent

def shutdown():
    rospy.signal_shutdown("Stopping PCB Sensor Data Handler node!")

def SensorData():
    rospy.init_node("robofriend_pcb_sensor_data")
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
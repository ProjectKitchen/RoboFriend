#!/usr/bin/env python
import rospy

# import ros services
from robofriend.srv import SrvPCBSensorData

# import ros message
from robofriend.msg import PCBSensorData

class PCBSensorDataHandler(object):
    def __init__(self, pub):
        self._pub = pub
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
        self._battery_over_volt = 14,7
        self._battery_discharge = 10,5

    def processSensorValues(self, args):
        if args is None:
            return

        self._recv_bat_val = (args.bat_voltage * (r1 + r2)) / r2
        self._battery_moving_average_val = __getMovingAverage(
            self._recv_bat_val, 
            self._battery_moving_average_val, 
            self._battery_moving_average)

        data = PCBSensorData()
        data.bat_voltage = self._battery_moving_average_val
        data.bat_percent = self.__get_battery_percent(self._battery_moving_average_val)
        data.inf_left = args.inf_left
        data.inf_middle = args.inf_middle
        data.inf_right = args.inf_right

        rospy.logdebug("{%s} - Updated battery value: %f", 
            self.__class__.__name__, 
            data.bat_voltage
            )

        # publish message to robobrain node
        self._pub.publish(data)

    def __getMovingAverage(newValue, currentMean, n):
        if not currentMean:
            return newValue
        newMean = currentMean - currentMean / n
        newMean = newMean + newValue / n
        return newMean

    def __get_battery_percent(batVoltage):
        batVoltageRounded = round(batVoltage, 2)
        percent = round(
            (batVoltageRounded - self._battery_discharge) / 
            (self._battery_over_volt - self._battery_discharge) * 
            100, 2)
        return percent

def shutdown():
    rospy.signal_shutdown("Stopping PCB Sensor Data Handler node!")

def SensorData():
    rospy.init_node("robofriend_pcb_sensor_data")
    rospy.loginfo("Starting PCB Sensor Data Handler node!")

    pub = rospy.Publisher("/robofriend/pcb_sensor_data", PCBSensorData, queue_size = 2)

    dh = PCBSensorDataHandler(pub)

    rate = rospy.Rate(1) # 1hz

    while not rospy.is_shutdown():
        srv_resp = None
        rospy.wait_for_service('/robofriend/get_pcb_sensor_data')

        try:
            request = rospy.ServiceProxy('/robofriend/get_pcb_sensor_data', PCBSensorData)
            srv_resp = request(True)
        except rospy.ServiceException:
            rospy.logwarn("{%s} - Service call failed", self.__class__.__name__)

        dh.processSensorValues(srv_resp) 

        rate.sleep() # make sure the publish rate maintains at the needed frequency

if __name__ == '__main__':
    try:
        rospy.loginfo("We start here")
        SensorData()
    except rospy.ROSInterruptException:
        pass
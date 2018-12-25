import rospy

# import ros services
from ros_robofriend.srv import BatInfData

# import ros message
from ros_robofriend.msg import BatInfMsgData

def getBatPercentMapping():
    return [[12.77,100], [12.76,99], [12.75,99], [12.74,99], [12.73,98], [12.72,98], [12.71,96], [12.7,95], [12.69,93],
            [12.68,91], [12.67,89], [12.66,86], [12.65,84], [12.64,83], [12.63,82], [12.62,81], [12.61,80], [12.6,78],
            [12.59,77], [12.58,75], [12.57,74], [12.56,73], [12.55,71], [12.54,70], [12.53,68], [12.52,67], [12.51,66],
            [12.5,65], [12.49,64], [12.48,63], [12.47,62], [12.46,61], [12.45,60], [12.44,59], [12.43,58], [12.42,57],
            [12.41,56], [12.4,54], [12.39,53], [12.38,51], [12.37,49], [12.36,48], [12.35,47], [12.34,46], [12.33,45],
            [12.32,44], [12.31,43], [12.3,41], [12.29,40], [12.28,39], [12.27,39], [12.26,38], [12.25,37], [12.24,36],
            [12.23,35], [12.22,35], [12.21,34], [12.2,34], [12.19,33], [12.18,32], [12.17,31], [12.16,30], [12.15,29],
            [12.14,28], [12.13,27], [12.12,25], [12.11,25], [12.1,24], [12.09,23], [12.08,22], [12.07,22], [12.06,21],
            [12.05,20], [12.04,20], [12.03,19], [12.02,18], [12.01,18], [12,17], [11.99,16], [11.98,15], [11.97,15],
            [11.96,14], [11.95,13], [11.94,12], [11.93,11], [11.92,10], [11.91,9], [11.9,9], [11.89,8], [11.88,7],
            [11.87,6], [11.86,6], [11.85,5], [11.84,4], [11.83,3], [11.82,2], [11.81,2], [11.8,1], [11.79,1], [11.78,0]]

def get_battery_percent(batVoltage):
    batVoltageRounded = round(batVoltage, 2)
    mapping = getBatPercentMapping()
    percent = 100
    for pair in mapping:
        if batVoltageRounded <= pair[0]:
            percent = pair[1]
    return percent

class BatteryInfraredDataHandler():

    def __init__(self):
        self._sensor_val = {
            "battery" : None, \
            "bat_percent" : None, \
            "inf_left" : None, \
            "inf_middle" : None, \
            "inf_right" : None
        }

        self._status_cnt = 0
        self._actual_bat = None
        self._prev_bat_val = None
        self._battery_average = 30
        self._battery_constant = 0.04783948

        self._pub = rospy.Publisher("T_BAT_INF_DATA", BatInfMsgData, queue_size = 10)
        self._msg = BatInfMsgData()

    def request_sensor_values(self):
        service_resp = None
        rospy.wait_for_service('S_BAT_INF_DATA')

        try:
            request = rospy.ServiceProxy('S_BAT_INF_DATA', BatInfData)
            service_resp = request(True)
            # print("[INFO] {} - Sensor values received: Battery: {}, Infrared left: {},\
            #         Infrared middle: {}, Infrared right: {}"
                   .format(__class__.__name__, service_resp.bat, service_resp.inf_left,
                   service_resp.inf_middle, service_resp.inf_right))
        except rospy.ServiceException:
            print("[INFO] {} - Service call failed!".format(__class__.__name__))

        self.__process_sensor_values(service_resp)

    def __process_sensor_values(self, sensor_values):
        self._status_cnt += 1
        #print("[INFO] Status Count: {}".format(self._status_cnt))

        self._actual_bat = self.__update_battery_voltage(sensor_values.bat, self._actual_bat, self._battery_average)
        self._sensor_val["battery"] = self._actual_bat
        self._sensor_val["bat_percent"] = get_battery_percent(self._actual_bat)
        self._sensor_val["inf_left"] = sensor_values.inf_left
        self._sensor_val["inf_middle"] = sensor_values.inf_middle
        self._sensor_val["inf_right"] = sensor_values.inf_right

        #print("[INFO] {} - Updated battery value: {}\n".format(__class__.__name__, self._sensor_val["battery"]))

        # publish message to robobrain node
        if self._status_cnt >= self._battery_average:
            self.__publish_message(self._sensor_val)


        # if self._status_cnt >= self._battery_average and self._sensor_val["battery"] < 12.0:
        #     batwWasLow = True
        #     print("[INFO] {} - Battery at low level, recharge!")
        #     self._pub.publish("")
        #     #TODO: publish message that battery is at low level
        # elif self._status_cnt >= self._battery_average and self._sensor_val["battery"] < 11.8:
        #     print("[INFO] {} - Battery reached lowesr level, going shutdown!")
        #     #TODO: publish message shutdown ==> look keyoard node_stop
        # elif batwWasLow == True and self._sensor_val["battery"] > 12.15:
        #     batwWasLow = False
        #     print("[INFO] Robofriend is recharged!")

    def __update_battery_voltage(self, new_bat, actual_bat, battery_average):
        if actual_bat == None:
            actual_bat = 1000

        new_bat *= self._battery_constant
        newMean = actual_bat - (actual_bat / battery_average)
        return (newMean + (new_bat / battery_average))

    def __publish_message(self, sensor_val):
        self._msg.bat_val = round(sensor_val["battery"], 3)
        self._msg.inf_left = round(sensor_val["inf_left"], 3)
        self._msg.inf_middle = round(sensor_val["inf_middle"], 3)
        self._msg.inf_right = round(sensor_val["inf_right"], 3)
        #print("[INFO] {} - Published sensor values to robobrain node: {}\n".format(__class__.__name__, self._msg))
        self._pub.publish(self._msg)

    # battery
    @property
    def bat(self):
        return self._bat

    @bat.setter
    def bat(self, value):
        self._bat = value

    # infrared left
    @property
    def inf_left(self):
        return self._inf_left

    @inf_left.setter
    def inf_left(self, value):
        self._inf_left = value

    # infrared middle
    @property
    def inf_middle(self):
        return self._inf_middle

    @inf_middle.setter
    def inf_middle(self, value):
        self._inf_middle = value

    # infrared right
    @property
    def inf_right(self):
        return self._inf_right

    @inf_right.setter
    def inf_right(self, value):
        self._inf_rightt = value
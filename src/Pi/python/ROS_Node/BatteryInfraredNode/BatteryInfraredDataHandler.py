import rospy

# import ros services
from ros_robofriend.srv import BatInfData

class BatteryInfraredDataHandler():

    def __init__(self):
        self._bat = None
        self._inf_left = None
        self._inf_middle = None
        self._inf_right = None

    def request_sensor_values(self):
        service_resp = None
        rospy.wait_for_service('S_BAT_INF_DATA')

        try:
            request = rospy.ServiceProxy('S_BAT_INF_DATA', BatInfData)
            # bat, inf_left, inf_middle, inf_right = request(True)
            service_resp = request(True)
            print("[INFO] {} - Sensor values received: Battery: {}, Infrared left: {},\
Infrared middle: {}, Infrared right: {}"
                    .format(__class__.__name__, service_resp.bat, service_resp.inf_left,
                    service_resp.inf_middle, service_resp.inf_right))

            self.bat = service_resp.bat
            self.inf_left = service_resp.inf_left
            self.inf_midde = service_resp.inf_middle
            self.inf_right = service_resp.inf_right
        except rospy.ServiceException:
            print("[INFO] {} - Service call failed!".format(__class__.__name__))

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

import rospy

class BatteryVoltageDataHandler():

    def __init__(self):
        self._voltage = 0

    def processData(self, data):
        self._voltage = data.data
        rospy.loginfo('{%s} DATA: %s', self.__class__.__name__, self._voltage)

    @property
    def voltage(self):
        return self._voltage

    @voltage.setter
    def voltage(self, value):
        self._voltage = value
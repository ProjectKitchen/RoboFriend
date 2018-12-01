import rospy

class OdometryDataHandler():

    def __init__(self):
        self._x = 0
        self._y = 0
        self._z = 0

    def processData(self, data):
    	self._x = data.x
        self._y = data.y
        self._z = data.z
        rospy.loginfo('{%s} DATA: %s %s %s', self.__class__.__name__, self._x, self._y, self._z)                                    

    # x
    @property
    def x(self):
        return self._x

    @x.setter
    def x(self, value):
        self._x = value

    @x.deleter
    def x(self):
        del self._x

    # y
    @property
    def y(self):
        return self._y

    @y.setter
    def y(self, value):
        self._y = value

    @y.deleter
    def y(self):
        del self._y

    # z
    @property
    def z(self):
        return self._z

    @z.setter
    def z(self, value):
        self._z = value

    @z.deleter
    def z(self):
        del self._z

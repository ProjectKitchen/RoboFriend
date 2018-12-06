# can use this params to check whether we hit a wall or an obstacle i.e.

import rospy

class OdometryDataHandler():

    def __init__(self):
        self._x = 0
        self._y = 0
        self._theta = 0
        self._linear_velocity = 0
        self._angular_velocity = 0

    def processData(self, data):
    	self._x = data.x
        self._y = data.y
        self._theta = data.theta
        self._linear_velocity = data.linear_velocity
        self._angular_velocity = data.angular_velocity
        rospy.loginfo('{%s} DATA: %f %f %f %s %s', 
        				self.__class__.__name__, 
        				self._x, 
        				self._y, 
        				self._theta,
        				self._linear_velocity,
        				self._angular_velocity
        				)                                    

    # x
    @property
    def x(self):
        return self._x

    @x.setter
    def x(self, value):
        self._x = value

    # y
    @property
    def y(self):
        return self._y

    @y.setter
    def y(self, value):
        self._y = value

    # theta
    @property
    def theta(self):
        return self._theta

    @theta.setter
    def theta(self, value):
        self._theta = value

    # linear_velocity
    @property
    def linear_velocity(self):
        return self._linear_velocity

    @linear_velocity.setter
    def linear_velocity(self, value):
        self._linear_velocity = value

    # angular_velocity
    @property
    def angular_velocity(self):
        return self._angular_velocity

    @angular_velocity.setter
    def angular_velocity(self, value):
        self._angular_velocity = value

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
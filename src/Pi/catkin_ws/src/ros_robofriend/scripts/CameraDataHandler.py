import rospy

class CameraDataHandler():

    def __init__(self):
        self._top = 0
        self._right = 0
        self._bottom = 0
        self._left = 0
        self._name = None

    def processData(self, data):
    	self._top = data.top
        self._right = data.right
        self._bottom = data.bottom
        self._left = data.left
        self._name = data.name
        rospy.loginfo('{%s} DATA: %s %s %s %s %s', self.__class__.__name__, self._top, self.right, self._bottom, self._left, self._name)                                    

    # top
    @property
    def top(self):
        return self._top

    @top.setter
    def top(self, value):
        self._top = value

    # right
    @property
    def right(self):
        return self._right

    @right.setter
    def right(self, value):
        self._right = value

    # bottom
    @property
    def bottom(self):
        return self._bottom

    @bottom.setter
    def bottom(self, value):
        self._bottom = value

    # left
    @property
    def left(self):
        return self._left

    @left.setter
    def left(self, value):
        self._left = value

    # name
    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, value):
        self._name = value


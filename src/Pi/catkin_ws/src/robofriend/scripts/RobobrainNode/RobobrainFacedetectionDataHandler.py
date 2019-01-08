import rospy

class RobobrainFacedetectionDataHandler():

    def __init__(self):
        self._top = 0
        self._right = 0
        self._bottom = 0
        self._left = 0
        self._name = None

    def process_data(self, data):
        self._top = data.top
        self._right = data.right
        self._bottom = data.bottom
        self._left = data.left
        self._name = data.name
        rospy.loginfo("{%s} name: %s, top: %d, right: %d, bottom: %d, left: %d", 
            self.__class__.__name__,
            self._name,
            self._top, 
            self._right, 
            self._bottom, 
            self._left)

    #top
    @property
    def top(self):
        return self._top

    #right
    @property
    def right(self):
        return self._right

    #left
    @property
    def left(arg):
        return self._left

    #bottom
    @property
    def bottom(self):
        return self._bottom

    #name
    @property
    def name(self):
        return self._name

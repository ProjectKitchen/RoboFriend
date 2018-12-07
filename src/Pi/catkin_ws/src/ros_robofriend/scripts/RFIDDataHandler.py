import rospy

class RFIDDataHandler():

	def __init__(self):
		pass

	def processData(self, data):
		rospy.loginfo('{%s} DATA: %s', self.__class__.__name__, data.data)
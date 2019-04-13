import rospy

# import ros services
from robofriend.srv import SrvObjectDetection

# import ros messages
from robofriend.msg import SpeechData

class RobobrainObjectdetectionHandler():

    def __init__(self):

        self._elapse_time = 1
        self._objectdetection_record_request = None
        self._objectdetection_node_started = False

        # init publishers
        self._pub_speech = rospy.Publisher('/robofriend/speech_data', SpeechData, queue_size = 10)
        self._msg_speech = SpeechData()

        # init service
        try:
            rospy.wait_for_service('robofriend/detect_objects', timeout = self._elapse_time)
        except rospy.ROSException:
            rospy.logwarn("{%s} - Objectdetetcion node was not able to start within %s seconds threfore no person detection possible!",
                self.__class__.__name__, str(self._elapse_time))
            self._objectdetection_node_started = False
        else:
            rospy.logdebug("{%s} - Objectdetection node started", self.__class__.__name__)
            self._objectdetection_record_request = rospy.ServiceProxy('robofriend/detect_objects', SrvObjectDetection)
            self._objectdetection_node_started = True

    def _start_persondetection(self):
        retVal  = False
        if self._objectdetection_node_started is False:
            rospy.logwarn("{%s} - Objectdetection node not started therefore leave object interaction state", self.__class__.__name__)
        else:
            self._publish_speech_message("custom", "Ich schaue mich dann mal nach Personen um!")
            obj_detection_response = self._objectdetection_record_request(detect_obj = True)
            rospy.logdebug("{%s} - Response from Object Detection node: %s",
                self.__class__.__name__, str(obj_detection_response.obj))

            if obj_detection_response.obj:
                rospy.logdebug("{%s} - An object is detected!", self.__class__.__name__)
                if obj_detection_response.obj == "person":
                    self._publish_speech_message("custom", "Ich habe eine Person gefunden!")
                    retVal = True
                else:
                    self._publish_speech_message("custom", "Ich habe keine Person gefunden!")
                    retVal = False
            else:
                rospy.logwarn("{%s} - No object ist detected!", self.__class__.__name__)
        return retVal

    def _publish_speech_message(self, mode, text = None):
        self._msg_speech.mode = mode
        self._msg_speech.text = text
        self._pub_speech.publish(self._msg_speech)
        rospy.logdebug("{%s} - Speech published data: {%s}",
            self.__class__.__name__, str(self._msg_speech))

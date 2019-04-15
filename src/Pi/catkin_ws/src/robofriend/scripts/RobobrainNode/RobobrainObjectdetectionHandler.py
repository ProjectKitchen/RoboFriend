import rospy
from threading import *
from time import *

# import ros services
from robofriend.srv import SrvObjectDetection
from robofriend.srv import SrvObjectHeartbeatData


# import ros messages
from robofriend.msg import SpeechData

class RobobrainObjectdetectionHandler():

    def __init__(self):

        self._elapse_time = 1
        self._objectdetection_record_request = None

        self._objectdetection_node_started = False
        self._objectdetection_node_status_lock = Lock()

        # init publishers
        self._pub_speech = rospy.Publisher('/robofriend/speech_data', SpeechData, queue_size = 10)
        self._msg_speech = SpeechData()

        # init service
        try:
            rospy.wait_for_service('robofriend/detect_objects', timeout = self._elapse_time)
            rospy.wait_for_service('/robofriend/obj_heartbeat', timeout = self._elapse_time)
        except rospy.ROSException:
            rospy.logwarn("{%s} - Objectdetetcion node was not able to start within %s seconds threfore no person detection possible!",
                self.__class__.__name__, str(self._elapse_time))
            self._object_detection_node_status(False)
        else:
            rospy.logdebug("{%s} - Objectdetection node started", self.__class__.__name__)
            self._object_detection_node_status(True)

        thread = Thread(target = self._hb_thread)
        thread.daemon = True
        thread.start()

    def _object_detection_node_status(self, alive = False):
        if alive is True:
            self._objectdetection_record_request = rospy.ServiceProxy('robofriend/detect_objects', SrvObjectDetection)
            self._object_hb_request = rospy.ServiceProxy('/robofriend/obj_heartbeat', SrvObjectHeartbeatData)
            self._set_objectdetection_status_flag(True)
        elif alive is False:
            self._objectdetection_record_request = None
            self._set_objectdetection_status_flag(False)

    def _hb_thread(self):
        rospy.logdebug("Heartbeat thread started!")
        while True:
            self._object_hb_request = rospy.ServiceProxy('/robofriend/obj_heartbeat', SrvObjectHeartbeatData)
            try:
                ret = self._object_hb_request(True)
            except rospy.ServiceException:
                rospy.logdebug("{%s} - Objectdetecion node not started yet!", rospy.get_caller_id())
                self._object_detection_node_status(False)
            else:
                rospy.logdebug("{%s} - Objectdetecion node started!", rospy.get_caller_id())
                self._object_detection_node_status(True)
            sleep(1)

    def _start_persondetection(self):
        retVal  = False
        if self._get_objectdetection_status_flag() is False:
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

    def _set_objectdetection_status_flag(self, alive = False):
        self._objectdetection_node_status_lock_aquire()
        self._objectdetection_node_started = alive
        self._objectdetection_node_status_lock_release()

    def _get_objectdetection_status_flag(self):
        self._objectdetection_node_status_lock_aquire()
        retVal = None
        retVal = self._objectdetection_node_started
        self._objectdetection_node_status_lock_release()
        return retVal

    def _objectdetection_node_status_lock_aquire(self):
        self._objectdetection_node_status_lock.acquire()

    def _objectdetection_node_status_lock_release(self):
        self._objectdetection_node_status_lock.release()

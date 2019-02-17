import rospy

# import ros messages and services
from robofriend.srv import SrvVoiceHotwordActivation, SrvVoiceHotwordActivationResponse

class VoiceDetectionDataHandler():

    def __init__(self):

        # declare voice hotword detection service
         serv = rospy.Service('/robofriend/voicehotword', SrvVoiceHotwordActivation, self._voice_hotword_service_handler)

    def _voice_hotword_service_handler(self, request):
        rospy.logdebug("{%s} - Voice hotword activation request received!\n",
            self.__class__.__name__)

        

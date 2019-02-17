from VoiceDetectionNode.VoiceDetectionDataHandler import *
import rospy

def node_start():
    rospy.logdebug("Voice detection Node started!\n")
    VoiceDetectionDataHandler()

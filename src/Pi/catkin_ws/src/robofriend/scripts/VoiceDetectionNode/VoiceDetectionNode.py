from VoiceDetectionNode.VoiceDetectionDataHandler import *
import rospy

# globals
voice = None

def node_start():
    global voice

    rospy.logdebug("Voice detection Node started!\n")
    VoiceDetectionDataHandler()

#!/usr/bin/env python3
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
import threading
import rospy
import json

# import ros messages and services
from robofriend.srv import SrvVoiceHotwordActivationData, SrvVoiceHotwordActivationDataResponse
from robofriend.msg import VoiceData


class VoiceDetectionDataHandler():

    def __init__(self):

        # declare voice hotword detection service
        serv = rospy.Service('/robofriend/voicehotword', SrvVoiceHotwordActivationData, self._voice_hotword_activation_service_handler)

        self._house_room = ['living room', 'bedroom', 'kitchen']
        self._on_off = ['on', 'off']

        self._host = 'localhost'
        self._port = 1883

        self._pub = rospy.Publisher('/robofriend/voice_data', VoiceData, queue_size = 10)
        self._msg = VoiceData()

        self._start_mqtt_thread()

    def _start_mqtt_thread(self):   # TODO: try without thread
        thread = threading.Thread(
            target = self._mqtt_client_init
        )

        # set thread as a daemon
        thread.daemon = True

        # start thread
        thread.start()

    def _mqtt_client_init(self):
        self._client = mqtt.Client()
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        self._client.connect(self._host, self._port, 60)
        self._client.loop_forever()

    def _on_connect(self, client, userdata, flags, rc):
        rospy.logdebug("{%s} - Client connected!\n",
            self.__class__.__name__)

        # Subscribe to the intent topic
        self._client.subscribe('hermes/intent/#')
        # Subscribe to the hotword detected topic
        self._client.subscribe('hermes/hotword/default/detected')
        # Subscribe to the intend Parsed topic
        #client.subscribe('hermes/nlu/intentParsed')


    def _on_message(self, client, userdata, msg):
        print("Message received on topic {0}: {1}\n".format(msg.topic, msg.payload))
        payload = json.loads(msg.payload.decode())
        #print("Payload: {}\n".format(payload))

        if msg.topic == 'hermes/intent/momokarl:Lights':
            if len(payload['slots']) < 2:
                rospy.logwarn("{%s} - Not enough slots!\n",
                    self.__class__.__name__)
            else:
                room = self._check_slot_value(len(payload["slots"]), payload, self._house_room)
                action = self._check_slot_value(len(payload["slots"]), payload, self._on_off)
                if room is False or action is False:
                    rospy.logwarn("{%s} - Wrong phrase!\n",
                        self.__class__.__name__)
                else:
                    rospy.logwarn("{%s} - Right slotvalues detected!\n",
                        self.__class__.__name__)
                    self._message_publish("lights", str(room) + "/" + str(action))

    def _check_slot_value(self, slotamount, payload, list):
        for cnt in range(slotamount):
            for elem in list:
                if elem in payload["slots"][cnt]["value"]["value"]:
                    rospy.logwarn("{%s} - Found item: {%s}", self.__class__.__name__, elem)
                    return elem
        else:
            return False

    def _voice_hotword_activation_service_handler(self, request):
        rospy.logdebug("{%s} - Voice hotword activation request received!\n",
            self.__class__.__name__)
        self._activate_hotword()
        return SrvVoiceHotwordActivationDataResponse(True)


    def _activate_hotword(self):
        publish.single('hermes/hotword/default/detected', payload=json.dumps(
        {'siteId': 'default', 'modelId': 'robofriend-ts'}), hostname='localhost',
        port=1883)

    def _message_publish(self, intent, slot):
        self._msg.intent = intent
        self._msg.slots = slot
        rospy.logwarn("Message send: %s!", self._msg)
        self._pub.publish(self._msg)

def shutdown():
    rospy.loginfo("{%s} - stopping voice node!",
        rospy.get_caller_id())
    pygame.quit()
    rospy.signal_shutdown("Stopping voice node")

def Voice():
    rospy.init_node("robofriend_voice", log_level = rospy.DEBUG)
    rospy.loginfo("{%s} - starting voice node!",
        rospy.get_caller_id())

    VoiceDetectionDataHandler()

    rate = rospy.Rate(10) # 2 fps

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        Voice()
    except rospy.ROSInterruptException:
        pass

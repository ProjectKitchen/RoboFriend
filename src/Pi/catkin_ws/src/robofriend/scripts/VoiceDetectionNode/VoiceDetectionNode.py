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

        # init Subscriber
        # rospy.Subscriber('/robofriend/voice_data', VoiceData, self._process_data)

        # declare voice hotword detection service
        serv = rospy.Service('/robofriend/voicehotword', SrvVoiceHotwordActivationData, self._voice_hotword_activation_service_handler)

        ########## Lights / TV / HIFI / DVD-Player ##########
        self._house_room = ['living room', 'bedroom', 'kitchen']
        self._on_off = ['on', 'off']

        ########## TV / HIFI ##########
        self._channel = ['channel_down', 'channel_up']
        self._volume = ['volume_down', 'volume_up', 'mute']

        ########## DVD-Player ##########
        self._dvd_action = ['pause', 'stop', 'play']


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
        room = False
        on_off = False
        dvd_action = False
        channel = False
        volume = False

        rospy.logdebug("{%s} - Message received on topic %s: %s",
            rospy.get_caller_id(), str(msg.topic), str(msg.payload))
        payload = json.loads(msg.payload.decode())

        ################################ Lights ################################
        if msg.topic == 'hermes/intent/momokarl:Lights':
            if len(payload['slots']) < 2:
                rospy.logwarn("{%s} - Not enough slots!\n",
                    self.__class__.__name__)
                self._message_publish(enough_slots = False)
            else:
                room = self._check_slot_value(len(payload["slots"]), payload, self._house_room)
                on_off = self._check_slot_value(len(payload["slots"]), payload, self._on_off)
                if room is False or on_off is False:
                    rospy.logwarn("{%s} - Wrong phrase!\n",
                        self.__class__.__name__)
                else:
                    rospy.logdebug("{%s} - Right slotvalues detected!\n",
                        self.__class__.__name__)
                    self._message_publish("lights", str(room) + "/" + str(on_off)) # z.B: "lights", "living room/on"

        ################################ TV ################################
        elif msg.topic == 'hermes/intent/momokarl:TV':
            rospy.logdebug("Within TV intent!")
            if len(payload['slots']) < 2:
                rospy.logwarn("{%s} - Not enough slots!\n",
                    self.__class__.__name__)
                self._message_publish(enough_slots = False)
            else:
                room = self._check_slot_value(len(payload["slots"]), payload, self._house_room)
                if room is False:
                    rospy.logwarn("{%s} / [TV] - No room given")
                else:
                    channel = self._check_slot_value(len(payload["slots"]), payload, self._channel)
                    volume = self._check_slot_value(len(payload["slots"]), payload, self._volume)
                    on_off = self._check_slot_value(len(payload["slots"]), payload, self._on_off)
                    for cnt in channel, volume, on_off:
                        if cnt is not False:
                            rospy.logdebug("Right slotvalues detected: %s", cnt)
                            self._message_publish("tv", str(room) + "/" + str(cnt))
                            break
                    else:
                        rospy.logwarn("{%s} - Wrong phrase!\n",
                        self.__class__.__name__)

        ################################ DVD-Player ################################
        elif msg.topic == 'hermes/intent/momokarl:DVD_Player':
            rospy.logdebug("Within DVD Player intent!")
            if len(payload['slots']) < 1:
                rospy.logwarn("{%s} - Not enough slots!\n",
                    self.__class__.__name__)
                self._message_publish(enough_slots = False)
            else:
                dvd_action = self._check_slot_value(len(payload["slots"]), payload, self._dvd_action)
                on_off = self._check_slot_value(len(payload["slots"]), payload, self._on_off)
                for cnt in dvd_action, on_off:
                    if cnt is not False:
                        rospy.logdebug("Right slotvalues detected: %s", cnt)
                        self._message_publish("dvd_player", str(cnt))
                        break
                else:
                    rospy.logwarn("{%s} - Wrong phrase!\n",
                    self.__class__.__name__)

        ################################ HiFi ################################
        elif msg.topic == 'hermes/intent/momokarl:HiFi':
            rospy.logdebug("Within HiFi intent!")
            if len(payload['slots']) < 1:
                rospy.logwarn("{%s} - Not enough slots!\n",
                    self.__class__.__name__)
                self._message_publish(enough_slots = False)
            else:
                channel = self._check_slot_value(len(payload["slots"]), payload, self._channel)
                volume = self._check_slot_value(len(payload["slots"]), payload, self._volume)
                on_of = self._check_slot_value(len(payload["slots"]), payload, self._on_off)
                for cnt in channel, volume, on_off:
                    if cnt is not False:
                        rospy.logdebug("Right slotvalues detected: %s", cnt)
                        self._message_publish("hifi", str(cnt))
                        break
                else:
                    rospy.logwarn("{%s} - Wrong phrase!\n",
                    self.__class__.__name__)

    def _check_slot_value(self, slotamount, payload, list):
        for cnt in range(slotamount):
            for elem in list:
                if elem in payload["slots"][cnt]["value"]["value"]:
                    rospy.logdebug("{%s} - Found item: {%s}", self.__class__.__name__, elem)
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

    def _message_publish(self, enough_slots = True, intent = "", slot = ""):
        self._msg.enough_slots = enough_slots
        self._msg.intent = intent
        self._msg.slots = slot
        rospy.logdebug("{%s} - Message send: %s!", self.__class__.__name__, self._msg)
        self._pub.publish(self._msg)

def shutdown():
    rospy.loginfo("{%s} - stopping voice node!",
        rospy.get_caller_id())
    pygame.quit()
    rospy.signal_shutdown("Stopping voice node")

def Voice():
    rospy.init_node("robofriend_voice", log_level = rospy.INFO)
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

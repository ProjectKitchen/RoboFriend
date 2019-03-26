#!/usr/bin/env python3
import rospy
import threading
import time
import random

# import ros message and services
from robofriend.msg import LedEarsData
from robofriend.msg import IOWarriorData
from robofriend.srv import SrvLedEarsData, SrvLedEarsDataResponse

class LedEarsDataHandler():

    def __init__(self):
        self.__random = None
        self.__repeat_num = None
        self.__rgb = None
        self.__factor = 1
        self.__refreshIntervalsMS = 300
        self.__red = 0
        self.__green = 10
        self.__blue = 10
        self.__random_flash_event = threading.Event()

        self.__iowarrior_pub =  rospy.Publisher('/robofriend/io_warrior_data', IOWarriorData, queue_size = 10)
        self.__iowarrior_msg = IOWarriorData()

        # Webserver services
        rospy.Service('/robofriend/led_ears_flash', SrvLedEarsData, self._service_handler)

    def _service_handler(self, request):
        rospy.logdebug("{%s} - Led Ears request received!",
            rospy.get_caller_id())

        # if request.mode == "on":
        #     self.__random = "on"
        # elif request.mode == "off":
        #     self.__random = "off"
        # elif request.mode == "rgb":
        #     self.__random = "rgb"
        #     if len(request.rgb_color) is 3:
        #         self.__rgb = list(request.rgb_color)
        #         print(self.__rgb)
        # self._flash_led()
        self.process_data(request)
        return SrvLedEarsDataResponse(True)


    def process_data(self, data):
        self.__random = data.random
        self.__repeat_num = list(data.repeat_num)
        self.__rgb = list(data.rgb_color)
        rospy.logdebug("{%s} - Received Data: %s, %s, %s",
            self.__class__.__name__, self.__random, self.__repeat_num, self.__rgb)
        self._flash_led()

    def _flash_led(self):
        if self.__random == "on" and self._is_random_flash_event_set() == False:
            self._set_event_random_flash()
        elif self.__random == "off" and self._is_random_flash_event_set() == True:
            self._clear_event_random_flash()
            if self.__check_number_elements(self.__rgb):
                self.__set_ear_color(self.__rgb[0], self.__rgb[1], self.__rgb[2])
            else:
                pass
        elif self.__random == "rgb":
            if self._is_random_flash_event_set() == True:
                self._clear_event_random_flash()
            if self.__check_number_elements(self.__rgb):
                self.__set_ear_color(self.__rgb[0], self.__rgb[1], self.__rgb[2])
            else:
                pass
        else:
            if self._is_random_flash_event_set() == True:
                self._clear_event_random_flash()

            if self.__repeat_num is not None:
                if len(self.__repeat_num) is not 0 and len(self.__rgb) > 3:
                    # rospy.logwarn("I ammmmm hereeeeeeeeee")
                    self.__set_ear_color_series()
                else:
                    if self.__check_number_elements(self.__rgb):
                        self.__set_ear_color(self.__rgb[0], self.__rgb[1], self.__rgb[2])
                    else:
                        pass

    def _random_flash(self):
        if self._is_random_flash_event_set() is True:
            rospy.logdebug("{%s} - Thread for random flashing is running!",
                    self.__class__.__name__)
            self.__red = self.__get_new_random_color(self.__red)
            self.__green = self.__get_new_random_color(self.__green)
            self.__blue = self.__get_new_random_color(self.__blue)
            self.__send_to_iowarrior(self.__red, self.__green, self.__blue)

    def _set_event_random_flash(self):
        self.__random_flash_event.set()

    def _clear_event_random_flash(self):
        self.__random_flash_event.clear()

    def _is_random_flash_event_set(self):
        return self.__random_flash_event.is_set()

    def __get_new_random_color(self, old_color):
        newColor = old_color + random.uniform(0, 1) * self.__factor
        if newColor > 15:
            newColor = 15
            self.__factor *= -1
        elif newColor < 0:
            newColor = 0
            self.__factor *= -1
        return newColor

    def __set_ear_color_series(self):
        if len(self.__rgb) % 3 is 0:
            elements_cnt = []
            for cnt in range(self.__repeat_num[0]):
                for elements in self.__rgb:
                    elements_cnt.append(elements)
                    if len(elements_cnt) is 3:
                        self.__set_ear_color(elements_cnt[0], elements_cnt[1], elements_cnt[2])
                        time.sleep(self.__repeat_num[1]/1000.0)
                        elements_cnt.clear()
        else:
            pass

    def __set_ear_color(self, red, green, blue):
        self.__red = red
        self.__green = green
        self.__blue = blue
        self.__send_to_iowarrior(self.__red, self.__green, self.__blue)

    def __check_number_elements(self, rgb):
        if rgb is not None:
            if len(rgb) is not 3:
                return False
            else:
                return True
        else:
            return False

    def __send_to_iowarrior(self, red, green, blue):
        self.__iowarrior_msg.rgb = [red, green, blue]
        self.__iowarrior_msg.cam_pos = 0
        self.__iowarrior_pub.publish(self.__iowarrior_msg)
        rospy.logdebug("{%s} - Publish message to IOWarrior Node: %s",
            self.__class__.__name__, self.__iowarrior_msg)

def shutdown():
    rospy.loginfo("{%s} - stopping led ears data handler.", rospy.get_caller_id())
    rospy.signal_shutdown("Stopping Led Ears Node!")


def LedEars():
    rospy.init_node("robofriend_led_ears", log_level = rospy.INFO)
    rospy.loginfo("{%s} - starting led ears node!",
        rospy.get_caller_id())

    led_ears = LedEarsDataHandler()
    rospy.Subscriber('/robofriend/led_ears_data',
            LedEarsData, led_ears.process_data)

    rate = rospy.Rate(3) # defines the flash speed in random mode

    while not rospy.is_shutdown():
        led_ears._random_flash()
        rate.sleep()


if __name__ == '__main__':
    try:
        LedEars()
    except rospy.ROSInterruptException:
        pass

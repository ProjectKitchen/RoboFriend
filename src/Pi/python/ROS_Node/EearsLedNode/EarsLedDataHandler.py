# import external modules
import random
import threading
import time

class EarsLedDataHandler():

    def __init__(self):
        self.__random = None
        self.__repeat_num = None
        self.__rgb = None
        self.__factor = 1
        self.__refreshIntervalsMS = 300
        self.__red = 0
        self.__green = 10
        self.__blue = 10

        self.__thread_event = threading.Event()
        self.__thread = threading.Thread(
            target = self.__random_thread
        )
        self.__thread.daemon = True
        self.__thread.start()

    def process_data(self, data):
        self.__random = data.random
        self.__repeat_num = list(data.repeat_num)
        self.__rgb = list(data.rgb_color)
        print("[INFO] {} - Received Data: {} {} {}"
            .format(self.__class__.__name__, self.__random, self.__repeat_num, self.__rgb))

        if self.__random == "on" and self.__is_thread_running() == False:
            self.__start_thread()
        elif self.__random == "off" and self.__is_thread_running() == True:
            self.__stop_thread()
            if self.__check_number_elements(self.__rgb):
                self.__set_ear_color(self.__rgb[0], self.__rgb[1], self.__rgb[2])
            else:
                print("[INFO] To few elemenst!")
        else:
            if self.__is_thread_running() == True:
                self.__stop_thread()

            if self.__repeat_num[0] is not 0 and len(self.__rgb) > 3:
                self.__set_ear_color_series()
            else:
                if self.__check_number_elements(self.__rgb):
                    self.__set_ear_color(self.__rgb[0], self.__rgb[1], self.__rgb[2])
                else:
                    print("[INFO] To few elemenst!")

    def __random_thread(self):
        while True:
            self.__thread_event.wait()
            print("[INFO] Thread for random is running!\n")
            self.__red = self.__get_new_random_color(self.__red)
            self.__green = self.__get_new_random_color(self.__green)
            self.__blue = self.__get_new_random_color(self.__blue)
            self.__send_to_iowarrior(self.__red, self.__green, self.__blue)
            time.sleep(self.__refreshIntervalsMS / 100) ########

    def __start_thread(self):
        print("[INFO] Set event to start random thread!")
        self.__thread_event.set()

    def __stop_thread(self):
        print("[INFO] Clear event to stop random thread!")
        self.__thread_event.clear() # sets to false

    def __is_thread_running(self):
        return self.__thread_event.is_set()

    def __get_new_random_color(self, old_color):
        print("[INFO] getNewRandomColor- old_color: {} \nfactor: {}".format(old_color, self.__factor))
        newColor = old_color + random.uniform(0, 1) * self.__factor
        if newColor > 15:
            newColor = 15
            self.__factor *= -1
        elif newColor < 0:
            newColor = 0
            self.__factor *= -1
        print("[INFO] getNewRandomColor- NewColor: {} \nfactor: {}".format(newColor, self.__factor))
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
            print("Wrongggg!!!")
    def __set_ear_color(self, red, green, blue):
        self.__red = red
        self.__green = green
        self.__blue = blue
        self.__send_to_iowarrior(self.__red, self.__green, self.__blue)

    def __check_number_elements(self, rgb):
        if len(rgb) is not 3:
            return False
        else:
            return True

    def __send_to_iowarrior(self, red, green, blue):
        print("[INFO] {} - Publish message to IOWarrior Node: {} {} {}".format(self.__class__.__name__, red, green, blue))

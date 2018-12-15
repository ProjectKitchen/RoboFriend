import threading
import rospy

# import ros services
from ros_robofriend.srv import BatInfData, BatInfDataResponse

class TeensyDataHandler():

    send_lock = threading.Lock()

    def __init__(self, serial):
        self._serial = serial

    def service_handler(self, request):
        sensor = None
        bat = None
        inf_left = None
        inf_middle = None
        inf_right = None

        print("\n[INFO] {} - Request received: {}".format(self.__class__.__name__, request.request))

        #resp_message = self.send_serial("R", True)

        ######################################
        # TODO: for test purposes fakink teensy values
        resp_message = "Sensors,0500,0200,0100,0300\n"
        ######################################

        print("[INFO] Sensor values from teensy: {}".format(resp_message))
        sensor, bat, inf_left, inf_middle, inf_right = resp_message.split(',')
        print("[INFO] {} - Response Service: Sensor: {}, Battery: {}, Infrared left: {}, Infrared middle: {}, Infrared right: {}"
                .format(__class__.__name__, sensor, bat, inf_left, inf_middle, inf_right))
        return BatInfDataResponse(int(bat), int(inf_left), int(inf_middle), int(inf_right))

    def send_serial(serial_message, read_response = False):
        response = None
        self.send_lock.acquire()

        try:
            self._serial.write(str.encode(serial_message) + '\r'.encode('ascii'))
            if read_response:
                response = str(self._serial.readline())
        except Exception as e:
            print("[INFO] Serial write error")
            print(type(e))
            print(inst.args)

        self.send_lock.release()
        return response

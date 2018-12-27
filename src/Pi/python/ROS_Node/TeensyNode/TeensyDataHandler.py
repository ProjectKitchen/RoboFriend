import threading
import rospy

# import ros services
from ros_robofriend.srv import BatInfData, BatInfDataResponse

class TeensyDataHandler():

    step_duration = 50
    loop_duration = 0
    send_lock = threading.Lock()
    motor_direction = {
        'right' : "D 128 -128 ", \
        'left' : "D -128 128 ", \
        'forward' : "D 255 255 ", \
        'backward' : "D -255 -255 ", \
        'backleft' : "D -128 0 ", \
        'backright' : "D 0 -128 ", \
        'forwardright' : "D 128 0 ", \
        'forwardleft' : "D 0 128 ", \
        'stop' : "D"
    }

    def __init__(self, serial):
        self._serial = serial
        self._direction = None
        self._duration = None
        self._webserver_motor_msg = None
        self._motor_loop = {}
        self._motor_step = {}

        for cnt in self.motor_direction:
            self._motor_loop[cnt] = self.motor_direction[cnt] + str(self.loop_duration)
            self._motor_step[cnt] = self.motor_direction[cnt] + str(self.step_duration)

    def service_handler(self, request):
        sensor = None
        bat = None
        inf_left = None
        inf_middle = None
        inf_right = None

        #print("\n[INFO] {} - Request received: {}".format(self.__class__.__name__, request.request))

        #resp_message = self.send_serial("R", True)

        ######################################
        # TODO: for test purposes fakink teensy values
        resp_message = "Sensors,0500,0200,0100,0300\n"
        ######################################

        #print("[INFO] Sensor values from teensy: {}".format(resp_message))
        sensor, bat, inf_left, inf_middle, inf_right = resp_message.split(',')
        # print("[INFO] {} - Response Service: Sensor: {}, Battery: {}, Infrared left: {}, Infrared middle: {}, Infrared right: {}"
        #         .format(__class__.__name__, sensor, bat, inf_left, inf_middle, inf_right))
        return BatInfDataResponse(int(bat), int(inf_left), int(inf_middle), int(inf_right))

    def motor_process_data(self, data):
        self._direction = data.direction
        self._duration = data.duration
        self._web_motor_msg = data.web_motor_msg

        print("[INFO] {} - Received Message from Brain Node: {}".format(__class__.__name__, data))

        if self._web_motor_msg:
            left, right, duration = self._web_motor_msg
            self.send_serial(self.motor_direction['stop'])
            self.send_serial("D " + str(left) + " " + str(right) + " " + str(duration))
        else:
            if self._duration == "loop":
                self.send_serial(self.motor_direction['stop'])
                self.send_serial(self._motor_loop[self._direction])
            elif self._duration == "step":
                self.send_serial(self.motor_direction['stop'])
                self.send_serial(self._motor_step[self._direction])
            elif self._duration == "stop":
                self.send_serial(self.motor_direction['stop'])
            else:
                print("[INFO] Wrong duration")

    def send_serial(self, serial_message, read_response = False):
        response = None
        self.send_lock.acquire()

        print("[INFO] {} - Teensy Send Serial: {}".format(__class__.__name__, serial_message))

        try:
            self._serial.write(str.encode(serial_message) + '\r'.encode('ascii'))
            if read_response:
                response = str(self._serial.readline())
        except Exception as e:
            print("[INFO] Serial write error")
            #print(type(e))
            #print(inst.args)

        self.send_lock.release()
        return response

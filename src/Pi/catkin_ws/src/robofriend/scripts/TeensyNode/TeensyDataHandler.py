import threading
import rospy

# import ros services
from robofriend.srv import *

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
        bat_voltage = None
        inf_left = None
        inf_middle = None
        inf_right = None


        if self._serial is not None:
            try:
                serial_resp = str(self._serial.readline())
            except Exception as e:
                rospy.logwarn('*** Read serial for Teensy failed! ***')
                rospy.logwarn(type(inst))
                rospy.logwarn(inst.args)
        else:
            # serial_resp = "Sensors,3.996,01.10,02.20,03.30\n" # full battery
            # serial_resp = "Sensors,3.796,01.10,02.20,03.30\n" # good
            # serial_resp = "Sensors,3.096,01.10,02.20,03.30\n" # charging
            serial_resp = "Sensors,2.966,01.10,02.20,03.30\n" # shutdown
        
        rospy.logdebug("{%s} Sensor values from teensy: %s", serial_resp)
        sensor, bat_voltage, inf_left, inf_middle, inf_right = serial_resp.split(',')
        rospy.logdebug("{%s} Response Service: Sensor: %s, Battery: %s, Infrared left: %s, Infrared middle: %s, Infrared right: %s",
                self.__class__.__name__, 
                sensor, 
                bat_voltage,
                inf_left, 
                inf_middle, 
                inf_right)

        return SrvPCBSensorDataResponse(
            float(bat_voltage), 
            float(inf_left), 
            float(inf_middle), 
            float(inf_right)
            )

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

    def send_serial(self, serial_message):
        self.send_lock.acquire()

        print("[INFO] {} - Teensy Send Serial: {}".format(__class__.__name__, serial_message))

        try:
            self._serial.write(str.encode(serial_message) + '\r'.encode('ascii'))
        except Exception as e:
            rospy.logwarn('*** Send serial for Teensy failed! ***')
            rospy.logwarn(type(inst))
            rospy.logwarn(inst.args)

        self.send_lock.release()

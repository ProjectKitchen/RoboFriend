import rospy, threading

# import user modules
import constants

# import ros services
from robofriend.srv import SrvPCBSensorDataResponse

# globals
ser = None
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

def drive():
    sendSerial('D')

def readSensorValue():
    print("readSensorValues")
    sendSerial('R')
    
def setSensorThresholds():
    sendSerial('S')

# map the inputs to the function blocks
options = {'D' : drive,
           'R' : readSensorValue,
           'S' : setSensorThresholds,
}

def sendSerial(cmd):
    send_lock.acquire()

    rospy.loginfo("{%s} - teensy send serial: \'%s\'", rospy.get_caller_id(), cmd)

    if ser is not None:
        try:
            ser.write(str.encode(cmd) + '\r'.encode('ascii'))
        except Exception as inst:
            rospy.logwarn('This is a controlled catch!')
            rospy.logwarn('*** Send serial for Teensy failed! ***')
            rospy.logwarn('Exception type: %s', type(inst))
            rospy.logwarn('Exception argument: %s', inst.args[1])
    
    send_lock.release()

def setSerial(serial):
    global ser
    ser = serial
#     direction = None
#     duration = None
#     webserver_motor_msg = None
#     motor_loop = {}
#     motor_step = {}
# 
#     for cnt in motor_direction:
#         motor_loop[cnt] = motor_direction[cnt] + str(loop_duration)
#         motor_step[cnt] = motor_direction[cnt] + str(step_duration)

def serviceHandler(req):
    options[req.cmd]()
    
    sensor = None
    bat_voltage = -1
    inf_left = -1
    inf_middle = -1
    inf_right = -1
    serial_resp = None
    
    if ser is not None:
        try:
            serial_resp = str(ser.readline())
        except Exception as inst:
            rospy.logwarn('This is a controlled catch!')
            rospy.logwarn('*** Read serial for Teensy failed! ***')
            rospy.logwarn('Exception type: %s', type(inst))
            rospy.logwarn('Exception argument: %s', inst.args[1])
    else:
        if constants.DEBUG is True:
            serial_resp = "Sensors,0696,01.10,02.20,03.30" # GOOD
    
    if serial_resp is not None:
        rospy.logdebug("{%s} Sensor values from teensy: %s", rospy.get_caller_id(), serial_resp)
        sensor, bat_voltage, inf_left, inf_middle, inf_right = serial_resp.split(',')
        rospy.logdebug("{%s} Response Service: Sensor: %s, Battery: %s, Infrared left: %s, Infrared middle: %s, Infrared right: %s",
                rospy.get_caller_id(), 
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

def motor_process_data(data):
    direction = data.direction
    duration = data.duration
    web_motor_msg = data.web_motor_msg

    print("[INFO] {} - Received Message from Brain Node: {}".format(__class__.__name__, data))

    if web_motor_msg:
        left, right, duration = web_motor_msg
        sendSerial(motor_direction['stop'])
        sendSerial("D " + str(left) + " " + str(right) + " " + str(duration))
    else:
        if duration == "loop":
            sendSerial(motor_direction['stop'])
#             sendSerial(motor_loop[direction])
        elif duration == "step":
            sendSerial(motor_direction['stop'])
#             sendSerial(motor_step[direction])
        elif duration == "stop":
            sendSerial(motor_direction['stop'])
        else:
            print("[INFO] Wrong duration")
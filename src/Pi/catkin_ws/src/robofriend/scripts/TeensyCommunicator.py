#!/usr/bin/env python
import rospy, serial, threading, time

# import user modules
import constants

# import ros services
from robofriend.srv import SrvTeensySerialData
from robofriend.srv import SrvTeensySerialDataResponse

# globals
ser = None
send_lock = threading.Lock()
driveDuration = 50

def move(left, right, duration = driveDuration):
    sendSerial(constants.STOP_MOVING)
    sendSerial("D " + str(left) + " " + str(right) + " " + str(duration))

# MOMOKARL: called from faceModule
# make the following service request: request('H', False) 
def shakeHeadForNo(): 
    sendSerial(constants.STOP_MOVING)
    sendSerial(constants.SHAKE_HEAD_FOR_NO_SEQ_1)
    time.sleep(0.5)
    sendSerial(constants.SHAKE_HEAD_FOR_NO_SEQ_2)

def readSensorValue():
    sendSerial('R')
    
def setSensorThresholds(left = -1, middle = -1, right = -1):
    sendSerial("S " + str(left) + " " + str(middle) + " " + str(right))
    
# map the inputs to the function blocks
options = {'D' : move,
           'H' : shakeHeadForNo,
           'R' : readSensorValue,
           'S' : setSensorThresholds,
}

def serviceHandler(req):
    if len(req.cmd) == 1:
        options[req.cmd]()
    elif len(req.cmd) > 1:
        param = req.cmd[2:].split()
        if len(param) == 2:
            options[req.cmd[0]](param[0], param[1], driveDuration)
        elif len(param) == 3:
            options[req.cmd[0]](param[0], param[1], param[2])

    bat_voltage = -1
    inf_left = -1
    inf_middle = -1
    inf_right = -1
    
    if req.readResponse is True:
        sensor = None
        serial_resp = None
        
        if ser is not None:
            try:
                serial_resp = str(ser.readline()).strip()
            except Exception as inst:
                rospy.logwarn('{%s} - this is a controlled catch.', rospy.get_caller_id())
                rospy.logwarn('{%s} - read serial for teensy failed.', rospy.get_caller_id())
                rospy.logwarn('{%s} - exception type: %s', rospy.get_caller_id(), type(inst))
                rospy.logwarn('{%s} - exception argument: %s', rospy.get_caller_id(), inst.args[1])
        else:
            if constants.DEBUG is True:
                serial_resp = "Sensors,0696,0100,0200,0300" # GOOD
        
        if serial_resp is not None:
            try:
                rospy.loginfo("{%s} - sensor values from teensy: %s", rospy.get_caller_id(), serial_resp)
                temp = serial_resp.split(',')
                if temp[0] == "Sensors":
                    sensor, bat_voltage, inf_left, inf_middle, inf_right = serial_resp.split(',')
                    rospy.logdebug("{%s} - Response Service: Sensor: %s, Battery: %s, Infrared left: %s, Infrared middle: %s, Infrared right: %s",
                     	rospy.get_caller_id(), 
                        	sensor, 
                        	bat_voltage,
                        	inf_left, 
                        	inf_middle, 
                        	inf_right)
            except Exception as inst:
                rospy.logwarn('{%s} - this is a controlled catch.', rospy.get_caller_id())
                rospy.logwarn('{%s} - parsing teensy serial data failed.', rospy.get_caller_id())
                rospy.logwarn('{%s} - exception type: %s', rospy.get_caller_id(), type(inst))
                rospy.logwarn('{%s} - exception argument: %s', rospy.get_caller_id(), inst.args[1])

    return SrvTeensySerialDataResponse(
        float(bat_voltage), 
        float(inf_left), 
        float(inf_middle), 
        float(inf_right)
        )

def sendSerial(cmd):
    send_lock.acquire()

    if cmd is not 'R':
        rospy.loginfo("{%s} - teensy send serial: \'%s\'", rospy.get_caller_id(), cmd)

    if ser is not None:
        try:
            ser.write(str.encode(cmd) + '\r'.encode('ascii'))
        except Exception as inst:
            rospy.logwarn('{%s} - this is a controlled catch.', rospy.get_caller_id())
            rospy.logwarn('{%s} - send serial for teensy failed.', rospy.get_caller_id())
            rospy.logwarn('{%s} - exception type: %s', rospy.get_caller_id(), type(inst))
            rospy.logwarn('{%s} - exception argument: %s', rospy.get_caller_id(), inst.args[1])
    
    send_lock.release()

def shutdown():
    if ser is not None:
        ser.close()
    rospy.loginfo("{%s} - stopping serial data handler node.", rospy.get_caller_id())
    rospy.signal_shutdown("controlled shutdown.")

def TeensyCommunicator():
    global ser
    
    rospy.init_node("robofriend_teensy_communicator", log_level = rospy.INFO)
    rospy.loginfo("{%s} - starting teensy communicator node.", rospy.get_caller_id())
    rospy.on_shutdown(shutdown)

    try:
        ser = serial.Serial(constants.SER_DEV_TEENSY, constants.SER_DEV_TEENSY_BD, timeout = 1)
        rospy.loginfo("{%s} - serial for teensy opened.", rospy.get_caller_id())
    except Exception as inst:
        rospy.logwarn('{%s} - this is a controlled catch.', rospy.get_caller_id())
        rospy.logwarn('{%s} - serial for teensy could not opened.', rospy.get_caller_id())
        rospy.logwarn('{%s} - exception type: %s', rospy.get_caller_id(), type(inst))
        rospy.logwarn('{%s} - exception argument: %s', rospy.get_caller_id(), inst.args[1])

    # declare services
    rospy.Service('/robofriend/teensy_serial_data', SrvTeensySerialData, serviceHandler)

    rospy.spin()
    
if __name__ == '__main__':
    try:
        TeensyCommunicator()
    except rospy.ROSInterruptException:
        pass


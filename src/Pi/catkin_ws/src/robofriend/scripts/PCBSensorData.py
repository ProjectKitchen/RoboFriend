#!/usr/bin/env python
import rospy

# import user modules
import constants

# import ros messages
from robofriend.msg import PCBSensorData

# import ros services
from robofriend.srv import SrvTeensySerialData

# globals
batVolt = None
statusCount = 0
batMovingAverage = 30

def processSensorValues(pub, args):
    global batVolt
    
    if args is None:
        return
    if args.bat_voltage == -1:
        return

    # this is the voltage divider result in volts (between 0 - ADC_INTERNAL_VREF)
    volt_div_val = constants.ADC_INTERNAL_VREF / float(constants.ADC_RESOLUTION) * args.bat_voltage
    print(volt_div_val)
    # this is the actual battery voltage (between 0 - 12V)
    act_bat_volt = (volt_div_val * (constants.VOLT_DIV_R1 + constants.VOLT_DIV_R2)) / constants.VOLT_DIV_R2

    # get moving average for more precise measurement
    batVolt = getMovingAverage(act_bat_volt, batVolt, batMovingAverage)
    # get the precent on the moving average
    batPercent = getBatteryPercent(batVolt)

    data = PCBSensorData()
    data.voltage = batVolt
    data.percentage = batPercent
    data.ir_sensor_left = args.inf_left
    data.ir_sensor_middle = args.inf_middle
    data.ir_sensor_right = args.inf_right
    data.power_supply_status = getPowerSupplyStatus(batPercent)
    
    rospy.logdebug("{%s} - pcb sensor data:\n%s", rospy.get_caller_id(), data)

    # publish message to robobrain node
    pub.publish(data)

def getMovingAverage(newValue, currentMean, n):
    if not currentMean:
        return newValue
    newMean = currentMean - currentMean / n
    newMean = newMean + newValue / n
    return newMean

def getBatteryPercent(batVoltage):
    batVoltageRounded = round(batVoltage, 2)

    # dont multiply by 100 cause sensor_msgs/BatteryState.percentage
    # is the charge percentage on 0 to 1 range
    percent = round(
        (batVoltageRounded - constants.BAT_LOWWER_THREDSHOLD) / 
        (constants.BAT_UPPER_THRESHOLD - constants.BAT_LOWWER_THREDSHOLD), 2)
    return percent

def getPowerSupplyStatus(val):
    global statusCount, batMovingAverage
    
    statusCount += 1
    if statusCount >= batMovingAverage:
        if val > 1.0:
            return constants.BAT_OVERCHARGED
        elif val <= 1.0 and val > 0.80:
            return constants.BAT_FULL
        elif val <= 0.80 and val > 0.20:
            return constants.BAT_GOOD
        elif val <= 0.20 and val > 0.05:
            return constants.BAT_WARNING
        elif val <= 0.5 and val > 0.0:
            return constants.BAT_CRITICAL
        else:
            return constants.BAT_UNKNOWN
    else:
        return constants.BAT_UNKNOWN

def shutdown():
    rospy.loginfo("{%s} - stopping pcb sensor data handler node.", rospy.get_caller_id())
    rospy.signal_shutdown("controlled shutdown.")

def PCBSensors():
    rospy.init_node("robofriend_pcb_sensor_data", log_level = rospy.INFO)
    rospy.loginfo("{%s} - starting pcb sensor data handler node.", rospy.get_caller_id())
    rospy.on_shutdown(shutdown)

    pub = rospy.Publisher("/robofriend/pcb_sensor_data", PCBSensorData, queue_size = 2)

    rate = rospy.Rate(1) # 1hz

    while not rospy.is_shutdown():
        srv_resp = None
        rospy.wait_for_service('/robofriend/teensy_serial_data')

        try:
            request = rospy.ServiceProxy('/robofriend/teensy_serial_data', SrvTeensySerialData)
            srv_resp = request('R', True)
        except rospy.ServiceException:
            rospy.logwarn("{%s} - service call failed. check the teensy serial data.", rospy.get_caller_id())

        processSensorValues(pub, srv_resp) 

        rate.sleep() # make sure the publish rate maintains at the needed frequency

if __name__ == '__main__':
    try:
        PCBSensors()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python
import rospy
import constants

# import ros message
from robofriend.msg import IRSensorData
from sensor_msgs.msg import BatteryState

# import ros services
from robofriend.srv import SrvPCBSensorData

def processSensorValues(bs_pub, ir_pub, args):
    if args is None:
        return
    if args.bat_voltage == -1:
        return

    # this is the voltage divider result in volts (between 0 - ADC_INTERNAL_VREF)
    volt_div_val = constants.ADC_INTERNAL_VREF / float(constants.ADC_RESOLUTION) * args.bat_voltage
    # this is the actual battery voltage (between 0 - 12V)
    act_bat_volt = (volt_div_val * (constants.VOLT_DIV_R1 + constants.VOLT_DIV_R2)) / constants.VOLT_DIV_R2

    batVolt = 0
    batMovingAverage = 30
    
    # get moving average for more precise measurement
    batVolt = getMovingAverage(act_bat_volt, batVolt, batMovingAverage)
    # get the precent on the moving average
    batPercent = getBatteryPercent(batVolt)

    # http://docs.ros.org/api/sensor_msgs/html/msg/BatteryState.html
    bs_data = BatteryState()
    bs_data.voltage = batVolt
    bs_data.percentage = batPercent
    bs_data.power_supply_status = getPowerSupplyStatus(batPercent)
    
    ir_data = IRSensorData()
    ir_data.inf_left = args.inf_left
    ir_data.inf_middle = args.inf_middle
    ir_data.inf_right = args.inf_right

    rospy.logdebug("{%s} - battery state data:\n%s", rospy.get_caller_id(), bs_data)
    rospy.logdebug("{%s} - infrared sensor data:\n%s", rospy.get_caller_id(), ir_data)

    # publish message to robobrain node
    bs_pub.publish(bs_data)
    ir_pub.publish(ir_data)

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
    # http://docs.ros.org/jade/api/sensor_msgs/html/msg/BatteryState.html
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

def shutdown():
    rospy.loginfo("{%s} - stopping pcb sensor data handler node.", rospy.get_caller_id())
    rospy.signal_shutdown("controlled shutdown.")

def PCBSensorData():
    rospy.init_node("robofriend_pcb_sensor_data", log_level = rospy.INFO)
    rospy.loginfo("{%s} - starting pcb sensor data handler node.", rospy.get_caller_id())
    rospy.on_shutdown(shutdown)

    bs_pub = rospy.Publisher("/robofriend/battery_state", BatteryState, queue_size = 2)
    ir_pub = rospy.Publisher("/robofriend/infrared_data", IRSensorData, queue_size = 2)

    rate = rospy.Rate(1) # 1hz

    while not rospy.is_shutdown():
        srv_resp = None
        rospy.wait_for_service('/robofriend/get_pcb_sensor_data')

        try:
            request = rospy.ServiceProxy('/robofriend/get_pcb_sensor_data', SrvPCBSensorData)
            srv_resp = request('R')
        except rospy.ServiceException:
            rospy.logwarn("{%s} - service call failed. check the teensy serial data.", rospy.get_caller_id())

        processSensorValues(bs_pub, ir_pub, srv_resp) 

        rate.sleep() # make sure the publish rate maintains at the needed frequency

if __name__ == '__main__':
    try:
        PCBSensorData()
    except rospy.ROSInterruptException:
        pass
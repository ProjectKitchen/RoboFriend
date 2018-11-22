import os
import constants

def getFilenames(path):
    return next(os.walk(path))[2]

def mapRange(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

def restrictRange(value, minValue, maxValue):
    return value if minValue <= value <= maxValue else (minValue if value < minValue else maxValue)

def getBatPercent(batVoltage):
    print ("BATTVOLTAGE: {}".format(batVoltage))
    batVoltageRounded = round(batVoltage, 2)
    mapping = constants.getBatPercentMapping()
    percent = 100
    for pair in mapping:
        if batVoltageRounded <= pair[0]:
            percent = pair[1]
    return percent


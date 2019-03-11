import os

""" MOMO: einfach die Methoden in die jeweiligen Module verschieben """
""" Es bedarf hier keine Topics """

# MOMOKARL: called from soundModule.py. Please make sure to implement this method.
def getFilenames(path):
    return next(os.walk(path))[2]

# MOMOKARL: called from faceModule.py. Please make sure to implement this method.
def mapRange(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

# MOMOKARL: called from faceModule.py. Please make sure to implement this method.
def restrictRange(value, minValue, maxValue):
    return value if minValue <= value <= maxValue else (minValue if value < minValue else maxValue)

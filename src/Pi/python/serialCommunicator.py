import serial

# opening serial for teensy
try:
    ser = serial.Serial("/dev/ttyACM0", 9600, timeout=1)
    print('***Serial for Teensy opened***')
except:
    print('***Serial for Teensy could not be opened***')
    #	sys.exit('Could not open serial for Teensy. Check connection and restart.')

def test():
    return "package works!"
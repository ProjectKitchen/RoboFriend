# coding=utf-8

# Dump Debug Data
DEBUG = True

# Serial Ports
SER_DEV_LIDAR	= "/dev/ttyUSB0"
SER_DEV_RFID	= "/dev/ttyUSB1"
SER_DEV_TEENSY	= "/dev/ttyACM0"

# Serial Baudrate
SER_DEV_RFID_BD	= 9600
SER_DEV_TEENSY_BD = 9600

# Battery Level
BAT_OVERCHARGED	= 5
BAT_FULL = 4
BAT_GOOD = 3
BAT_WARNING = 2
BAT_CRITICAL = 1
BAT_UNKNOWN = 0

# Teensy ADC Resolution
ADC_INTERNAL_VREF = 5
ADC_EXTERNAL_VREG = 4.096
ADC_RESOLUTION = 1023

# Battery Hysteresis
BAT_UPPER_THRESHOLD = 14.7
BAT_LOWWER_THREDSHOLD = 10.5

# Voltage Divider
# http://www.peacesoftware.de/einigewerte/spannungsteiler.html
# Spannung U1: 14.7
# Spannung U2: 4.096
# Rges: 10000
# Widerstandsreihe
# Parall/Serienschaltung zulassen
# Siehe also schematic 
VOLT_DIV_R1 = 7210
VOLT_DIV_R2 = 2780

# Robobrain States
RF_ADMIN = 0
RF_CHARGE = 1
RF_FACEDETECTION = 2
RF_MANUAL = 3
RF_AUTONOM = 4
RF_IDLE = 5
RF_SHUTDOWN = 6

# Motor Move Commands
MOVE_STEP_FWD = "D 255 255"
MOVE_STEP_RYT = "D 128 -128"
MOVE_STEP_BCK = "D -255 -255"
MOVE_STEP_LFT = "D -128 128"
MOVE_LOOP_FWD = "D 255 255 0"
MOVE_LOOP_RYT = "D 128 -128 0"
MOVE_LOOP_BCK = "D -255 -255 0"
MOVE_LOOP_LFT = "D -128 128 0"
MOVE_LOOP_FWD_RYT = "D 128 0 0"
MOVE_LOOP_FWD_LFT = "D 0 128 0"
MOVE_LOOP_BCK_RYT = "D 0 -128 0"
MOVE_LOOP_BCK_LFT = "D -128 0 0"
STOP_MOVING = "S"
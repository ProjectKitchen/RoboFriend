# coding=utf-8

# Battery Level
BAT_OVERCHARGED = 5
BAT_FULL = 4
BAT_GOOD = 3
BAT_WARNING = 2
BAT_CRITICAL = 1
BAT_UNKNOWN = 0

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
RF_MANUAL = 2
RF_AUTONOM = 3
RF_IDLE = 4
RF_SHUTDOWN = 5
    
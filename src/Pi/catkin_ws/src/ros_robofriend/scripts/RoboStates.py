#!/usr/bin/env python3

from enum import Enum

class RoboStates(Enum):
    ADMIN = 1 
    FIND_CHARGING_STATION = 2
    AUTONOM = 3
    MANUAL = 4
    
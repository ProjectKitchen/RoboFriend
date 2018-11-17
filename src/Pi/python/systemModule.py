# external modules
import os
import time

# own modules
import speechModule

# globals
# ... none

def shutdown():
    speechModule.speakShutdown()
    time.sleep(3)
    os.system('sudo init 0')
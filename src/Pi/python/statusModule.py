# external modules
import time

# globals
keyScreenshotTimestamp = 'screenshotTimestamp'
screenshotTimestamp = None

# TODO: make sure to pass this to the index.html via the webserver
def getScreenshotTimestamp():
    global currentStatus, keyScreenshotTimestamp
    return currentStatus[keyScreenshotTimestamp] if keyScreenshotTimestamp in currentStatus else None

# TODO: called from the faceModule
def setScreenshotTimestamp(timestamp = None):
    if not timestamp:
        timestamp = time.time()
    global currentStatus, keyScreenshotTimestamp
    currentStatus[keyScreenshotTimestamp] = timestamp

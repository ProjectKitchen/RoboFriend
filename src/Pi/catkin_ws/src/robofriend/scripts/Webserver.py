#!/usr/bin/env python
from flask import Flask, make_response, send_file
import json, os, rospy, threading, time, subprocess, urllib

# import user modules
import constants

# import ros messages
from robofriend.msg import PCBSensorData

# import ros services
from robofriend.srv import SrvTeensySerialData
from robofriend.srv import SrvServoCameraData

# globals
currentStatus = {}
webserverHost = '0.0.0.0'
webserverPort = 8765
webserverDebug = False
password = 'iamrobo'
TEENSY_SRV_REQ = None
servo_cam_req = None

# init webserver
app = Flask(__name__, static_folder='../../../../static')

# *********************************************************************** flask

def getResponse(responseString):
    resp = make_response(responseString)
    resp.headers['Access-Control-Allow-Origin'] = '*'
    return resp

@app.route('/')
def index():
    return make_response(send_file('../../../../index.html'))

@app.route('/map/save/<filename>', methods=['POST'])
def saveMap(filename):
    filename = "../../../../maps/" + filename
    print(filename)
    subprocess.call(["rosrun", "map_server", "map_saver", "-f", "filename"])
    return getResponse("OK")

@app.route('/control/shutdown/<userPassword>', methods=['POST'])
def shutdown(userPassword):
    global password
    if userPassword == password:
        time.sleep(3)
        subprocess.call(["sudo", "init", "0"])
        return getResponse("OK")
    else:
        return getResponse("WRONG PASSWORD")

# ***************************************************************** face module

@app.route('/mouth/smile/<action>', methods=['POST'])
def changeSmile(action):
    methods = {'increase': faceModule.increaseSmile,
               'decrease': faceModule.decreaseSmile
               }
    if action in methods: methods[action]()
    return getResponse("OK")

@app.route('/face/image', methods=['GET'])
def getface():
    global app
    response = make_response(send_file("../" + faceModule.getScreenshotFilename()))
    response.headers['Cache-control'] = 'no-cache'
    response.headers['Access-Control-Allow-Origin'] = '*'
    return response

@app.route('/eyes/move/<direction>', methods=['POST'])
def moveEyes(direction):
    methods = {'up':    faceModule.eyesUp,
               'down':  faceModule.eyesDown,
               'left':  faceModule.eyesLeft,
               'right': faceModule.eyesRight
               }
    if direction in methods:
        methods[direction]()
    return getResponse("OK")

@app.route('/eyes/set/<xPercent>/<yPercent>', methods=['POST'])
def moveEyesXY(xPercent, yPercent):
    faceModule.setEyes(int(xPercent), int(yPercent))
    return getResponse("OK")

# ************************************************************ iowarrior module

@app.route('/camera/change/<increment>', methods=['POST'])
def camerachange(increment):
    global servo_cam_req
    response = None
    response = servo_cam_req(int(increment))
    if response.resp:
        rospy.logdebug("{%s} - Successfull response / camerachange",
                rospy.get_caller_id())
    else:
        rospy.logwarn("{%s} - Erroneous response / camerachange",
                rospy.get_caller_id())
    return getResponse("OK")

@app.route('/camera/down', methods=['POST'])
def cameradown():
    global servo_cam_req
    response = None
    response = servo_cam_req(constants.DOWN)
    if response.resp:
        rospy.logdebug("{%s} - Successfull response / cameradown",
                rospy.get_caller_id())
    else:
        rospy.logwarn("{%s} - Erroneous response / cameradown",
                rospy.get_caller_id())
    return getResponse("OK")

@app.route('/camera/up', methods=['POST'])
def cameraup():
    global servo_cam_req
    response = None
    response = servo_cam_req(constants.UP)
    if response.resp:
        rospy.logdebug("{%s} - Successfull response / cameradown",
                rospy.get_caller_id())
    else:
        rospy.logwarn("{%s} - Erroneous response / cameradown",
                rospy.get_caller_id())
    return getResponse("OK")

@app.route('/ear/color/random/off', methods=['POST'])
def earRandomOff():
    ioWarriorModule.stopRandomEarColor()
    return getResponse("OK")

@app.route('/ear/color/random/on', methods=['POST'])
def earRandomOn():
    ioWarriorModule.startRandomEarColor()
    return getResponse("OK")

@app.route('/ear/color/<earColorR>/<earColorG>/<earColorB>', methods=['POST'])
def setEarRGB(earColorR, earColorG, earColorB):
    r = translateIntRange(int(earColorR), 0, 255, 0, 15)
    g = translateIntRange(int(earColorG), 0, 255, 0, 15)
    b = translateIntRange(int(earColorB), 0, 255, 0, 15)
    ioWarriorModule.setEarColor(r, g, b)
    return getResponse("OK")

def translateIntRange(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return abs(rightMin + (valueScaled * rightSpan))


# ***************************************************************** mood module

@app.route('/mood/set/<moodState>', methods=['POST'])
def setMood(moodState):
    methods = {'happy':     moodModule.setHappy,
               'sad':       moodModule.setSad,
               'angry':     moodModule.setAngry,
               'neutral':   moodModule.setNeutral,
               'tired':     moodModule.setTired
               }
    if moodState in methods:
        methods[moodState]()
    return getResponse("OK")

# *************************************************************** status module

@app.route('/get/status', methods=['GET'])
def getStatus():
    return getResponse(json.dumps(currentStatus))

def providePCBSensorData(data):
    global currentStatus
    currentStatus['voltage'] = round(data.voltage, 2)
    currentStatus['percentage'] = round(data.percentage, 2)
    currentStatus['ir_sensor_left'] = data.ir_sensor_left
    currentStatus['ir_sensor_middle'] = data.ir_sensor_middle
    currentStatus['ir_sensor_right'] = data.ir_sensor_right

# *************************************************************** speech module

@app.route('/speech/say/custom/<text>', methods=['POST'])
def speak(text):
    text = urllib.parse.unquote(text).encode('utf8') #decode action to string
    speechModule.speak(text.decode("utf-8"))
    return getResponse("OK")

@app.route('/speech/say/bullshit', methods=['POST'])
def speakBullshit():
    speechModule.speakBullshit()
    return getResponse("OK")

@app.route('/speech/say/random', methods=['POST'])
def speakRandom():
    speechModule.speakRandom()
    return getResponse("OK")

@app.route('/control/update/<userPassword>', methods=['POST'])
def update(userPassword):
    global password
    if userPassword == password:
        speechModule.speak('Ich aktualisiere mich.')
        p = subprocess.Popen(['git', 'pull'], stdout=subprocess.PIPE)
        p.wait()
        if p.returncode == 0:
            speechModule.speak('Neustart. Bis gleich!')
            subprocess.call(["sudo", "reboot"])
        return getResponse("OK")
    else:
        return getResponse("WRONG PASSWORD")

# **************************************************************** sound module

@app.route('/sound/play/file/<filename>', methods=['POST'])
def playSound(filename):
    soundModule.playSoundFile('data/random/' + filename)
    return getResponse("OK")

@app.route('/sound/play/random', methods=['POST'])
def randomSound():
    soundModule.playRandom()
    return getResponse("OK")

@app.route('/sound/play/mood', methods=['POST'])
def moodSound():
    soundModule.playMood()
    return getResponse("OK")

@app.route('/sound/get/random', methods=['GET'])
def getRandomSounds():
    return getResponse(json.dumps(soundModule.getRandomSounds()))

# *************************************************************** teensy module

@app.route('/move/flex/<left>/<right>/<duration>', methods=['POST'])
def move(left, right, duration):
    global TEENSY_SRV_REQ
    TEENSY_SRV_REQ = "D " + str(left) + " " + str(right) + " " + str(duration)
    return getResponse("OK")

@app.route('/move/stop', methods=['POST'])
def moveStop():
    global TEENSY_SRV_REQ
    TEENSY_SRV_REQ = constants.STOP_MOVING
    return getResponse("OK")

@app.route('/move/simple/<direction>', methods=['POST'])
def moveSimple(direction):
    global TEENSY_SRV_REQ
    methods = {'forward':   constants.MOVE_STEP_FWD,
               'backward':  constants.MOVE_STEP_BCK,
               'left':      constants.MOVE_STEP_LFT,
               'right':     constants.MOVE_STEP_RYT
               }
    if direction in methods:
        TEENSY_SRV_REQ = methods[direction]
    return getResponse("OK")

@app.route('/speech/get/<textCategory>', methods=['GET'])
def getTextsBullshit(textCategory):
    methods = {'random':    speechModule.getRandomTexts,
               'bullshit':  speechModule.getBullshitTexts,
               'left':      constants.MOVE_STEP_LFT,
               'right':     constants.MOVE_STEP_RYT
               }
    texts = []
    if textCategory in methods:
        texts = methods[textCategory]()
    return getResponse(json.dumps(texts))

def stop():
    rospy.loginfo("{%s} - stopping web server node.", rospy.get_caller_id())
    rospy.signal_shutdown("controlled shutdown.")

def run():
    try:
        app.run(debug=webserverDebug, host=webserverHost, port=webserverPort)
    except Exception as inst:
        rospy.logwarn('{%s} - this is a controlled catch.', rospy.get_caller_id())
        rospy.logwarn('{%s} - webserver could not be started.', rospy.get_caller_id())
        rospy.logwarn('{%s} - exception type: %s', rospy.get_caller_id(), type(inst))
        rospy.logwarn('{%s} - exception argument: %s', rospy.get_caller_id(), inst.args[1])

def Webserver():
    global app, webserverDebug, webserverHost, webserverPort
    global TEENSY_SRV_REQ
    global servo_cam_req

    rospy.init_node("robofriend_web_server", log_level = rospy.INFO)
    rospy.loginfo("{%s} - starting webserver node.", rospy.get_caller_id())
    rospy.on_shutdown(stop)

    rospy.Subscriber("/robofriend/pcb_sensor_data", PCBSensorData, providePCBSensorData)

    ws = threading.Thread(target = run)
    ws.daemon = True
    ws.start()

    # create service to communicate with Servo Cam Node
    rospy.wait_for_service('robofriend/camera_position')
    servo_cam_req = rospy.ServiceProxy('/robofriend/camera_position', SrvServoCameraData)

    rate = rospy.Rate(100) # 100hz

    while not rospy.is_shutdown():
        # send data to teensy per service request param
        if TEENSY_SRV_REQ is not None:
            rospy.wait_for_service('/robofriend/teensy_serial_data')

            try:
                request = rospy.ServiceProxy('/robofriend/teensy_serial_data', SrvTeensySerialData)
                request(TEENSY_SRV_REQ, False)
                TEENSY_SRV_REQ = None
            except rospy.ServiceException:
                rospy.logwarn("{%s} - service call failed. check the teensy serial data.", rospy.get_caller_id())

        rate.sleep() # make sure the publish rate maintains at the needed frequency

if __name__ == '__main__':
    try:
        Webserver()
    except rospy.ROSInterruptException:
        pass

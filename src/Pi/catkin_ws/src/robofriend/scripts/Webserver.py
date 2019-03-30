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
from robofriend.srv import SrvLedEarsData
from robofriend.srv import SrvSpeechData
from robofriend.srv import SrvFaceDrawData
from robofriend.srv import SrvFaceScreenshotTimestamp, SrvFaceScreenshotTimestampResponse
from robofriend.srv import SrvSoundData


# globals
currentStatus = {}
webserverHost = '0.0.0.0'
webserverPort = 8765
webserverDebug = False
password = 'iamrobo'
TEENSY_SRV_REQ = None
servo_cam_req = None
led_ears_req = None
speech_req = None
face_req = None
screenshot_filename = ""
sound_req = None

# init webserver
app = Flask(__name__, static_folder='../../../../static')

# *********************************************************************** flask

def getResponse(responseString):
    resp = make_response(responseString)
    resp.headers['Access-Control-Allow-Origin'] = '*'
    return resp

@app.route('/')
def index():
    return make_response(send_file('../../../../index.html')) # TODO: rename file to index.html after testing

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
def record_face_timestamp(request):
    global currentStatus
    retVal = False
    if request.record_time is True:
        currentStatus['screenshotTimestamp'] = time.time()
        retVal = True
    else:
        rospy.logwarn("{%s} - False timestamp request received!",
            rospy.get_caller_id())
        retVal = False
    return SrvFaceScreenshotTimestampResponse(retVal)

@app.route('/mouth/smile/<action>', methods=['POST'])
def changeSmile(action):
    global face_req
    response = None
    param = []

    if action in constants.INCREASE_SMILE:
        response = face_req("increase", param)
    elif action in constants.DECREASE_SMILE:
        response = face_req("decrease", param)
    service_response_check(response.resp, "changeSmile")
    return getResponse("OK")

@app.route('/face/image', methods=['GET'])
def getface():
    global app, screenshot_filename
    resp = None
    param = []

    response = make_response(send_file(screenshot_filename))
    response.headers['Cache-control'] = 'no-cache'
    response.headers['Access-Control-Allow-Origin'] = '*'
    return response

@app.route('/eyes/move/<direction>', methods=['POST'])
def moveEyes(direction):
    global face_req
    response = None
    param = []

    methods = ["up", "down", "left", "right"]

    if direction in methods:
        response = face_req(direction, param)
    service_response_check(response.resp, "moveEyes")
    return getResponse("OK")

@app.route('/eyes/set/<xPercent>/<yPercent>', methods=['POST'])
def moveEyesXY(xPercent, yPercent):
    global face_req
    param = []
    param.append(int(xPercent))
    param.append(int(yPercent))
    response = face_req(constants.SET_EYES, param)
    service_response_check(response.resp, "movesEyesXY")
    return getResponse("OK")

# ************************************************************ iowarrior module

@app.route('/camera/change/<increment>', methods=['POST'])
def camerachange(increment):
    global servo_cam_req
    response = None
    response = servo_cam_req(int(increment))
    service_response_check(response.resp, "camerachange")
    return getResponse("OK")

@app.route('/camera/down', methods=['POST'])
def cameradown():
    global servo_cam_req
    response = None
    response = servo_cam_req(constants.DOWN)
    service_response_check(response.resp, "cameradown")
    return getResponse("OK")

@app.route('/camera/up', methods=['POST'])
def cameraup():
    global servo_cam_req
    response = None
    response = servo_cam_req(constants.UP)
    service_response_check(response.resp, "cameraup")
    return getResponse("OK")

@app.route('/ear/color/random/on', methods=['POST'])
def earRandomOn():
    global led_ears_req
    response = led_ears_req(constants.RANDOM_ON, [], None)
    service_response_check(response.resp, "earRandomOn")
    return getResponse("OK")

@app.route('/ear/color/random/off', methods=['POST'])
def earRandomOff():
    global led_ears_req
    response = led_ears_req(constants.RANDOM_OFF, [], None)
    service_response_check(response.resp, "earRandomOff")
    return getResponse("OK")

@app.route('/ear/color/<earColorR>/<earColorG>/<earColorB>', methods=['POST'])
def setEarRGB(earColorR, earColorG, earColorB):
    global led_ears_req
    rgb = []
    rgb.append(translateIntRange(int(earColorR), 0, 255, 0, 15))
    rgb.append(translateIntRange(int(earColorG), 0, 255, 0, 15))
    rgb.append(translateIntRange(int(earColorB), 0, 255, 0, 15))

    response = led_ears_req(constants.RGB, [], rgb)
    service_response_check(response.resp, "setEarRGB")
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
    # methods = {'happy':     moodModule.setSad,
    #            'sad':       moodModule.setSad,
    #            'angry':     moodModule.setAngry,
    #            'neutral':   moodModule.setNeutral,
    #            'tired':     moodModule.setTired
    #            }

    if moodState in "happy":
        set_happy()
    elif moodState in "sad":
        set_sad()
    elif moodState in "neutral":
        set_neutral()
    elif moodState in "angry":
        set_angry()
    elif moodState in "tired":
        set_tired()
    else:
        rospy.logwarn("{%s} - Wrong mood!",
            rospy.get_caller_id())


    return getResponse("OK")

def set_happy():
    global face_req, sound_req, led_ears_req

    face_req(constants.SET_EYES, [9, -47])
    face_req(constants.SET_SMILE, [80])
    led_ears_req("", [5, 200], [255, 69, 0, 255, 191, 0])
    sound_req(False, "mood", "happy.wav", [])

def set_sad():
    global face_req, sound_req, led_ears_req

    face_req(constants.SET_EYES, [-1, -44])
    face_req(constants.SET_SMILE, [-70])
    led_ears_req(constants.RGB, [], [0, 0, 255])
    sound_req(False, "mood", "sad.wav", [])

def set_neutral():
    global face_req, sound_req, led_ears_req

    face_req(constants.SET_EYES, [0, 0])
    face_req(constants.SET_SMILE, [20])
    led_ears_req(constants.RGB, [], [255, 255, 255])
    sound_req(False, "mood", "neutral.wav", [])

def set_angry():
    global face_req, sound_req, led_ears_req

    face_req(constants.SET_EYES, [-46, -40])
    face_req(constants.SET_SMILE, [-10])
    led_ears_req("", [4, 300], [0, 0, 0, 255, 0, 0])
    sound_req(False, "mood", "angry.wav", [])

def set_tired():
    global face_req, sound_req, led_ears_req

    face_req(constants.SET_EYES, [23, -21])
    face_req(constants.SET_SMILE, [-10])
    led_ears_req(constants.RGB, [], [0, 0, 0])
    sound_req(False, "mood", "tired.wav", [])





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
    global speech_req
    #text = urllib.parse.unquote(text).encode('utf8') #decode action to string
    response = speech_req(False, constants.CUSTOM, text)
    service_response_check(response.resp, "speak")
    return getResponse("OK")

@app.route('/speech/say/bullshit', methods=['POST'])
def speakBullshit():
    global speech_req
    response = speech_req(False, constants.BULLSHIT, constants.STR_NONE)
    service_response_check(response.resp, "speakBullshit")
    return getResponse("OK")

@app.route('/speech/say/random', methods=['POST'])
def speakRandom():
    global speech_req
    response = speech_req(False, constants.RANDOM, constants.STR_NONE)
    service_response_check(response.resp, "speakRandom")
    return getResponse("OK")

@app.route('/control/update/<userPassword>', methods=['POST'])
def update(userPassword):
    global password, speech_req
    text = ""
    if userPassword == password:
        text = "Ich aktualisiere mich."
        response = speech_req(False, constants.CUSTOM, text)
        service_response_check(response.resp, "update")
        p = subprocess.Popen(['git', 'pull'], stdout=subprocess.PIPE)
        p.wait()
        if p.returncode == 0:
            text = "Neustart. Bis gleich!."
            response = speech_req(False, constants.CUSTOM, text)
            service_response_check(response.resp, "update")
            subprocess.call(["sudo", "reboot"])
        return getResponse("OK")
    else:
        return getResponse("WRONG PASSWORD")

# **************************************************************** sound module

@app.route('/sound/play/file/<filename>', methods=['POST'])
def playSound(filename):
    global sound_req
    sound_req(False, "random", filename, [])
    return getResponse("OK")

@app.route('/sound/play/random', methods=['POST'])
def randomSound():
    global sound_req
    resp = sound_req(False, "random", "", [])
    return getResponse("OK")

@app.route('/sound/play/mood', methods=['POST'])
def moodSound():
    global sound_req
    sound_req(False, "mood", "", [])
    return getResponse("OK")

@app.route('/sound/get/random', methods=['GET'])
def getRandomSounds():
    global sound_req
    response = sound_req(True, "", "", [])
    return getResponse(json.dumps(response.filedir))

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
    retVal = None

    if textCategory in constants.RANDOM:
        retVal = get_random_text()
    elif textCategory in constants.BULLSHIT:
        retVal = get_bullshit_text()
    else:
        pass

    methods = {'left':      constants.MOVE_STEP_LFT,
               'right':     constants.MOVE_STEP_RYT
               }
    texts = []
    if textCategory in methods:
        texts = methods[textCategory]()
    return getResponse(json.dumps(retVal))

def get_random_text():
    response = speech_req(True, constants.RANDOM, constants.STR_NONE)
    service_response_check(response.resp, "get_random_text")
    return response.get_text

def get_bullshit_text():
    response = speech_req(True, constants.BULLSHIT, constants.STR_NONE)
    service_response_check(response.resp, "get_bullshit_text")
    return response.get_text

def service_response_check(response = False, funct = ""):
    if response:
        rospy.logdebug("{%s} - Successfull response / %s",
                rospy.get_caller_id(), funct)
    else:
        rospy.logwarn("{%s} - Erroneous response / %s",
                rospy.get_caller_id(), funct)

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
    global servo_cam_req, led_ears_req, speech_req, face_req, screenshot_filename, sound_req

    param = []

    rospy.init_node("robofriend_web_server", log_level = rospy.INFO)
    rospy.loginfo("{%s} - starting webserver node.", rospy.get_caller_id())
    rospy.on_shutdown(stop)


    rospy.Subscriber("/robofriend/pcb_sensor_data", PCBSensorData, providePCBSensorData)

    ws = threading.Thread(target = run)
    ws.daemon = True
    ws.start()

    # create service to communicate with servo cam node
    rospy.wait_for_service('/robofriend/camera_position')
    servo_cam_req = rospy.ServiceProxy('/robofriend/camera_position', SrvServoCameraData)

    # create service to communicate with led ears node
    rospy.wait_for_service('/robofriend/led_ears_flash')
    led_ears_req = rospy.ServiceProxy('/robofriend/led_ears_flash', SrvLedEarsData)

    # create service to communicate with speech node
    rospy.wait_for_service('/robofriend/speech')
    speech_req = rospy.ServiceProxy('/robofriend/speech', SrvSpeechData)

    rospy.Service("/robofriend/face_screen_timestamp", SrvFaceScreenshotTimestamp, record_face_timestamp)

    # create service to communicate with face node
    rospy.wait_for_service('/robofriend/face')
    face_req = rospy.ServiceProxy('/robofriend/face', SrvFaceDrawData)
    resp = face_req(constants.GET_SCREEN_FN, param)
    screenshot_filename = resp.filename

    # TODO: make threads
    rospy.wait_for_service('/robofriend/sound')
    sound_req = rospy.ServiceProxy('/robofriend/sound', SrvSoundData)

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

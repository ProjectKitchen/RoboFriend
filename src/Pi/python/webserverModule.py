# external modules
from flask import Flask, render_template, request, redirect, url_for, make_response, send_file
import threading
import urllib
import json
import logging
import os
import subprocess

#own modules
import ioWarriorModule
import teensyCommunicator
import statusModule
import soundModule
import faceModule
import speechModule
import moodModule
import systemModule

#init
app = Flask(__name__, static_folder='../static')
webserverDebug = False
webserverHost = '0.0.0.0'
webserverPort = 8765
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)
password = 'iamrobo'

# globals
# --- none ---

@app.route('/')
def index():
    return make_response(send_file('../index.html'))

@app.route('/camera/up', methods=['POST'])
def cameraup():
    global cameraPos
    ioWarriorModule.changeCameraPos(10)
    return getResponse("OK")

@app.route('/camera/down', methods=['POST'])
def cameradown():
    global cameraPos
    ioWarriorModule.changeCameraPos(-10)
    return getResponse("OK")

@app.route('/camera/change/<increment>', methods=['POST'])
def camerachange(increment):
    global cameraPos
    ioWarriorModule.changeCameraPos(int(increment))
    return getResponse("OK")

@app.route('/ear/color/<earColorR>/<earColorG>/<earColorB>', methods=['POST'])
def setEarRGB(earColorR, earColorG, earColorB):
    r = translateIntRange(int(earColorR), 0, 255, 0, 15)
    g = translateIntRange(int(earColorG), 0, 255, 0, 15)
    b = translateIntRange(int(earColorB), 0, 255, 0, 15)
    ioWarriorModule.setEarColor(r, g, b)
    return getResponse("OK")

@app.route('/ear/color/random/on', methods=['POST'])
def earRandomOn():
    ioWarriorModule.startRandomEarColor()
    return getResponse("OK")

@app.route('/ear/color/random/off', methods=['POST'])
def earRandomOff():
    ioWarriorModule.stopRandomEarColor()
    return getResponse("OK")

@app.route('/eyes/move/<direction>', methods=['POST'])
def moveEyes(direction):
    methods = {'up': faceModule.eyesUp,
               'down': faceModule.eyesDown,
               'left': faceModule.eyesLeft,
               'right': faceModule.eyesRight
               }
    if direction in methods: methods[direction]()
    return getResponse("OK")

@app.route('/eyes/set/<xPercent>/<yPercent>', methods=['POST'])
def moveEyesXY(xPercent, yPercent):
    faceModule.setEyes(int(xPercent), int(yPercent))
    return getResponse("OK")

@app.route('/mouth/smile/<action>', methods=['POST'])
def changeSmile(action):
    methods = {'increase': faceModule.increaseSmile,
               'decrease': faceModule.decreaseSmile
               }
    if action in methods: methods[action]()
    return getResponse("OK")

@app.route('/mood/set/<moodState>', methods=['POST'])
def setMood(moodState):
    methods = {'happy': moodModule.setHappy,
               'sad': moodModule.setSad,
               'angry': moodModule.setAngry,
               'neutral': moodModule.setNeutral,
               'tired': moodModule.setTired
               }
    if moodState in methods: methods[moodState]()
    return getResponse("OK")

@app.route('/move/flex/<left>/<right>/<duration>', methods=['POST'])
def move(left, right, duration):
    teensyCommunicator.move(left, right, duration)
    return getResponse("OK")

@app.route('/control/shutdown/<userPassword>', methods=['POST'])
def shutdown(userPassword):
    global password
    if userPassword == password:
        systemModule.shutdown()
        return getResponse("OK")
    else:
        return getResponse("WRONG PASSWORD")

@app.route('/control/update/<userPassword>', methods=['POST'])
def update(userPassword):
    global password
    if userPassword == password:
        speechModule.speak('Ich aktualisiere mich.')
        p = subprocess.Popen(['git', 'pull'], stdout=subprocess.PIPE)
        p.wait()
        if p.returncode == 0:
            speechModule.speak('Neustart. Bis gleich!')
            os.system('sudo reboot')
        return getResponse("OK")
    else:
        return getResponse("WRONG PASSWORD")

@app.route('/move/simple/<direction>', methods=['POST'])
def moveSimple(direction):
    methods = {'forward': teensyCommunicator.moveForwardStep,
               'backward': teensyCommunicator.moveBackStep,
               'left': teensyCommunicator.moveLeftStep,
               'right': teensyCommunicator.moveRightStep
               }
    if direction in methods: methods[direction]()
    return getResponse("OK")

@app.route('/move/stop', methods=['POST'])
def moveStop():
    teensyCommunicator.stopMovement()
    return getResponse("OK")

@app.route('/get/status', methods=['GET'])
def getStatus():
    return getResponse(json.dumps(statusModule.getStatus()))

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

@app.route('/speech/say/custom/<text>', methods=['POST'])
def speak(text):
    text = urllib.unquote(text).encode('utf8') #decode action to string
    speechModule.speak(text)
    return getResponse("OK")

@app.route('/speech/say/random', methods=['POST'])
def speakRandom():
    speechModule.speakRandom()
    return getResponse("OK")

@app.route('/speech/say/bullshit', methods=['POST'])
def speakBullshit():
    speechModule.speakBullshit()
    return getResponse("OK")

@app.route('/speech/get/<textCategory>', methods=['GET'])
def getTextsBullshit(textCategory):
    methods = {'random': speechModule.getRandomTexts,
               'bullshit': speechModule.getBullshitTexts,
               'left': teensyCommunicator.moveLeftStep,
               'right': teensyCommunicator.moveRightStep
               }
    texts = []
    if textCategory in methods:
        texts = methods[textCategory]()
    return getResponse(json.dumps(texts))

@app.route('/face/image', methods=['GET'])
def getface():
    global app
    response = make_response(send_file("../" + faceModule.getScreenshotFilename()))
    response.headers['Cache-control'] = 'no-cache'
    response.headers['Access-Control-Allow-Origin'] = '*'
    return response

def getResponse(responseString):
    resp = make_response(responseString)
    resp.headers['Access-Control-Allow-Origin'] = '*'
    return resp

# This function is used as Thread to open and handle the webserver
def webserver():
    global app, webserverDebug, webserverHost, webserverPort
    app.run(debug=webserverDebug, host=webserverHost, port=webserverPort)

def start():
    print("starting webserverModule...")
    WebserverThread = threading.Thread(target=webserver)
    WebserverThread.daemon = True
    WebserverThread.start()

def stop():
    print("stopping webserverModule...")
    # TODO: stop flask?!

# TODO: move to util module
def translateIntRange(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return abs(rightMin + (valueScaled * rightSpan))

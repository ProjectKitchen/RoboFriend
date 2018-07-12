# external modules
from flask import Flask, render_template, request, redirect, url_for, make_response, send_file
import threading
import urllib
import json

#own modules
import ioWarriorModule
import teensyCommunicator
import legacyApiModule
import statusModule

#init
app = Flask(__name__, static_folder='../static')
webserverDebug = False
webserverHost = '0.0.0.0'
webserverPort = 8765

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

@app.route('/ear/color/<earColorR>/<earColorG>/<earColorB>', methods=['POST'])
def setEarRGB(earColorR, earColorG, earColorB):
    r = translateIntRange(int(earColorR), 0, 255, 0, 15)
    g = translateIntRange(int(earColorG), 0, 255, 0, 15)
    b = translateIntRange(int(earColorB), 0, 255, 0, 15)
    ioWarriorModule.setEarColor(r, g, b)
    return getResponse("OK")

@app.route('/move/<left>/<right>/<duration>', methods=['POST'])
def move(left, right, duration):
    teensyCommunicator.move(left, right, duration)
    return getResponse("OK")

@app.route('/get/status', methods=['GET'])
def getStatus():
    return getResponse(json.dumps(statusModule.getStatus()))

@app.route('/<action>', methods=['POST'])
# webserver rerouting - action indicates the chosen command which will be decoded and then interpreted with the function chooseAction
def reroute(action):
    print(action)
    try:
        action = urllib.unquote(action).decode('utf8') #decode action to string
    except:
        print("action is not valid")
    else:
        legacyApiModule.chooseAction(action)
    return getResponse("OK")

def getResponse(responseString):
    resp = make_response(responseString)
    resp.headers['Access-Control-Allow-Origin'] = '*'
    return resp

# This function is used as Thread to open and handle the webserver
def webserver():
    global app, webserverDebug, webserverHost, webserverPort
    app.run(debug=webserverDebug, host=webserverHost, port=webserverPort)

def start():
    print "starting webserverModule..."
    WebserverThread = threading.Thread(target=webserver)
    WebserverThread.daemon = True
    WebserverThread.start()

def stop():
    print "stopping webserverModule..."
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
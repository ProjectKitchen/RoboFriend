function Communicator() {
    var thiz = this;
    var hostname = window.location.hostname != 'localhost' ? window.location.hostname : '192.168.1.137';
    var RESTbaseUrl = "http://" + hostname + ":8765/";

    thiz.sendAction = function (actionString) {
        sendHttpPost(actionString);
    };

    thiz.sendMove = function (directionConstant) {
        switch (directionConstant) {
            case CONST.DIR_UP:
                thiz.sendAction(CONST.MOVECMD_FORWARD);
                break;
            case CONST.DIR_DOWN:
                thiz.sendAction(CONST.MOVECMD_BACKWARD);
                break;
            case CONST.DIR_LEFT:
                thiz.sendAction(CONST.MOVECMD_LEFT);
                break;
            case CONST.DIR_RIGHT:
                thiz.sendAction(CONST.MOVECMD_RIGHT);
                break;
        }
        if(CONST.MOVECOMMANDS.includes(directionConstant)) {
            thiz.sendAction(directionConstant);
        }
    };

    this.say = function (stringToSay) {
        if(stringToSay) {
            thiz.sendAction(CONST.CMD_SAY + stringToSay)
        }
    };

    thiz.sendMoveXY = function (x,y) {
        console.log('sending move xy: ' + x + " " + y);
    };

    thiz.initVideo = function () {
        var url = "http://" + hostname + ":8080/?action=stream";
        L('#webcam').setAttribute("src", url);
    };

    function sendHttpPost(restPath, successCallback, errorCallback) {
        console.log('sending: ' + restPath);
        var url = RESTbaseUrl + restPath;
        var xmlHttp = new XMLHttpRequest();
        xmlHttp.onreadystatechange = function() {
            if (xmlHttp.readyState === 4) {
                if (xmlHttp.status === 200) {
                    if(successCallback) successCallback(xmlHttp.responseText);
                } else {
                    if(errorCallback) errorCallback();
                }
            }
        };
        xmlHttp.open("POST", url);
        xmlHttp.send();
    }

}

window.communicator = new Communicator();



function Communicator() {
    var thiz = this;
    var hostname = window.location.hostname != 'localhost' ? window.location.hostname : '192.168.1.137';
    var RESTbaseUrl = "http://" + hostname + ":8765/";
    var defaultMoveDuration = 50;

    thiz.sendAction = function (actionString) {
        sendHttp(actionString);
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

    thiz.say = function (stringToSay) {
        if(stringToSay) {
            thiz.sendAction(CONST.CMD_SAY + stringToSay)
        }
    };

    thiz.sendMoveXY = function (x,y) {
        thiz.sendAction('move/' + x + '/' + y + '/' + defaultMoveDuration);
    };

    thiz.getStatus = function () {
        return sendHttp('get/status', 'GET');
    };

    thiz.initVideo = function () {
        var url = "http://" + hostname + ":8080/?action=stream";
        L('#webcam').setAttribute("src", url);
    };

    function sendHttp(restPath, method) {
        method = method || 'POST';
        console.log('sending (' + method + '): ' + restPath);
        return new Promise((resolve, reject) => {
            var url = RESTbaseUrl + restPath;
            var xmlHttp = new XMLHttpRequest();
            xmlHttp.onreadystatechange = function() {
                if (xmlHttp.readyState === 4) {
                    if (xmlHttp.status === 200) {
                        resolve(xmlHttp.responseText);
                    } else {
                        reject();
                    }
                }
            };
            xmlHttp.open(method, url);
            xmlHttp.send();
        });
    }

}

window.communicator = new Communicator();



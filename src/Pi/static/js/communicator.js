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
                thiz.sendAction('move/simple/forward');
                break;
            case CONST.DIR_DOWN:
                thiz.sendAction('move/simple/backward');
                break;
            case CONST.DIR_LEFT:
                thiz.sendAction('move/simple/left');
                break;
            case CONST.DIR_RIGHT:
                thiz.sendAction('move/simple/right');
                break;
        }
    };

    thiz.moveStop = function () {
        thiz.sendAction('move/stop');
    };

    thiz.say = function (stringToSay) {
        if(stringToSay) {
            thiz.sendAction('speech/say/' + stringToSay)
        }
    };

    thiz.sendMoveXY = function (x,y) {
        thiz.sendAction('move/flex/' + x + '/' + y + '/' + defaultMoveDuration);
    };

    thiz.getStatus = function () {
        return new Promise(resolve => {
            sendHttp('get/status', 'GET').then(result => {
                resolve(JSON.parse(result));
            });
        })
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



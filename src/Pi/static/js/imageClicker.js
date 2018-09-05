function ImageClicker() {
    var thiz = this;
    thiz.isMoving = false;
    thiz.lastMove = new Date().getTime();
    thiz.eyeMovePause = 500;
    thiz.cameraPause = 200;
    thiz.movePause = 200;
    thiz.eyeTimeoutHandler = null;
    thiz.moveTimeoutHandler = null;
    thiz.cameraTimeoutHandler = null;
    thiz.moveL = 0;
    thiz.moveR = 0;
    thiz.moveCam = 0;

    thiz.webcamMouseDown = function (event) {
        event.preventDefault();
        thiz.isMoving = true;
        calcMovementValues(event);
        wheelMove();
        cameraMove();
    };

    thiz.mouseMoved = function (event) {
        event.preventDefault();
        //wheel movement
        calcMovementValues(event);

        //eye movement
        if (thiz.eyeTimeoutHandler) {
            clearInterval(thiz.eyeTimeoutHandler);
        }
        if (new Date().getTime() - thiz.lastMove > thiz.eyeMovePause) {
            setEyeDirection(event)
        } else {
            thiz.eyeTimeoutHandler = setTimeout(function () {
                setEyeDirection(event)
            }, thiz.eyeMovePause);
        }
    };

    thiz.stopMovement = function () {
        thiz.isMoving = false;
        if (thiz.moveTimeoutHandler) {
            clearInterval(thiz.moveTimeoutHandler);
        }
        if (thiz.cameraTimeoutHandler) {
            clearInterval(thiz.cameraTimeoutHandler);
        }
        communicator.moveStop();
        thiz.moveL = 0;
        thiz.moveR = 0;
        thiz.moveCam = 0;
    };

    thiz.stop = function (event) {
        if (thiz.eyeTimeoutHandler) {
            clearInterval(thiz.eyeTimeoutHandler);
        }
        thiz.stopMovement();
        communicator.setEyes(0, 0);
    };

    function setEyeDirection(event) {
        thiz.lastMove = new Date().getTime();
        var rect = L('#' + event.target.id).getBoundingClientRect();
        var x = event.x - rect.left;
        var y = event.y - rect.top;
        var px = Math.round((((x / rect.width) * 100) - 50) * -2); // +/- 100%, middle is 0 %
        var py = Math.round((((y / rect.height) * 100) - 50) * 2);
        communicator.setEyes(px, py);
    }

    function cameraMove() {
        if(!thiz.isMoving) return;

        if (thiz.moveCam) {
            communicator.changeCamera(thiz.moveCam);
        }
        thiz.cameraTimeoutHandler = setTimeout(function () {
            cameraMove();
        }, thiz.cameraPause);
    }

    function wheelMove() {
        communicator.sendMoveXY(thiz.moveL, thiz.moveR, 100);
        thiz.moveTimeoutHandler = setTimeout(function () {
            wheelMove();
        }, thiz.movePause);
    }

    function calcMovementValues(event) {
        if(!thiz.isMoving) return;

        var rect = L('#' + event.target.id).getBoundingClientRect();
        var x = event.x - rect.left;
        var y = event.y - rect.top;
        var px = (x / rect.width) * 100;
        var py = (y / rect.height) * 100;

        var isLeft = px < 40;
        var isRight = px > 60;
        var isUp = py < 30;
        var isDown = py > 70;
        var isMiddleY = !isUp && !isDown;
        var isMiddleX = !isLeft && !isRight;
        var isMoving = thiz.moveL != 0 || thiz.moveR != 0;
        var shouldMove = isMoving || isMiddleY;
        if (shouldMove && isMiddleX) {
            thiz.moveL = 100;
            thiz.moveR = 100;
        } else if (shouldMove && isLeft) {
            thiz.moveL = Math.round(((px / 40)) * 200 - 100);
            thiz.moveR = Math.round(((px / 40)) * 0 + 100);
        } else if (shouldMove && isRight) {
            thiz.moveL = Math.round((1 - ((px - 60) / 40)) * 0 + 100);
            thiz.moveR = Math.round((1 - ((px - 60) / 40)) * 200 - 100);
        } else {
            thiz.moveL = 0;
            thiz.moveR = 0;
        }

        if (isUp) {
            thiz.moveCam = 4;
        } else if (isDown) {
            thiz.moveCam = -4;
        } else {
            thiz.moveCam = 0;
        }
    }
}

window.imageClicker = new ImageClicker();



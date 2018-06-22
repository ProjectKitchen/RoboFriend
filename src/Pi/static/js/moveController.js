function MoveController() {
    var thiz = this;
    var currentEvent = {};
    var waitBeforeShowDot = 300;
    var waitBeforeStartMoving = 0;
    var moveSendInterval = 500;
    var deadzone = 5;
    var coordinateValues = [0, 150, 300];
    var interpolatorLeft = new Interpolator(coordinateValues, coordinateValues, [
        [0, 255, 255],
        [-255, 0, 255],
        [0, -255, -255]
    ]);
    var interpolatorRight = new Interpolator(coordinateValues, coordinateValues, [
        [255, 255, 0],
        [255, 0, -255],
        [-255, -255, 0]
    ]);
    var array = Array.from({length: 61}, (v, k) => (k*5));
    console.log(toCSV(interpolatorLeft.interpolateBilinearArray(array, array)));
    console.log(toCSV(interpolatorRight.interpolateBilinearArray(array, array)));

    function toCSV(data) {
        var lineArray = [];
        data.forEach(function (infoArray, index) {
            var line = infoArray.join(";");
            lineArray.push(line);
        });
        return lineArray.join("\n");
    }

    thiz.doNothing = function(event) {
        event.preventDefault();
        return false;
    };

    thiz.clickstart = function(event, direction) {
        if(currentEvent.mouseX || (event instanceof KeyboardEvent && event.keyCode != 13 && event.keyCode != 32)) {
            return; // prevent duplicated event from container
        }
        currentEvent.direction = direction;
        if(!(event instanceof KeyboardEvent)) {
            currentEvent.originalMouseX = getXVal(event);
            currentEvent.originalMouseY = getYVal(event);
            currentEvent.mouseX = getXVal(event);
            currentEvent.mouseY = getYVal(event);
            event.preventDefault();
            calcMoveXY();
            currentEvent.timer = setTimeout(function () {
                if(currentEvent.mouseX) {
                    setPosIndicator(currentEvent.mouseX, currentEvent.mouseY);
                    L.setVisible('#posIndicator');
                    L.setVisible('#posIndicatorHelpers');
                    currentEvent.actionSendTimeoutHandler = setTimeout(function () {
                        currentEvent.actionSendIntervalHandler = setInterval(function () {
                            currentEvent.didDynamicMovement = true;
                            communicator.sendMoveXY(currentEvent.moveLeft, currentEvent.moveRight);
                        }, moveSendInterval);
                    }, waitBeforeStartMoving);
                }
            }, waitBeforeShowDot);
        }
        document.onmousemove = evalMouseMove;
        document.ontouchmove = evalMouseMove;
        document.onmouseup = evalMouseUp;
        document.ontouchend = evalMouseUp;
        document.onkeyup = evalMouseUp;
    };

    function evalMouseMove(event) {
        currentEvent.mouseX = getXVal(event);
        currentEvent.mouseY = getYVal(event);
        calcMoveXY();
    }

    function getXVal(event) {
        return event.pageX != undefined ? event.pageX : event.touches[0].pageX;
    }

    function getYVal(event) {
        return event.pageY != undefined ? event.pageY : event.touches[0].pageY;
    }

    function calcMoveXY() {
        var boundingRect = L('#navigation-box').getBoundingClientRect();
        var bodyRect = L('body')[0].getBoundingClientRect();
        var xVal = L.cutToIntRange(currentEvent.mouseX, boundingRect.left - bodyRect.left + 3, boundingRect.right - bodyRect.left - 3); //3px = border
        var yVal = L.cutToIntRange(currentEvent.mouseY, boundingRect.top - bodyRect.top + 3, boundingRect.bottom - bodyRect.top - 3);
        setPosIndicator(xVal, yVal);
        var xValNorm = (xVal - boundingRect.left - (bodyRect.left * (-1)) - 3); //3px = border
        var yValNorm = (yVal - boundingRect.top - (bodyRect.top * (-1)) - 3);
        currentEvent.moveLeft = Math.round(interpolatorLeft.interpolateBilinear(xValNorm, yValNorm));
        currentEvent.moveRight = Math.round(interpolatorRight.interpolateBilinear(xValNorm, yValNorm));
        currentEvent.moveLeft = Math.abs(currentEvent.moveLeft) >= deadzone ? currentEvent.moveLeft : 0;
        currentEvent.moveRight = Math.abs(currentEvent.moveRight) >= deadzone ? currentEvent.moveRight : 0;
        currentEvent.moveLeft = L.cutToIntRange(currentEvent.moveLeft, -255, 255);
        currentEvent.moveRight = L.cutToIntRange(currentEvent.moveRight, -255, 255);
        L('#liveMoveLeft').innerHTML = currentEvent.moveLeft;
        L('#liveMoveRight').innerHTML = currentEvent.moveRight;
    }

    function setPosIndicator(x, y) {
        L('#posIndicator').style.top = y + 'px';
        L('#posIndicator').style.left = x + 'px';
    }

    function evalMouseUp() {
        var hasMoved = currentEvent.originalMouseX != currentEvent.mouseX || currentEvent.originalMouseY != currentEvent.mouseY;
        if(!hasMoved && currentEvent.direction && currentEvent.direction != CONST.DIR_NONE && !currentEvent.didDynamicMovement) {
            communicator.sendMove(currentEvent.direction);
        } else {
            communicator.sendMove(CONST.MOVECMD_STOP);
        }

        //reset
        document.onmousemove = null;
        document.ontouchmove = null;
        document.onmouseup = null;
        document.ontouchend = null;
        document.onkeyup = null;
        clearTimeout(currentEvent.actionSendTimeoutHandler);
        clearInterval(currentEvent.actionSendIntervalHandler);
        currentEvent = {};
        L.setVisible('#posIndicator', false);
        L.setVisible('#posIndicatorHelpers', false);
    }

}

window.moveController = new MoveController();



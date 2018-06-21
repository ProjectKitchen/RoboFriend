function MoveController() {
    var thiz = this;
    var currentEvent = {};
    var waitBeforeShowDot = 300;
    var waitBeforeStartMoving = 0;
    var moveSendInterval = 500;

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
                            communicator.sendMoveXY(currentEvent.moveX, currentEvent.moveY);
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
        var moveX = (xVal - boundingRect.left - (bodyRect.left * (-1)) - 3); //3px = border
        var moveY = (yVal - boundingRect.top - (bodyRect.top * (-1)) - 3);
        currentEvent.moveX = Math.round(L.mapRange(moveX, 0, 300, -255, 255));
        currentEvent.moveY = Math.round(L.mapRange(moveY, 0, 300, -255, 255) * (-1));
        L('#livePosX').innerHTML = currentEvent.moveX;
        L('#livePosY').innerHTML = currentEvent.moveY;
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



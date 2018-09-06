function KeyController() {
    var thiz = this;
    thiz.keyCodeUp = 87; //W
    thiz.keyCodeDown = 83; //A
    thiz.keyCodeLeft = 65;//S
    thiz.keyCoderight = 68; //D

    thiz.lastMove = new Date().getTime();
    thiz.movePause = 400;
    thiz.pressedLeft = false;
    thiz.pressedRight = false;
    thiz.pressedUp = false;
    thiz.pressedDown = false;
    thiz.moving = false;

    thiz.init = function () {
        document.addEventListener('keydown', function (event) {
            switch (event.keyCode) {
                case thiz.keyCodeUp:
                    thiz.moving = true;
                    thiz.pressedUp = true;
                    sendMoveInternal(event);
                    break;
                case thiz.keyCodeDown:
                    thiz.moving = true;
                    thiz.pressedDown = true;
                    sendMoveInternal(event);
                    break;
                case thiz.keyCodeLeft:
                    thiz.moving = true;
                    thiz.pressedLeft = true;
                    sendMoveInternal(event);
                    break;
                case thiz.keyCoderight:
                    thiz.moving = true;
                    thiz.pressedRight = true;
                    sendMoveInternal(event);
                    break;
                case 27: //ESC
                    event.preventDefault();
                    stopMoving();
                    imageClicker.stop();
                    break;
            }
        });
        document.addEventListener('keyup', function (event) {
            switch (event.keyCode) {
                case thiz.keyCodeUp:
                    thiz.pressedUp = false;
                    break;
                case thiz.keyCodeDown:
                    thiz.pressedDown = false;
                    break;
                case thiz.keyCodeLeft:
                    thiz.pressedLeft = false;
                    break;
                case thiz.keyCoderight:
                    thiz.pressedRight = false;
                    break;
            }
            if (!thiz.pressedUp && !thiz.pressedDown && !thiz.pressedLeft && !thiz.pressedRight) {
                stopMoving();
            }
        });
    };

    function stopMoving() {
        thiz.moving = false;
        thiz.pressedUp = false;
        thiz.pressedDown = false;
        thiz.pressedLeft = false;
        thiz.pressedRight = false;
        communicator.moveStop();
    }

    function sendMoveInternal(event) {
        event.preventDefault();
        if (new Date().getTime() - thiz.lastMove > thiz.movePause && thiz.moving) {
            thiz.lastMove = new Date().getTime();
            if(thiz.pressedUp) {
                communicator.sendMoveXY(100, 100, 100);
            }
            if(thiz.pressedDown) {
                communicator.sendMoveXY(-100, -100, 100);
            }
            if(thiz.pressedLeft) {
                communicator.sendMoveXY(-50, 50, 50);
            }
            if(thiz.pressedRight) {
                communicator.sendMoveXY(50, -50, 50);
            }
        }
        if(thiz.moving) {
            setTimeout(function () {
                sendMoveInternal(event);
            }, thiz.movePause + 1)
        }
    }
}

window.keyController = new KeyController();



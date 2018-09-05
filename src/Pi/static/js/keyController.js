function KeyController() {
    var thiz = this;
    thiz.lastMove = new Date().getTime();
    thiz.movePause = 200;
    thiz.pressedLeft = false;
    thiz.pressedRight = false;
    thiz.pressedUp = false;
    thiz.pressedDown = false;
    thiz.moving = false;

    thiz.init = function () {
        document.addEventListener('keydown', function (event) {
            switch (event.keyCode) {
                case 38: //UP
                    thiz.moving = true;
                    thiz.pressedUp = true;
                    sendMoveInternal(event);
                    break;
                case 40: //DOWN
                    thiz.moving = true;
                    thiz.pressedDown = true;
                    sendMoveInternal(event);
                    break;
                case 37: //LEFT
                    thiz.moving = true;
                    thiz.pressedLeft = true;
                    sendMoveInternal(event);
                    break;
                case 39: //RIGHT
                    thiz.moving = true;
                    thiz.pressedRight = true;
                    sendMoveInternal(event);
                    break;
            }
        });
        document.addEventListener('keyup', function (event) {
            switch (event.keyCode) {
                case 38: //UP
                    thiz.pressedUp = false;
                    break;
                case 40: //DOWN
                    thiz.pressedDown = false;
                    break;
                case 37: //LEFT
                    thiz.pressedLeft = false;
                    break;
                case 39: //RIGHT
                    thiz.pressedRight = false;
                    break;
            }
            if (!thiz.pressedUp && !thiz.pressedDown && !thiz.pressedLeft && !thiz.pressedRight) {
                communicator.moveStop();
                thiz.moving = false;
            }
        });
    };

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
            }, 100)
        }
    }
}

window.keyController = new KeyController();



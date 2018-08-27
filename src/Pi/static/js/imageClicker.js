function ImageClicker() {
    var thiz = this;

    thiz.webcamClicked = function(event) {
        var rect = L('#webcamPlaceholder').getBoundingClientRect();
        var x = event.x - rect.left;
        var y = event.y - rect.top;
        var px = (x/rect.width) * 100;
        var py = (y/rect.height) * 100;

        var isLeft = px < 30;
        var isRight = px > 70;
        var isUp = py < 30;
        var isDown = py > 70;
        var isMiddleY = !isUp && !isDown;
        var isMiddleX = !isLeft && !isRight;
        if(isMiddleY && isLeft){
            communicator.sendMoveXY(-50,50,1);
        }
        if(isMiddleY && isRight){
            communicator.sendMoveXY(50,-50,1);
        }
    };
}

window.imageClicker = new ImageClicker();



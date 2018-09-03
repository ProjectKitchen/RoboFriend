function EarColorService() {
    var thiz = this;
    thiz.randomOn = false;
    thiz.randomMaxInc = 15;
    thiz.randomTimeoutHandler = null;
    thiz.timeoutHandler = null;
    thiz.sendPause = 100;
    thiz.r = getRandomInt(20,255);
    thiz.g = getRandomInt(20,255);
    thiz.b = getRandomInt(20,255);
    thiz.factorR = 1;
    thiz.factorG = 1;
    thiz.factorB = 1;

    thiz.onMousedown = function (event) {
        calcColor(event);
        setColor();
    };

    thiz.onMouseMove = function (event) {
        calcColor(event);
    };

    thiz.random = function () {
        if(thiz.randomOn) {
            thiz.randomOn = false;
            thiz.stop();
        } else {
            thiz.randomOn = true;
            randomColor();
        }
    };

    thiz.stop = function () {
        if (thiz.timeoutHandler) {
            clearInterval(thiz.timeoutHandler);
        }
        if (thiz.randomTimeoutHandler) {
            clearInterval(thiz.randomTimeoutHandler);
        }
    };

    function setColor() {
        communicator.sendAction('ear/color/' + thiz.r + '/' + thiz.g + '/' + thiz.b);
        L('#colorPicker').value = rgbToHex(thiz.r, thiz.g, thiz.b);
        thiz.timeoutHandler = setTimeout(function () {
            setColor();
        }, thiz.sendPause);
    }

    function randomColor() {
        thiz.r = (thiz.r + getRandomInt(1, thiz.randomMaxInc) * thiz.factorR);
        thiz.g = (thiz.g + getRandomInt(1, thiz.randomMaxInc) * thiz.factorG);
        thiz.b = (thiz.b + getRandomInt(1, thiz.randomMaxInc) * thiz.factorB);
        if(thiz.r > 255 || thiz.r < 0) {
            thiz.r = thiz.r > 255 ? 255 : 0;
            thiz.factorR *= -1;
        }
        if(thiz.g > 255 || thiz.g < 0) {
            thiz.g = thiz.g > 255 ? 255 : 0;
            thiz.factorG *= -1;
        }
        if(thiz.b > 255 || thiz.b < 0) {
            thiz.b = thiz.b > 255 ? 255 : 0;
            thiz.factorB *= -1;
        }
        communicator.sendAction('ear/color/' + thiz.r + '/' + thiz.g + '/' + thiz.b);
        L('#colorPicker').value = rgbToHex(thiz.r, thiz.g, thiz.b);
        thiz.randomTimeoutHandler = setTimeout(function () {
            randomColor();
        }, thiz.sendPause);
    }

    function calcColor (event) {
        // get click position in clicked image
        var elem = L('#' + event.target.id);
        var rect = elem.getBoundingClientRect();
        var xpos = event.x - rect.left;
        var ypos = event.y - rect.top;
        var px = (xpos / rect.width);
        var py = (ypos / rect.height);

        // get color at clicked pixel
        var myImg = new Image();
        myImg.src = event.target.src;
        var canvas = document.createElement('canvas');
        var context = canvas.getContext('2d');
        canvas.width = elem.naturalWidth;
        canvas.height = elem.naturalHeight;
        context.drawImage(myImg, 0, 0);
        var x = Math.round(px * elem.naturalWidth);
        var y = Math.round(py * elem.naturalHeight);
        var data = context.getImageData(x, y, 1, 1).data;
        console.log(x + " " + y)
        console.log(elem.naturalWidth + " " + elem.naturalHeight)
        thiz.r = data[0];
        thiz.g = data[1];
        thiz.b = data[2];
    };
}

function rgbToHex(r, g, b) {
    return "#" + ((1 << 24) + (r << 16) + (g << 8) + b).toString(16).slice(1);
}

function getRandomInt(min, max) {
    return Math.floor(Math.random() * (max - min + 1)) + min;
}

window.earColorService = new EarColorService();



function EarColorService() {
    var thiz = this;
    thiz.sendPause = 100;
    thiz.lastSend = new Date().getTime();
    thiz.r = 0;
    thiz.g = 0;
    thiz.b = 0;

    thiz.onchange = function (event) {
        calcColor(event);
        setColor();
    };

    thiz.randomOn = function () {
        L.toggle('.randomEarBtn');
        communicator.sendAction('ear/color/random/on')
    };

    thiz.randomOff = function () {
        L.toggle('.randomEarBtn');
        communicator.sendAction('ear/color/random/off')
    };

    function setColor() {
        L('#colorPicker').value = rgbToHex(thiz.r, thiz.g, thiz.b);
        if(new Date().getTime() - thiz.lastSend > thiz.sendPause) {
            thiz.lastSend = new Date().getTime();
            communicator.sendAction('ear/color/' + thiz.r + '/' + thiz.g + '/' + thiz.b);
        }
    }

    function calcColor (event) {
        var imgElem = L('#colorPickerImg');
        var px = parseInt(event.target.value) / 100;

        // get color at clicked pixel
        var myImg = new Image();
        myImg.src = imgElem.src;
        var canvas = document.createElement('canvas');
        var context = canvas.getContext('2d');
        canvas.width = imgElem.naturalWidth;
        canvas.height = imgElem.naturalHeight;
        context.drawImage(myImg, 0, 0);
        var x = Math.round(px * imgElem.naturalWidth);
        var y = 1;
        var data = context.getImageData(x, y, 1, 1).data;
        thiz.r = data[0];
        thiz.g = data[1];
        thiz.b = data[2];
    }
}

function rgbToHex(r, g, b) {
    return "#" + ((1 << 24) + (r << 16) + (g << 8) + b).toString(16).slice(1);
}

window.earColorService = new EarColorService();



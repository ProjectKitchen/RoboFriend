function SpeechComboHandler() {
    var thiz = this;

    thiz.init = function () {
        communicator.sendAction('speech/get/random').then(function (response) {
            fillCombo('comboRandom', JSON.parse(response));
        });
        communicator.sendAction('speech/get/bullshit').then(function (response) {
            fillCombo('comboBullshit', JSON.parse(response));
        });
    };

    thiz.selected = function (event) {
        L('#customText').value = event.target.value;
        communicator.say(event.target.value);
    };

    function fillCombo(comboId, data) {
        var elem = L('#' + comboId);
        if(data && data instanceof  Array) {
            data.forEach(function (textElem) {
                elem.innerHTML += `<option value="${textElem}">${textElem}</option>`
            });
        }
    }
}

window.speechComboHandler = new SpeechComboHandler();



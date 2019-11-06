function ComboHandler() {
    var thiz = this;

    thiz.init = function () {
        communicator.sendAction('speech/get/random', 'GET').then(function (response) {
            fillCombo('comboRandom', JSON.parse(response));
        });
        communicator.sendAction('speech/get/bullshit', 'GET').then(function (response) {
            fillCombo('comboBullshit', JSON.parse(response));
        });
        communicator.sendAction('sound/get/random', 'GET').then(function (response) {
            fillCombo('comboSounds', JSON.parse(response));
        });
    };

    thiz.selectedSpeech = function (event) {
        L('#customText').value = event.target.value;
        communicator.say(event.target.value);
    };

    thiz.selectedSound = function (event) {
        communicator.sendAction('sound/play/file/' + event.target.value);
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

window.comboHandler = new ComboHandler();



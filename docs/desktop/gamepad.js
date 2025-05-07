// Gamepad code
(function () {
    // Essential variables
    var haveEvents = 'GamepadEvent' in window;
    var haveWebkitEvents = 'WebKitGamepadEvent' in window;
    var controllers = {};
    var gamepadStatus = false;
    var saveElement = document.getElementById('savePrefs');
    var loadElement = document.getElementById('loadPrefs');
    var restoreElement = document.getElementById('restoreDefaults');
    var gamepadElement = document.getElementById('gamepadSelection');
    var gamepadSettingsElement = document.getElementById('gamepadSettings');
    gamepadElement.style.display = 'none';
    // User selection, -1 means no choice
    var gamepadChoices = [-1, -1, -1, -1, -1, -1, -1, -1];
    var axesChoices = [-1, -1, -1, -1];
    var rAF = window.mozRequestAnimationFrame || window.requestAnimationFrame;
    // Connection handler
    function connecthandler(e) {
        addgamepad(e.gamepad);
    }
    // New gamepad
    function addgamepad(gamepad) {
        controllers[gamepad.index] = gamepad;
        // Create DOM per gamepad with specific id that starts from 0
        var controller = document.createElement('div');
        controller.setAttribute('id', 'controller-' + gamepad.index);
        var cname = document.createElement('h6');
        var gid = document.createElement('b');
        gid.className = 'text-success';
        gid.appendChild(document.createTextNode(gamepad.id));
        cname.appendChild(document.createTextNode('Gamepad: '));
        cname.appendChild(gid);
        controller.appendChild(cname);
        // Add gamepad buttons table
        var row = document.createElement('div');
        row.className = 'row';
        var buttons = document.createElement('div');
        buttons.className = 'table-responsive col';
        // Add buttons title
        var buttonsTitle = document.createElement('h6');
        buttonsTitle.appendChild(document.createTextNode('Buttons'));
        buttons.appendChild(buttonsTitle);
        var btable = document.createElement('table');
        btable.className = 'table gamepad-table';
        var btbody = document.createElement('tbody');
        for (var i = 0; i < gamepad.buttons.length; i++) {
            var btr = document.createElement('tr');
            var bth = document.createElement('th');
            bth.innerHTML = i;
            bth.setAttribute('scope', 'row');
            var btd = document.createElement('td');
            btd.className = 'padButton text-danger';
            btd.innerHTML = 'Not pressed';
            btr.appendChild(bth);
            btr.appendChild(btd);
            btbody.appendChild(btr);
        }
        btable.appendChild(btbody);
        buttons.appendChild(btable);
        // Add buttons content
        row.appendChild(buttons);
        // Add axes
        var axis = document.createElement('div');
        axis.className = 'table-responsive col';
        // Add axis title
        var axisTitle = document.createElement('h6');
        axisTitle.appendChild(document.createTextNode('Axes'));
        axis.appendChild(axisTitle);
        var atable = document.createElement('table');
        atable.className = 'table gamepad-table';
        var atbody = document.createElement('tbody');
        for (i = 0; i < gamepad.axes.length; i++) {
            var atr = document.createElement('tr');
            var ath = document.createElement('th');
            ath.innerHTML = i;
            ath.setAttribute('scope', 'row');
            var atd = document.createElement('td');
            var progress = document.createElement('div');
            progress.className = 'progress gamepad-progress';
            var bar = document.createElement('div');
            bar.className = 'progress-bar axes';
            bar.setAttribute('role', 'progressbar');
            bar.setAttribute('style', 'width: ' + 0 + '%;');
            bar.innerHTML = 0 + '%';
            progress.appendChild(bar);
            atd.appendChild(progress);
            atr.appendChild(ath);
            atr.appendChild(atd);
            atbody.appendChild(atr);
        }
        atable.appendChild(atbody);
        axis.appendChild(atable);
        // Add axis content
        row.appendChild(axis);
        controller.appendChild(row);
        // Hide message and display settings
        document.getElementById('start').style.display = 'none';
        gamepadSettingsElement.appendChild(controller);
        rAF(updateStatus);
    }
    // Disconnect controller
    function disconnecthandler(e) {
        removegamepad(e.gamepad);
        gamepadStatus = false;
        gamepadChoices = [-1, -1, -1, -1, -1, -1, -1, -1];
        axesChoices = [-1, -1, -1, -1];
        gamepadElement.style.display = 'none';
    }
    // Remove controller 
    function removegamepad(gamepad) {
        var d = document.getElementById('controller-' + gamepad.index);
        document.getElementById('start').style.display = 'block';
        gamepadSettingsElement.removeChild(d);
        delete controllers[gamepad.index];
    }
    // Check buttons and axis state
    function updateStatus() {
        scangamepads();
        for (j in controllers) {
            var controller = controllers[j];
            var d = document.getElementById('controller-' + j);
            var buttons = d.getElementsByClassName('padButton');
            for (var i = 0; i < controller.buttons.length; i++) {
                var b = buttons[i];
                var val = controller.buttons[i];
                var pressed = val === 1.0;
                var touched = false;
                if (typeof (val) === 'object') {
                    pressed = val.pressed;
                    if ('touched' in val) {
                        touched = val.touched;
                    }
                    val = val.value;
                }
                // Change buttons UI in settings
                b.innerHTML = 'Not pressed';
                b.className = 'padButton text-danger';
                if (pressed) {
                    b.innerHTML = 'Pressed';
                    b.className = 'padButton text-success';
                    buttonsHandler(i);
                }
            }
            // Change axis UI in settings
            var axes = d.getElementsByClassName('axes');
            for (var i = 0; i < controller.axes.length; i++) {
                var a = axes[i];
                // Normalize values from -1 to 1, 0 to 100
                var normalized = (controller.axes[i] - (-1)) / (1 - (-1)) * 100;
                a.innerHTML = parseInt(normalized) + '%';
                a.setAttribute('style', 'width: ' + parseInt(normalized) + '%;');
                axesHandler(i, controller.axes[i]);
            }
            if (!gamepadStatus) {
                var buttonsIds = 0;
                var axesIds = 0;
                buttonsIds = controller.buttons.length
                axesIds = controller.axes.length;
                connectGamepadROS(buttonsIds, axesIds);
            }
        }
        rAF(updateStatus);
    }
    // Scan for new gamepads
    function scangamepads() {
        var gamepads = navigator.getGamepads ? navigator.getGamepads() : (navigator.webkitGetGamepads ? navigator.webkitGetGamepads() : []);
        for (var i = 0; i < gamepads.length; i++) {
            if (gamepads[i] && (gamepads[i].index in controllers)) {
                controllers[gamepads[i].index] = gamepads[i];
            }
        }
    }
    // Events
    if (haveEvents) {
        window.addEventListener('gamepadconnected', connecthandler);
        window.addEventListener('gamepaddisconnected', disconnecthandler);
    } else if (haveWebkitEvents) {
        window.addEventListener('webkitgamepadconnected', connecthandler);
        window.addEventListener('webkitgamepaddisconnected', disconnecthandler);
    } else {
        setInterval(scangamepads, 500);
    }
    // Gamepad and ROS connection
    function connectGamepadROS(bids, aids) {
        gamepadElement.style.display = 'block';
        gamepadStatus = true;
        for (var i = 0; i < (gamepadChoices.length + axesChoices.length); i++) {
            var list = document.getElementById('gamepad' + (i + 1));
            // Add listener to key selection
            list.addEventListener('change', (e) => {
                var gid = e.target.id;
                gid = gid.replace('gamepad', '');
                var id = parseInt(gid);
                // Save user selections
                if (id >= 1 && id < 9) {
                    gamepadChoices[id - 1] = parseInt(e.target.value);
                }
                if (id >= 9 && id < 13) {
                    axesChoices[id - 9] = parseInt(e.target.value);
                }
            });
        }
        for (var i = 0; i < gamepadChoices.length; i++) {
            var list = document.getElementById('gamepad' + (i + 1));
            list.innerHTML = '';
            var option = document.createElement('option');
            option.setAttribute('value', '-1');
            option.setAttribute('disabled', 'true');
            option.setAttribute('selected', 'true');
            option.innerHTML = 'Select button';
            list.appendChild(option);
            for (var j = 0; j < bids; j++) {
                var option = document.createElement('option');
                option.setAttribute('value', j);
                option.innerHTML = 'Button: ' + j;
                list.appendChild(option);
            }
        }
        for (var i = 0; i < axesChoices.length; i++) {
            var list = document.getElementById('gamepad' + (i + 9));
            list.innerHTML = '';
            var option = document.createElement('option');
            option.setAttribute('value', '-1');
            option.setAttribute('disabled', 'true');
            option.setAttribute('selected', 'true');
            option.innerHTML = 'Select axis';
            list.appendChild(option);
            for (var j = 0; j < aids; j++) {
                var option = document.createElement('option');
                option.setAttribute('value', j);
                option.innerHTML = 'Axis: ' + j;
                list.appendChild(option)
            }
        }
    }
    // Catch button events
    var buttonTimer = null;
    var buttonIsPressed = false;
    var delayMilliseconds = 25;
    function buttonsHandler(button) {
        var choice = -1;
        for (var i = 0; i < gamepadChoices.length; i++) {
            if (gamepadChoices[i] === button) {
                choice = i;
            }
        }
        if (choice !== -1) {
            if (!buttonIsPressed) {
                if (choice === 0) {
                    // Increase linear speed
                    changeSpeed(0, 0);
                } else if (choice === 1) {
                    // Decrease linear speed
                    changeSpeed(0, 1);
                } else if (choice === 2) {
                    // Increase angular speed
                    changeSpeed(1, 0);
                } else if (choice === 3) {
                    // Decrease angular speed
                    changeSpeed(1, 1);
                } else if (choice === 4) {
                    // Perform action 1
                    doAction(0);
                } else if (choice === 5) {
                    // Perform action 2
                    doAction(1);
                } else if (choice === 6) {
                    // Perform action 3
                    doAction(2);
                } else if (choice === 7) {
                    // Perform action 4
                    doAction(3);
                }
            }
            // Fire actions once when user press the key
            buttonIsPressed = true;
            clearTimeout(buttonTimer);
            buttonTimer = setTimeout(() => { buttonIsPressed = false; }, delayMilliseconds);
        }
    }
    // Catch axes events
    var stopTimer = [null, null];
    var stopActionsTimer = [null, null];
    function axesHandler(button, distance) {
        var choice = -1;
        for (var i = 0; i < axesChoices.length; i++) {
            if (axesChoices[i] === button) {
                choice = i;
                if (distance.toFixed(2) >= 0.1 || distance.toFixed(2) <= -0.1) {
                    if (choice === 0) {
                        // Set linear speed
                        setSpeed(0, distance);
                        clearTimeout(stopTimer[0]);
                        stopTimer[0] = setTimeout(() => { setSpeed(2, 0); }, delayMilliseconds);
                    } else if (choice === 1) {
                        // Set angular speed
                        setSpeed(1, distance);
                        clearTimeout(stopTimer[1]);
                        stopTimer[1] = setTimeout(() => { setSpeed(3, 0); }, delayMilliseconds);
                    }
                    else if (choice === 2) {
                        // Set movements linear speed
                        doMovement(0, distance);
                        clearTimeout(stopActionsTimer[0]);
                        stopActionsTimer[0] = setTimeout(() => { doMovement(2, 0); }, delayMilliseconds);
                    }
                    else if (choice === 3) {
                        // Set movements angular speed
                        doMovement(1, distance);
                        clearTimeout(stopActionsTimer[1]);
                        stopActionsTimer[1] = setTimeout(() => { doMovement(3, 0); }, delayMilliseconds);
                    }
                }
            }
        }
    }
    // Set save button listener
    saveElement.addEventListener('click', () => {
        // Change storage state
        localStorage.setItem('GAMEPAD_STATE', true);
        // Initialize data object
        var GAMEPAD_DATA = {};
        for (var i = 0; i < gamepadChoices.length; i++) {
            GAMEPAD_DATA['gamepadChoices' + i] = gamepadChoices[i];
        }
        for (var i = 0; i < axesChoices.length; i++) {
            GAMEPAD_DATA['axesChoices' + i] = axesChoices[i];
        }
        // Put the object into storage
        localStorage.setItem('GAMEPAD_DATA', JSON.stringify(GAMEPAD_DATA));
    });
    // Set load button listener
    loadElement.addEventListener('click', () => {
        gamepadChoices = [-1, -1, -1, -1, -1, -1, -1, -1];
        axesChoices = [-1, -1, -1, -1];
        if (localStorage.getItem('GAMEPAD_STATE') === 'true') {
            // Retrieve the object from local storage
            const JSON_DATA = localStorage.getItem('GAMEPAD_DATA');
            GAMEPAD_DATA = JSON.parse(JSON_DATA);
            // Get buttons and axes
            for (var i = 0; i < (gamepadChoices.length + axesChoices.length); i++) {
                if (i < gamepadChoices.length) {
                    var key = GAMEPAD_DATA['gamepadChoices' + i];
                    gamepadChoices[i] = key;
                    document.getElementById('gamepad' + (i + 1)).value = key;
                }
                if (i >= gamepadChoices.length) {
                    var key = GAMEPAD_DATA['axesChoices' + (i - gamepadChoices.length)];
                    axesChoices[i - gamepadChoices.length] = key;
                    document.getElementById('gamepad' + (i + 1)).value = key;
                }
            }
        }
    });
    // Set restore button listener
    restoreElement.addEventListener('click', () => {
        for (var i = 0; i < (gamepadChoices.length + axesChoices.length); i++) {
            document.getElementById('gamepad' + (i + 1)).value = -1;
        }
        gamepadChoices = [-1, -1, -1, -1, -1, -1, -1, -1];
        axesChoices = [-1, -1, -1, -1];
    });
})();
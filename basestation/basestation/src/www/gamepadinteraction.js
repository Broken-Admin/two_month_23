var cGamepadID;
var pollInterval = undefined;
var sendInterval = undefined;
var clearInterval = undefined;
var gamepadConnected = false;

var leftTriggerID = 6;
var rightTriggerID = 7;

window.addEventListener("gamepadconnected", (e) => {
    console.log(
      "Gamepad connected at index %d: %s. %d buttons, %d axes.",
      e.gamepad.index,
      e.gamepad.id,
      e.gamepad.buttons.length,
      e.gamepad.axes.length,
    );

    // Update the gamepad ID
    cGamepadID = e.gamepad.id;

    // If a new gamepad has been connected
    if (pollInterval != undefined || sendInterval != undefined || clearInterval != undefined) {
        clearInterval(pollInterval);
        clearInterval(sendInterval);
        clearInterval(clearInterval)
    }

    // Set an interval to regularly poll the controller for
    // updates and to update the display
    pollMilliseconds = 100
    pollInterval = setInterval(pollGamepad, pollMilliseconds);
    
    // Regularly send data
    sendMilliseconds = pollMilliseconds * 2.5;
    sendInterval = setInterval(sendGamepad, sendMilliseconds);

    // Clear the log regularly
    clearMilliseconds = sendMilliseconds * 10;
    clearInterval = setInterval(clearControlLog, clearMilliseconds);
    

    /* Update the display */

    // Items whose visibility needs to change
    notConnectedIndicator = document.getElementById('controller-not-connected');
    controllerIndicators = document.getElementById('controller-indicators');
    
    // Show the controller indicators
    let classes = controllerIndicators.className.trim().split(' ');
    let deleteIndex = classes.indexOf('d-none');
    if(deleteIndex != -1) classes.splice(deleteIndex, 1);
    controllerIndicators.className = classes.join(' ');

    // Hide the not connected message
    classes = notConnectedIndicator.className.trim().split(' ');
    classes.push('d-none');
    notConnectedIndicator.className = classes.join(' ').trim();

});

window.addEventListener("gamepaddisconnected", (e) => {
    // Is this the gamepad that's currently being used, if not exit
    if(e.gamepad.id != cGamepadID) return;
    // If this is the gamepad that's being used perform some updates
    
    // Items whose visibility needs to change
    notConnectedIndicator = document.getElementById('controller-not-connected');
    controllerIndicators = document.getElementById('controller-indicators');
    
    // Show the not connected message
    let classes = notConnectedIndicator.className.split(' ');
    let deleteIndex = classes.indexOf('d-none');
    if(deleteIndex != -1) classes.splice(deleteIndex, 1);
    notConnectedIndicator.className = classes.join(' ');

    // Hide the controls indicators
    classes = controllerIndicators.className.split(' ');
    classes.push('d-none');
    controllerIndicators.className = classes.join(' ');
})

// Button indexes
const button_indexes = ["a","b","x","y","left_bumper","left_bumper","right_bumper","left_trigger","right_trigger","view","menu","d_up","d_down","d_left","d_right"];
const stick_indexes = ["left_hval","left_vval","right_hval","right_vval"];
const trigger_indexes = ["left", "right"];

var gamepadStatus;

function pollGamepad() {
    // https://developer.mozilla.org/en-US/docs/Web/API/GamepadButton
    // https://developer.mozilla.org/en-US/docs/Web/API/Gamepad/axes

    // Gamepad status
    {/* 
    {
        buttons: {
            a: bool,
            b: bool,
            x: bool,
            y: bool,
            left_bumper: bool,
            right_bumper: bool,
            left_trigger: bool,
            right_trigger: bool,
            view: bool,
            menu: bool,
            d_up: bool,
            d_down: bool,
            d_left: bool,
            d_right: bool
        },
        trigger_vals: {
            left_trigger_val: float,
            right_trigger_val: float
        },
        stick_vals: {
            left_hval: float,
            left_vval: float,
            right_hval: float,
            right_vval: float
        }
    }
    */}
    
    // Reset
    gamepadStatus = {
        buttons: {
            a: false,
            b: false,
            x: false,
            y: false,
            left_bumper: false,
            right_bumper: false,
            left_trigger: false,
            right_trigger: false,
            view: false,
            menu: false,
            d_up: false,
            d_down: false,
            d_left: false,
            d_right: false
        },
        trigger_vals: {
            left_trigger_val: 0,
            right_trigger_val: 0
        },
        stick_vals: {
            left_hval: 0,
            left_vval: 0,
            right_hval: 0,
            right_vval: 0
          }
    };

    // Define controller data variable
    let cGamepadState;
    // Update controller data
    var gamepads = navigator.getGamepads ? navigator.getGamepads() : (navigator.webkitGetGamepads ? navigator.webkitGetGamepads() : []);
    for (var i = 0; i < gamepads.length; i++) {
        if (gamepads[i] && gamepads[i].id == cGamepadID) {
            cGamepadState = gamepads[i];
            break;
        }
    }

    // Status reading

    var buttonEls = document.getElementsByClassName("controller-buttons");
    // Only 15 of the available 16 are used
    for (var i=0; i<=15; i++) {
        // Get button status
        var buttonElement = buttonEls[i];
        var gamepadButton = cGamepadState.buttons[i];
        var isPressed = gamepadButton == 1.0;
        var isTrigger = false;
        // Triggers
        if (typeof(gamepadButton) == "object" && (i == leftTriggerID || i == rightTriggerID)) {
            isPressed = gamepadButton.pressed;
            isTrigger = true;
        }
        // Other buttons
        else if (typeof(gamepadButton) == "object") { 
            isPressed = gamepadButton.pressed;
        }
        
        /* Update button JSON for communication */
        gamepadStatus["buttons"][button_indexes[i]] = gamepadButton.pressed;

        // Update the button
        buttonElement.className = "controller-buttons btn text-center";
        
        // If the button is currently being pressed
        if (isPressed) {
            buttonElement.className += " btn-success";
        } else {
            buttonElement.className += " btn-info";
        }

        // Triggers
        if (isTrigger) {
            let triggerEls = document.getElementsByClassName("trigger-value");
            if (i == leftTriggerID) {
                triggerEls[0].innerHTML = i + ": " + gamepadButton.value;
                triggerEls[0].setAttribute("value", gamepadButton.value);
            }
            else if (i == rightTriggerID) {
                triggerEls[1].innerHTML = i + ": " + gamepadButton.value;
                triggerEls[1].setAttribute("value", gamepadButton.value);
            }
            /* Update trigger value JSON for communication */
            let offsetIndex = i - leftTriggerID;
            gamepadStatus["trigger_vals"][trigger_indexes[offsetIndex]] = gamepadButton.value.toFixed(4);
        }
    }

    let axes = document.getElementsByClassName("controller-axis");
    // axes[0] horizontal left
    // axes[1] vertical left
    // axes[2] horizontal right
    // axes[3] vertical right
    // Right = increase
    // Left = decrease

    for (let i=0; i<cGamepadState.axes.length; i++) {
        let axis = axes[i];
        // Update the innerhtml of the meter
        axis.innerHTML = i + ": " + cGamepadState.axes[i].toFixed(4);
        // Update the bar length
        axis.setAttribute("value", cGamepadState.axes[i]);
        /* Update stick value JSON for communication */
        gamepadStatus["stick_vals"][stick_indexes[i]] = cGamepadState.axes[i].toFixed(4);
    }
}

function sendGamepad() {
    // Make use of JSON library to convert object to a string
    let statusString = JSON.stringify(gamepadStatus)
    socket.emit("control", statusString);
}

function clearControlLog() {
    logEl = document.getElementById('/rover_one/control_data');
    logEl.innerHTML = "";
}

// Quality of life
// https://developer.mozilla.org/en-US/docs/Web/API/Gamepad/vibrationActuator
var cGamepadID;
var pollInterval = undefined;
var gamepadConnected = false;

var leftTriggerID = 6;
var rightTriggerID = 7;

// https://developer.mozilla.org/en-US/docs/Web/API/window/requestAnimationFrame
var AnimationCallbackAssign = window.mozRequestAnimationFrame ||
  window.webkitRequestAnimationFrame ||
  window.requestAnimationFrame;

window.addEventListener("gamepadconnected", (e) => {
    console.log(
      "Gamepad connected at index %d: %s. %d buttons, %d axes.",
      e.gamepad.index,
      e.gamepad.id,
      e.gamepad.buttons.length,
      e.gamepad.axes.length,
    );
    cGamepadID = e.gamepad.id;
    if (pollInterval != undefined) {
        clearInterval(pollInterval)
    }

    // Regularly poll the controller
    pollInterval = setInterval(pollGamepad, 100);

    // Update the display

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

function pollGamepad() {
    // https://developer.mozilla.org/en-US/docs/Web/API/GamepadButton
    /* for(buttonIndex in cGamepadState.buttons) {
        console.log(`cGamepadState.buttons[${buttonIndex}]: ${cGamepadState.buttons[buttonIndex]}`)
    } */
    // https://developer.mozilla.org/en-US/docs/Web/API/Gamepad/axes
    /* for(axisIndex in cGamepadState.axes) {
        console.log(`cGamepadState.axes[${axisIndex}]: ${cGamepadState.axes[axisIndex]}`)
    } */

    // Define controller data variable
    let cGamepadState;
    // Update controller data
    var gamepads = navigator.getGamepads ? navigator.getGamepads() : (navigator.webkitGetGamepads ? navigator.webkitGetGamepads() : []);
    for (var i = 0; i < gamepads.length; i++) {
        if (gamepads[i] && gamepads[i].id == cGamepadID) {
            cGamepadState = gamepads[i];
        }
    }

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
        // Update the button
        buttonElement.className = "controller-buttons btn text-center";
        // If the button is currently being pressed
        if (isPressed) {
            buttonElement.className += " btn-success";
        } else {
            buttonElement.className += " btn-info";
        }
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
        }
    }

    let axes = document.getElementsByClassName("controller-axis");
    for (let i=0; i<cGamepadState.axes.length; i++) {
        
        let axis = axes[i];
        // Update the innerhtml of the meter
        axis.innerHTML = i + ": " + cGamepadState.axes[i].toFixed(4);
        // Update the bar length
        axis.setAttribute("value", cGamepadState.axes[i]);
    }
}

// Quality of life
// https://developer.mozilla.org/en-US/docs/Web/API/Gamepad/vibrationActuator
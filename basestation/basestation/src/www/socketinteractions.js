// The `socket` has already been initalized by the page
socket.on('status', processStatus);

function codeAppend(elementID, text, elementColorClass="", elementClass="text-wrap") {
    // Create elements and append them to the provided element

    // Code block element
    parentEl = document.getElementById(elementID);
    // If the parent doesn't exist
    if (!parentEl) return;

    // Create an element
    messageEl = document.createElement('span');
    // Update the text
    messageEl.innerText = text;
    // Update the class
    messageEl.className = `${elementClass} ${elementColorClass}`;
    // Add the element to the DOM
    parentEl.innerHTML = `${messageEl.outerHTML}\n${parentEl.innerHTML}`;
}

/* 
<span class="text-danger">[ERROR]</span>
<span class="text-success text-wrap">[GOOD]</span>
<span class="text-info text-wrap">[INFO]</span>
<span class="text-warning text-wrap">[WARNING]</span>
*/

// ERROR GOOD INFO WARNING

// Process status data sent Pico -(serial)> Libre -(ROS2)> Basestation node
// -(Socket.io)> Basestation web
function processStatus(msg) {
    let statusTag = msg.split(' ')[0] // Split by space, grab the first word
        .replace(/(\[|\])/g, '') // Remove "[" and "]"
        .toLowerCase(); // Change to lowercase
    // Assume that it is of the correct format and perform checks
    let statusColorClass = 'text-';
    switch (statusTag) {
        case 'error':
            statusColorClass += 'danger';
            break;
        case 'good':
            statusColorClass += 'success';
            break;
        case 'info':
            statusColorClass += 'info';
            break;
        case 'warning':
            statusColorClass += 'warning';
            break;
        // Default to the info tag if there is no
        // valid status code provided
        default:
            statusColorClass += 'info';
            break;
    }

    codeAppend('/rover_one/pico_status', msg, statusColorClass);
}

// socket.on('control_return', processControlData);

function processControlData(msg) {
    codeAppend('/rover_one/control_data', msg);
}
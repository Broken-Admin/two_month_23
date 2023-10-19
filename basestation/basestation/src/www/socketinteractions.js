// The `socket` has already been initalized by the page
socket.on('status', processStatus);


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
    let statusClass = 'text-'; // Begin the class type
    console.log(statusTag);
    switch (statusTag) {
        case 'error':
            statusClass += 'danger';
            break;
        case 'good':
            statusClass += 'success';
            break;
        case 'info':
            statusClass += 'info';
            break;
        case 'warning':
            statusClass += 'warning';
            break;
        // Default to the info tag if there is no
        // valid status code provided
        default:
            statusClass += 'info';
            break;
    }
    // Create elements and append them to the `/rover_one/pico_status`
    // code block
    statusEl = document.getElementById('/roverone/pico_status');
    // Create an element
    messageEl = document.createElement('span');
    messageEl.className = `${statusClass} text-wrap`;
    messageEl.innerText = msg;
    // Add the element to the DOM
    statusEl.innerHTML += `${messageEl.outerHTML}\n`;
}

socket.on('control', processControlData);

function processControlData() {
    
}
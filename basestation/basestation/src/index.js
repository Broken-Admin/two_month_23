const rclnodejs = require('rclnodejs');
const express = require('express');
const app = express();
const http = require('http');
// Create an http server using the expess application
const server = http.createServer(app);
// Socket.io integration
const { Server } = require("socket.io");
const io = new Server(server);

// Special Basestation class extending the Node
class Basestation extends rclnodejs.Node {
  constructor() {
    super('Basestation');

    // Create ROS2 publisher
    // Parsed by Libre node
    // Parsed data via serial to the Pico
    this.controls = this.createPublisher(
      'std_msgs/msg/String',
      '/rover_one/control_data'
    );

    // Create a ROS2 subscriber
    // Output from the Pico, parsed by the Libre
    // Libre parsed data sent via ROS2
    this.status = this.createSubscription(
      'std_msgs/msg/String',
      '/rover_one/pico_status',
      this.handler
    );

    /* this.echo = this.createSubscription(
      'std_msgs/msg/String',
      '/rover_one/control_data',
      this.echoHandler
    ); */
  }

  // Handle sending data
  // String data
  // Create a format
  publish(data) {
    this.controls.publish(data);
  }

  // Subscriber handler / Pico return data 
  // Integrate socket.io
  handler(msg) {
    console.log(`Pico reply: ${msg.data}`);
    io.emit('status', msg.data);
  }

  // Handles output from this node to be echoed
  // All control data sent from this node is processed here
  // per the this.echo subscription
  echoHandler(msg) {
    // console.log(`Data was sent on the control topic: ${msg.data}`);
    io.emit('control_return', msg.data);
  }
}


// Initalize the ROS2 node for use in the socket
var basestationNode;

// ROS2 Node handling
async function ROS2_handle() {
  // Initalize the ROS2 master
  await rclnodejs.init();

  basestationNode = new Basestation();
  // Test data publish
  basestationNode.publish('ROS2 published data');

  // Initialize the ROS2 node
  basestationNode.spin();

  console.log('Use this command to view the node\'s published messages: ros2 topic echo /rover_one/control_data std_msgs/msg/String');
}

// ROS2 handler initialization
(async function main() {
  ROS2_handle();
}()).catch(() => {
  process.exitCode = 1;
});

/* WEB SOCKET HANDLERS */
/*
 * All web page hosting performed
 * after the ROS2 node has been intialized and spun
*/
/***********************/

// Initialize the static web page, updated via the socket connection
app.use(express.static(__dirname + '/www'));

// Socket handlers
io.on('connection', (socket) => {
  console.log('Connection');
  // Let the rover know the basestation is connected
  
  // Disconnection event handler
  socket.on('disconnect', () => {
    console.log('Disconnection.');
    // Let the rover know the basestation is disconnected
  });

  // Returned controller data update
  socket.on('control', (data) => {
    // console.log(`Received data from the web page: ${data}`);
    basestationNode.publish(data);
  });
});

// Listen for connections on the public IP address
// Serve the used web directory
server.listen(3000, '0.0.0.0', () => {
  console.log('listening on *:3000');
});
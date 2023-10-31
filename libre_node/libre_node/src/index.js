const rclnodejs = require('rclnodejs');
const { SerialPort } = require('serialport')

class LibreROS extends rclnodejs.Node {
    constructor() {
        super('Libre');

        // Create ROS2 subscriber
        // Sent by Basestation controls
        this.controls = this.createSubscription(
            'std_msgs/msg/String',
            '/rover_one/control_data',
            this.controlRelay
        );

        // Create ROS2 publisher
        // Sent to Basestation controls
        this.status = this.createPublisher(
            'std_msgs/msg/String',
            '/rover_one/pico_status',
        );
    }

    controlRelay(msg) {
        console.log(msg.data);
    }

    picoPublish(data) {
        this.status.publish(data);
    }
}

// Create a node that publishes a msg to the topic 'foo' every 1 second.
// View the topic from the ros2 commandline as shown below:
//    ros2 topic echo foo std_msgs/msg/String
async function libre_node() {
    await rclnodejs.init();

    // Initalize ROS2 subscriber and publisher
    libreNode = new LibreROS()

    libreNode.spin();

    setInterval(() => {
        console.log("Sending info message to basestation");
        libreNode.picoPublish('[INFO] Basestation Connection Message');
    }, 5000);

    // Poll device serial ports
    let ports;
    await SerialPort.list().then((data, err) => {
        // JSON object array
        /*
        {
            path: 'absolute /dev/ path',
            ... irrelevant data
        }
        */
        ports = data;
    })
}

// ROS2 handler initialization
(async function main() {
    libre_node();
}()).catch(() => {
    process.exitCode = 1;
});

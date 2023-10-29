import rclpy
from rclpy.node import Node

import serial
import sys
import threading
import glob
import json

from std_msgs.msg import String

class SerialRelay(Node):
    joint1_angle = 0
    joint2_angle = 0
    joint3_angle = 0
    joint4_angle = 0
    joint5_angle = 0
    joint6_angle = 0

    def __init__(self):
        # Initalize node with name
        super().__init__("serial_publisher")

        # Create a publisher to publish any output the pico sends
        self.publisher = self.create_publisher(String, '/rover_one/pico_status', 10) 

        # Create a subscriber to listen to any commands sent for the pico
        self.subscriber = self.create_subscription(String, '/rover_one/control_data', self.processController, 10)

        # Serial conneciton
        self.sef = serial.Serial();
        return

        # Loop through all serial devices on the computer to check for the pico
        self.port = None
        ports = SerialRelay.list_serial_ports()
        for port in ports:
            try:
                # connect and send a ping command
                self.ser = serial.Serial(port, timeout=1)
                ser.write(b"ping\n\r")
                response = ser.read_until("\n")

                # if pong is in response, then we are talking with the pico
                if b"pong" in response:
                    self.port = port
                    print(f"Found pico at {self.port}!")
                    break
            except:
                pass
        
        if self.port is None:
            print("Unable to find pico... please make sure it is connected.")
            sys.exit(1)
        
        self.ser = serial.Serial(self.port, 115200)

        # self.ser.write(bytes("servo 1 0", "utf8"));
        self.ser.write(bytes("servo 2 0", "utf8"));
        self.ser.write(bytes("servo 3 0", "utf8"));
        self.ser.write(bytes("servo 4 0", "utf8"));
        # self.ser.write(bytes("servo 5 0", "utf8"));
        self.ser.write(bytes("servo 6 0", "utf8"));

    def run(self):
        # This thread makes all the update processes run in the background
        thread = threading.Thread(target=rclpy.spin, args={self}, daemon=True)
        thread.start()
        
        try:
            while rclpy.ok():
                # Check the pico for updates
                self.read_pico()

        except KeyboardInterrupt:
            sys.exit(0)
        

    def read_pico(self):
        if(self.sef is None):
            return
        output = str(self.ser.readline(), "utf8")
        # If received output
        if output:
            print(f"[Pico] {output}", end="")
            # Create a string message object
            msg = String()

            # Set message data
            msg.data = f"[INFO] {output}"

            # Publish data
            self.publisher.publish(msg)
            #print(f"[Pico] Publishing: {msg}")
    
    def processController(self, msg):
        print(msg.data)
        
        controller_data = json.loads(msg.data)
        # Joint 6 end effector
        if(control_data["buttons"]["left_trigger"]):
            joint6_angle += control_data["trigger_vals"]["left_trigger"]
        if(control_data["buttons"]["right_trigger"]):
            joint6_angle -= control_data["trigger_vals"]["right_trigger"]
        # Joint 2 & 3
        joint3_angle -= control_data["stick_vals"]["left_vval"]
        joint2_angle -= control_data["stick_vals"]["right_vval"]
        # Joint 5 end effector rot
        #define CONT_MIN 72 // Clockwise
        #define CONT_MAX 113 // Counterclockwise
        # [72, 113]
        left_hval = round(control_data["stick_vals"]["left_hval"] * 100)
        if(left_hval > 10 or left_hval < -10):
            joint5_angle = (113 - 72) * (left_hval / 100) + 72
        else:
            joint5_angle = 0
        # joint 4 end effector elevator
        if(control_data["buttons"]["left_bumper"]):
            joint4_angle += 0.2
        if(control_data["buttons"]["right_bumper"]):
            joint4_angle -= 0.2
        # joint 1 rotation
        if(control_data["buttons"]["d_left"]): #clockwise, left
            joint1_angle = 72
        elif(control_data["buttons"]["d_right"]): #counterclockwise, right
            joint1_angle = 113
        else:
            joint1_angle = 0

        
        self.ser.write(bytes(f"servo 1 {joint1_angle} \n\rservo 2 {joint2_angle} \n\rservo 3 {joint3_angle} \n\r", "utf-8"))
        self.ser.write(bytes(f"servo 4 {joint4_angle} \n\rservo 5 {joint5_angle} \n\rservo 6 {joint6_angle} \n\r", "utf-8"))

    @staticmethod
    def list_serial_ports():
        # This code only works on linux systems
        # See here for other OSes: https://stackoverflow.com/a/14224477
        return glob.glob("/dev/tty[A-Za-z]*")
        

def main(args=None):
    rclpy.init(args=args)

    serial_pub = SerialRelay()
    serial_pub.run()


if __name__ == '__main__':
    main()
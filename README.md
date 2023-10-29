# Rover Two Month 2023 Software Stack
* Developed by Alexander Resurreccion for the University of Alabama in Huntsville's Space Hardware Club introductory Two Month project
* Requirements include the development of mechanical and electrical hardware as well as a ROS2 connected control station (basestation).

# Structure
* `/basestation/basestation`: Code implementing a ROS2 integrated basestation in Node.js
     * rclnodejs
     * socket.io
     * express
     * http
* `/embedded/embedded`: Embedded Arduino code for the Pi Pico
     * ArduinoCore-mbed RP2040 board manager
          * UART
          * I2C
          * PWM
          * Digital
     * Adafruit Sensor libraries
* `ros2_ws/src/pico_relay`: ROS2 and UART (to Pi Pico) communication node developed for the ROC-RK3328-CC
     * rclpy
     * Python serial library

## Notes
1. I've included the source file as a .ino so you can just use Arduino IDE for programming your pico
2. If you 'd like to use PlatformIO (with VS Code)
     If you run into any problems with coding in PlatformIO, try making sure there are no spaces in your folder path.
     All you should need to do to get the .ino functioning in Platform is change it to .cpp and include the <Arduino.h> library

## Credit
Credit for the bootstrapping goes to Areeb Mohammed and Tristan McGinnis

# Basestation Outline
* ROS2 Node
* Web control interface host
    * Socket.io communication between host file and page
# ROS2 Command Line Interaction
### Info
* In order for these commands to be run the ROS2 package must be built with `colcon` and run with `ros2 launch basestation basestation.launch.py`
### Command Output: `/roverone/control_data`
* View sent data: `ros2 topic echo /roverone/control_data`
* Send data `ros2 topic pub -1 /roverone/control_data std_msgs/msg/String "{data: 'CONTROL DATA FORMAT'}"`
### Pico Return Data: `/roverone/pico_status`
* View sent data: `ros2 topic echo /roverone/pico_status`
* Send data: `ros2 topic pub -1 /roverone/pico_status std_msgs/msg/String "{data: '[STATUS] Information'}"`
    * Available `[STATUS]`: `ERROR`, `GOOD`, `INFO`, `WARNING`
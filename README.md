# arduino_servo
This  is for controling rc servo motors with arduino. 16 channel pwm controller by
adafruit is used on the arduino uno. Also ROS is used for communication.

Usage codes:

rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0
rostopic pub servo std_msgs/UInt16  <angle>

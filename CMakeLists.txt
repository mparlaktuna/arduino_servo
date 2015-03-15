cmake_minimum_required(VERSION 2.8.3)
project(arduino_servo)

find_package(catkin REQUIRED COMPONENTS
  rosserial_arduino
  rosserial_client
  std_msgs
  beginner_tutorials
)

rosserial_generate_ros_lib(
	PACKAGE rosserial_arduino
	SCRIPT make_libraries.py
)

rosserial_configure_client(
	DIRECTORY firmware
	TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}	
)



catkin_package()

rosserial_add_client_target(firmware motor_servo ALL)
rosserial_add_client_target(firmware motor_servo-upload)


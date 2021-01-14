cmake_minimum_required(VERSION 2.8.3)
project(arduino-ros)

find_package(catkin REQUIRED COMPONENTS
  rosserial_arduino
  rosserial_client
)

catkin_package()

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

rosserial_configure_client(
  DIRECTORY test
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(test hello ALL)
rosserial_add_client_target(test hello-upload)

##########################################
## Install launch files
##########################################
install(DIRECTORY launch
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
FILES_MATCHING PATTERN "*.launch"
)
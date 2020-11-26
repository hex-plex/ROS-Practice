cmake_minimum_required(VERSION 3.0.2)

include_directories(${ROS_LIB_DIR})

add_definitions(-DUSB_CON)

generate_arduino_firmware(hello
  SRCS chatter.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD uno
  PORT /dev/ttyACM0

)

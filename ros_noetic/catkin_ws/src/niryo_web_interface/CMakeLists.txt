cmake_minimum_required(VERSION 3.0.2)
project(niryo_web_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

## Add service files
add_service_files(
  FILES
  PickObject.srv
)

## Generate messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime rospy std_msgs
)
find_package(Python3 3.8 EXACT REQUIRED COMPONENTS Interpreter)
set(PYTHON_EXECUTABLE /usr/bin/python3.8)

catkin_install_python(PROGRAMS
  scripts/robot_control_node.py
  scripts/web_camera_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

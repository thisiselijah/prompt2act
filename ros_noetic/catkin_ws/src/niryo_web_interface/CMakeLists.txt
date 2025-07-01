cmake_minimum_required(VERSION 3.0.2)
project(niryo_web_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/web_interface_node.py
  scripts/robot_control_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

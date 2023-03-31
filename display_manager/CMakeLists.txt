cmake_minimum_required(VERSION 3.0.2)
project(display_manager)

find_package(catkin REQUIRED COMPONENTS
 rospy
 std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
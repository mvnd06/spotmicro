cmake_minimum_required(VERSION 3.0.2)
project(oled_display)

find_package(catkin REQUIRED COMPONENTS
 rospy
 std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/oled_display.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
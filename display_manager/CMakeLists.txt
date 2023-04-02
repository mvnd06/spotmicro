cmake_minimum_required(VERSION 3.0.2)
project(display_manager)

find_package(catkin REQUIRED COMPONENTS
 rospy
 std_msgs
 system_monitor
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/display_manager.py
  scripts/system_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    config
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
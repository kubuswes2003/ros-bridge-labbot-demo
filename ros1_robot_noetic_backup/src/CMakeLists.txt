cmake_minimum_required(VERSION 3.0.2)
project(labbot_mock)

# Znajdź komponenty catkin
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
)

# Zadeklaruj pakiet catkin
catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs
)

# Oznacz skrypty Pythona jako executable
catkin_install_python(PROGRAMS
  mock_robot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
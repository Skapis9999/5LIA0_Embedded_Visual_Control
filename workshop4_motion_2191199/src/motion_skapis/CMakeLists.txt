cmake_minimum_required(VERSION 3.0.2)
project(motion_skapis)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
  dualImage.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

# Declare a catkin package
catkin_package()

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install Python scripts
catkin_install_python(PROGRAMS
  src/processing_node.py
  src/publisher.py
  src/subscriber.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
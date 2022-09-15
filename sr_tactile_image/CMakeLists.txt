cmake_minimum_required(VERSION 3.12)
project(sr_tactile_image)

# find catkin and any catkin packages
find_package(
	catkin REQUIRED
	COMPONENTS
		rospy
		# sr_robot_commander
		# sr_utils
		# sr_robot_msgs
		# sr_hand
)

# declare a catkin package
catkin_package(
	INCLUDE_DIRS
	LIBRARIES
		sr_test
	CATKIN_DEPENDS
		rospy
	DEPENDS
)

## process setup.py
catkin_python_setup()

## install python nodes
catkin_install_python(
  PROGRAMS 
    nodes/sr_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
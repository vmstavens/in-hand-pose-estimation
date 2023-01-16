cmake_minimum_required(VERSION 3.12)
project(sr_tactile_perception)

# find catkin and any catkin packages
find_package(
	catkin REQUIRED
	COMPONENTS
		rospy
		ros_utils_py
		# sr_robot_commander
		# ros_utils_py
		# sr_robot_msgs
		# sr_hand
)

# declare a catkin package
catkin_package(
	INCLUDE_DIRS
	LIBRARIES
		${PROJECT_NAME}
	# sr_test
	CATKIN_DEPENDS
		rospy
		ros_utils_py
	DEPENDS
)

## process setup.py
catkin_python_setup()

## install python nodes
catkin_install_python(
  PROGRAMS 
    nodes/echo.py
    nodes/test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
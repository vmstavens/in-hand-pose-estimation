cmake_minimum_required(VERSION 3.12)
project(sr_tactile_perception)

# find catkin and any catkin packages
find_package(
	catkin REQUIRED
	COMPONENTS
		rospy
		ros_utils_py
		shadow_hand
		gazebo_msgs
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
		shadow_hand
		gazebo_msgs
	DEPENDS
)

## process setup.py
catkin_python_setup()

## install python nodes
catkin_install_python(
  PROGRAMS 
    scripts/sr_tactile_perception.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
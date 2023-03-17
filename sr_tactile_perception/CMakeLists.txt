cmake_minimum_required(VERSION 3.12)
project(sr_tactile_perception)

# find catkin and any catkin packages
find_package(
	catkin REQUIRED
	COMPONENTS
		rospy
		geometry_msgs
		shadow_hand
		gazebo_msgs
		ros_utils_py
)

# declare a catkin package
catkin_package(
	INCLUDE_DIRS
	LIBRARIES
		${PROJECT_NAME}
	# sr_test
	CATKIN_DEPENDS
		rospy
		geometry_msgs
		shadow_hand
		gazebo_msgs
		ros_utils_py
	DEPENDS
)

## process setup.py
catkin_python_setup()

## install python nodes
catkin_install_python(
  PROGRAMS 
    src/sr_tactile_perception.py
    src/test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
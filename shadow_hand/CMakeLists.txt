cmake_minimum_required(VERSION 3.12)
project(shadow_hand)

# find catkin and any catkin packages
find_package(
	catkin REQUIRED
	COMPONENTS
		rospy
		ros_utils_py
)

# declare a catkin package
catkin_package(
	INCLUDE_DIRS
	LIBRARIES
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
	src/ShadowHand/ShadowHand.py
    examples/example.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
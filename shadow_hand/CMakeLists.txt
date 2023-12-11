cmake_minimum_required(VERSION 3.12)
project(shadow_hand)

# find catkin and any catkin packages
find_package(
	catkin REQUIRED
	COMPONENTS
		rospy
		ros_utils_py
		gazebo_msgs
)

# declare a catkin package
catkin_package(
	INCLUDE_DIRS
	LIBRARIES
	CATKIN_DEPENDS
		rospy
		ros_utils_py
		gazebo_msgs
	DEPENDS
)

## process setup.py
catkin_python_setup()

## install python nodes
catkin_install_python(
  PROGRAMS 
    examples/hand_control_demo.py
    examples/robot_control_demo.py
    examples/debug.py

	
    # src/shadow_hand/ShadowFinger.py
    # src/shadow_hand/ContactPoint.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
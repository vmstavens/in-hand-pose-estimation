cmake_minimum_required(VERSION 3.12)
project(sr_tactile_image)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find catkin and any catkin packages
find_package(
	catkin REQUIRED
	COMPONENTS
		roscpp
		sr_robot_msgs
		sr_hand
)

# find system libraries
find_package(Eigen3 REQUIRED)

# declare a catkin package
catkin_package(
	INCLUDE_DIRS
		include
		${EIGEN3_INCLUDE_DIR}
	LIBRARIES
		sr_test
	CATKIN_DEPENDS
		roscpp
	DEPENDS
		EIGEN3 # uppercase due to catkin_package() warning
)

# additional locations of header files
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
)
################################################################################################

# libraries
# REMEMBER TO ADD TO INSTALL TARGETS (bottom of file) !!!

# add_library(sr_tactile_image_test src/sr_tactile_image_test.cpp)
# target_link_libraries(sr_tactile_image_test ${catkin_LIBRARIES})

# ################################################################################################

# nodes (executables)

add_executable(sr_test src/sr_test.cpp)
target_link_libraries(sr_test ${catkin_LIBRARIES})

################################################################################################

# install targets

## executables and/or libraries for installation
install(
	TARGETS
		sr_test
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## install header files
install(
	DIRECTORY
		include/sr_test/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## install gazebo stuff
install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################################

# experiments (executables from /tests)

message(STATUS "Addings experiments from 'tests/*/*.cpp'...")
file(GLOB files CONFIGURE_DEPENDS "tests/*/*.cpp")
foreach(file ${files})
	get_filename_component(name ${file} NAME_WE)
	message(STATUS "Found experiment: ${name}...")
	add_executable(${name} ${file})
	target_link_libraries(${name} ${catkin_LIBRARIES} teleop_grasp)
endforeach()
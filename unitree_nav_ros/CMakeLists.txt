cmake_minimum_required(VERSION 3.0.2)
project(unitree_nav_ros)

add_compile_options(-std=c++11)
add_compile_options(-std=c++14)
set(OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4/)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	tf
	airsim_ros_pkgs
	OpenCV
	cv_bridge
	cmake_modules
  sensor_msgs
  image_transport
)


add_library(
	spline_planner
	src/spline_planner.cpp
)

catkin_package(
	LIBRARIES visionutil rosutil
	INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS}  ${OpenCv_INCLUDE_DIRS})

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_executable(unitree_nav_ros src/UnitreeNav.cc src/UnitreeNav_node.cc)

target_link_libraries(unitree_nav_ros

	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	spline_planner
)

cmake_minimum_required(VERSION 2.8.3)
project(plane_ground_filter)
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
	pcl_ros
	roscpp
	sensor_msgs
	tf2_ros
	tf
	tf2_eigen
)

find_package(PCL 1.7 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

add_executable(${PROJECT_NAME}_node src/ src/plane_ground_filter.cpp)


target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
